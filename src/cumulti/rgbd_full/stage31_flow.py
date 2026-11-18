"""Supplementary depth-masked visual tracking; frozen geometric gates unchanged."""
import cv2
import numpy as np

from fusion import rigid

POLICY=dict(detector='Shi-Tomasi on valid measured depth mask',max_corners=2400,
            quality_level=.008,min_distance_px=4,lk_window_px=31,lk_levels=4,
            forward_backward_max_px=1.,ransac_iterations=512,inlier_distance_m=.1,
            min_mutual=60,min_metric=40,min_inliers=30,min_fraction=.5,max_rmse_m=.1,
            purpose='Address sparse ORB features on measured-depth pixels; keep frozen frame split and acceptance gates')


def estimate(left,right,k,d,i,j):
    images=[cv2.cvtColor(frame[0],cv2.COLOR_RGB2GRAY) for frame in (left,right)]
    depths=[frame[1]/1000. for frame in (left,right)]
    mask=((depths[0]>=.3)&(depths[0]<=10)).astype(np.uint8)*255
    uv=cv2.goodFeaturesToTrack(images[0],maxCorners=2400,qualityLevel=.008,minDistance=4,mask=mask)
    if uv is None or len(uv)<40:
        return dict(mutual=0,metric_matches=0,reason='NO_DEPTH_MASK_CORNERS'),None
    kwargs=dict(winSize=(31,31),maxLevel=4,criteria=(cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,40,.01))
    other,status,_=cv2.calcOpticalFlowPyrLK(images[0],images[1],uv,None,**kwargs)
    back,reverse,_=cv2.calcOpticalFlowPyrLK(images[1],images[0],other,None,**kwargs)
    a,b=uv.reshape(-1,2),other.reshape(-1,2)
    keep=(status.ravel()==1)&(reverse.ravel()==1)&(np.linalg.norm(a-back.reshape(-1,2),axis=1)<=1)
    keep&=(b[:,0]>=0)&(b[:,0]<640)&(b[:,1]>=0)&(b[:,1]<400)
    a,b=a[keep],b[keep]
    counts=dict(mutual=len(a),metric_matches=0)
    if not len(a):
        return counts,None
    xyz=[]
    valid=[]
    for coords,zmap in zip((a,b),depths):
        pix=np.floor(coords).astype(int)
        z=zmap[pix[:,1],pix[:,0]]
        rays=cv2.undistortPoints(coords.reshape(-1,1,2),k,d).reshape(-1,2)
        xyz.append(np.column_stack([rays,np.ones(len(rays))])*z[:,None])
        valid.append((z>=.3)&(z<=10))
    okay=valid[0]&valid[1]
    target,source=[points[okay] for points in xyz]
    counts['metric_matches']=len(target)
    if len(target)<12:
        return counts,None
    rng=np.random.default_rng(3100+i*100000+j)
    best=np.zeros(len(target),bool)
    for _ in range(512):
        selected=rng.choice(len(target),3,replace=False)
        x,y=target[selected],source[selected]
        if min(np.linalg.norm(np.cross(v[1]-v[0],v[2]-v[0])) for v in (x,y))<.002:
            continue
        rotation,translation=rigid(y,x)
        inside=np.linalg.norm(source@rotation.T+translation-target,axis=1)<.1
        if inside.sum()>best.sum():
            best=inside
    counts['inliers']=int(best.sum())
    if best.sum()<12:
        return counts,None
    rotation,translation=rigid(source[best],target[best])
    result=np.eye(4)
    result[:3,:3],result[:3,3]=rotation,translation
    error=np.linalg.norm(source@rotation.T+translation-target,axis=1)
    counts.update(fit_fraction=float(best.mean()),fit_inlier_rmse_m=float(np.sqrt(np.mean(error[best]**2))),
                  tracking='forward-backward LK on independent RGB observations with measured depth')
    return counts,result
