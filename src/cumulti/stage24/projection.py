"""Explicit forward projection; never fills unseen geometry or missing calibration."""
import cv2
import numpy as np

def project(points,K,D,T_camera_from_lidar,width,height,*,verified=False,max_sync_ms=50,sync_ms=0):
    if not verified:raise ValueError('Complete measured optical-frame calibration chain is required')
    p=np.asarray(points,float);T=np.asarray(T_camera_from_lidar,float)
    if T.shape!=(4,4) or not np.isfinite(T).all() or not np.allclose(T[3],[0,0,0,1]):raise ValueError('Invalid SE3')
    if not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-7) or abs(np.linalg.det(T[:3,:3])-1)>1e-7:raise ValueError('Invalid SO3')
    valid=np.isfinite(p).all(1)
    if abs(sync_ms)>max_sync_ms:return np.zeros(len(p),bool),np.zeros((len(p),2),int)
    xyz=p@T[:3,:3].T+T[:3,3];valid&=xyz[:,2]>.1
    uv=np.zeros((len(p),2),int);indices=np.flatnonzero(valid)
    if len(indices):
        projected=cv2.projectPoints(xyz[indices],np.zeros(3),np.zeros(3),np.asarray(K,float),np.asarray(D,float))[0].reshape(-1,2)
        uv[indices]=np.floor(projected+.5).astype(int)
    valid&=(uv[:,0]>=0)&(uv[:,0]<width)&(uv[:,1]>=0)&(uv[:,1]<height)
    # Visible nearest sample per exact pixel. No hole filling or assumed visibility.
    ids=np.flatnonzero(valid);pix=uv[ids,1]*width+uv[ids,0]
    order=np.lexsort((ids,xyz[ids,2],pix));ordered=ids[order];pixel_ordered=pix[order]
    keep=np.r_[True,np.diff(pixel_ordered)!=0] if len(ids) else np.array([],bool)
    mask=np.zeros(len(p),bool);mask[ordered[keep]]=True
    return mask,uv

def tests():
    K=np.array([[100.,0,100],[0,100,100],[0,0,1]])
    points=np.array([[0,0,2],[0,0,3],[0,0,-1],[100,0,1],[np.nan,0,1],[.2,0,2]])
    mask,uv=project(points,K,np.zeros(5),np.eye(4),200,200,verified=True)
    assert mask.tolist()==[True,False,False,False,False,True]
    assert uv[5].tolist()==[110,100]
    assert not project(points,K,np.zeros(5),np.eye(4),200,200,verified=True,sync_ms=51)[0].any()
    try:project(points,K,np.zeros(5),np.eye(4),200,200)
    except ValueError:pass
    else:raise AssertionError('Missing calibration not rejected')
    T=np.eye(4);T[0,3]=.2
    m,u=project(np.array([[0,0,2.]]),K,np.zeros(5),T,200,200,verified=True)
    assert m[0] and u[0].tolist()==[110,100]
    return dict(status='PASS',synthetic_only=True,cases=['front/back/FoV/NaN','nearest-depth occlusion','strict synchronization cutoff','missing calibration rejected','T_target_source direction'])
