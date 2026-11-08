"""Check scientific adapter invariants, artifact identity and actual measured tracks."""
import hashlib,json
import cv2
import numpy as np
from export import OUT,ROOT,align_depth,dump

def run():
    K=np.array([[100.,0,100],[0,100,100],[0,0,1]])
    p=np.array([[0,0,2],[0,0,3],[0,0,-1],[0,0,65.535],[np.nan,0,2],[.4,0,2.]])
    d=align_depth(p,np.eye(3),np.zeros(3),K,np.zeros(5),200,200)
    assert d[100,100]==2 and d[100,120]==2 and np.count_nonzero(d)==2
    shifted=align_depth(np.array([[0.,0,2]]),np.eye(3),np.array([.2,0,0]),K,np.zeros(5),200,200)
    assert shifted[100,110]==2 and np.count_nonzero(shifted)==1
    site=ROOT/'demo/rgbd_camera_mapping';data=json.loads((site/'data.json').read_text())
    manifest=json.loads((OUT/'map_manifest.json').read_text());checks=[]
    for robot,s in data['robots'].items():
        assert len(s['frames'])==60 and s['points']>1000
        assert all(int(s['frames'][i]['rgb_bag_ns'])<int(s['frames'][i+1]['rgb_bag_ns']) for i in range(59))
        assert all(f['bag_sync_ms']<=25 for f in s['frames'])
        exported=json.loads((OUT/robot/'export.json').read_text())
        for frame in exported['frames']:
            depth=cv2.imread(str(ROOT/frame['depth']),cv2.IMREAD_UNCHANGED)
            assert depth.shape==(400,640) and depth.dtype==np.uint16
            good=depth[depth>0];assert len(good) and good.min()>=300 and good.max()<=10000
        for sub in s['submaps']:
            frames=[f for f in s['frames'] if f['tracking_epoch']==sub['epoch']]
            assert all(a['points_end']<=b['points_end'] for a,b in zip(frames,frames[1:]))
            assert frames[-1]['points_end']==sub['points']
            assert len((site/sub['positions']).read_bytes())==sub['points']*12
            assert len((site/sub['colors']).read_bytes())==sub['points']*3
            good=[f for f in frames if f['valid_pose']]
            assert len({f['map_id'] for f in good})<=1
            for frame in good:
                assert np.isfinite(frame['pose']).all() and abs(np.linalg.norm(frame['pose'][3:])-1)<1e-5
        assert s['max_roundtrip_pixel_error']<.1,s['max_roundtrip_pixel_error']
        assert not (OUT/robot/'work/camera_depth.db3').exists()
        checks.append(dict(robot=robot,tracking=s['tracking']['status'],reset_separation='PASS',finite_map='PASS',timestamps='PASS',bag_sync='PASS',
            projection_backprojection_roundtrip_max_px=s['max_roundtrip_pixel_error'],
            measured_alignment_accuracy='NOT_ESTABLISHED_BY_ROUNDTRIP_TEST',depth_accuracy='NOT_GT_EVALUATED',
            unknown_depth='zero / no fill',point_count=s['points']))
    for a in manifest['artifacts']:
        path=ROOT/a['path'];assert path.stat().st_size==a['bytes'] and hashlib.sha256(path.read_bytes()).hexdigest()==a['sha256']
    assert not data['GT'] and not data['LiDAR'] and not data['merged_map']
    tracking_pass=all(s['tracking']['status']=='PASS' for s in data['robots'].values())
    result=dict(status='PASS_FUNCTIONAL_DEMO_TRACKING_PASS' if tracking_pass else 'PASS_FUNCTIONAL_DEMO_TRACKING_PARTIAL',
        full_window_tracking='PASS' if tracking_pass else 'PARTIAL_PREDECLARED_GATE_NOT_MET',
        adapter_synthetic=['nearest-Z','behind camera','saturation','NaN','T_color_from_depth direction'],
        actual_robot_checks=checks,artifact_hashes_checked=len(manifest['artifacts']),
        cross_robot_fusion='NOT_RUN',full_sequence='NOT_RUN',GT_evaluation='NOT_RUN',
        LiDAR_to_camera_mounting_calibration='NOT_REQUIRED_NO_LIDAR_INPUT',
        caveats=data['limitations'])
    dump(OUT/'validation.json',result);print(json.dumps(result),flush=True)

if __name__=='__main__':run()
