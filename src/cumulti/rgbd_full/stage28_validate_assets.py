"""Audit Stage28 artifact identities, temporal prefixes and coordinate conventions."""
import numpy as np

from common import SITE, dump, sha
from stage28_gravity import points
from stage28_lidar import BASE, DEST, REPORT, ROBOTS, lookup, matrices
from stage26_manifest import read_csv


def is_rigid(matrix):
    r=np.array(matrix)[:3,:3]
    assert np.isfinite(matrix).all()
    assert np.allclose(r.T@r,np.eye(3),atol=1e-8) and abs(np.linalg.det(r)-1)<1e-8
    assert np.allclose(np.array(matrix)[3],[0,0,0,1])
    return r


def main():
    report={}
    alignment=lookup(REPORT/'gravity_alignment.json')
    stable=read_csv(BASE/'12d_clean_backend/clean_all75_gtsam_trajectory.csv')
    frozen_rgbd=lookup(BASE/'27_continuous_rgbd/chunk_manifest.json')
    for sensor,root in (('lidar',DEST),('rgbd',SITE/'continuous')):
        manifest=lookup(root/'manifest.json')
        trajectories=lookup(root/manifest['trajectory'])
        if sensor=='rgbd':
            assert frozen_rgbd==manifest
        for chunk in manifest['chunks']:
            path=root/chunk['file']
            assert path.stat().st_size==chunk['bytes']==15*chunk['points'] and sha(path)==chunk['sha256']
            assert np.isfinite(points(path)[0]).all()
            t,n=np.array(chunk['steps']).T
            assert (np.diff(t)>0).all() and (np.diff(n)>=0).all()
            assert t[0]==chunk['start'] and t[-1]==chunk['end'] and n[-1]==chunk['points']
        for row in manifest['transforms']:
            tm=row.get('T_display_from_epoch',row.get('T_display_from_raw_ekf'))
            if tm is not None:
                is_rigid(tm)
        for robot in (*ROBOTS,'combined'):
            artifact=alignment[sensor][robot]['artifact']
            path=REPORT/artifact['path']
            assert path.stat().st_size==artifact['bytes'] and sha(path)==artifact['sha256']
            r=is_rigid(alignment[sensor][robot]['T_ground_from_display'])
            assert np.allclose(np.linalg.norm(r@[1,2,3]),np.linalg.norm([1,2,3]))
            assert np.linalg.norm(r@[0,0,1])>.99
        for robot,rows in trajectories.items():
            ts=np.array([r[0] for r in rows])
            assert (np.diff(ts)>0).all() and np.isfinite(rows).all()
            if sensor=='lidar':
                saved=[r for r in stable if r['robot_id']==robot]
                tail=np.array(rows[-len(saved):])
                assert tail[:,1].tolist()==[int(r['keyframe_id']) for r in saved]
                frozen=matrices(saved,['tx','ty','tz','qx','qy','qz','qw'])
                assert np.allclose(tail[:,3:6],frozen[:,:3,3],atol=1e-10)
                from scipy.spatial.transform import Rotation
                assert np.allclose(Rotation.from_quat(tail[:,6:]).as_matrix(),frozen[:,:3,:3],atol=1e-10)
        report[sensor]=dict(chunk_count=len(manifest['chunks']),points=manifest['points'],
                            frozen_rgbd_chunks_preserved=sensor=='rgbd',
                            frozen_lidar_tail_pose_equality=sensor=='lidar',
                            rigid_gravity_transforms=True,finite_points_and_poses=True)
    dump(REPORT/'asset_validation.json',dict(status='PASS',sensors=report))


if __name__=='__main__':
    main()
