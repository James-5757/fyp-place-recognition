"""Check recorded calibration and test a bounded trajectory-only hand-eye hypothesis."""
import argparse
import csv
import re
import sqlite3
import xml.etree.ElementTree as ET
import zipfile
from contextlib import closing
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp

from common import OUT, RAW, dump
from stage26_manifest import write_csv
from stage28_lidar import BASE, REPORT, ROBOTS, lookup, matrices

OFFSETS_MS = (-100,-50,0,50,100)
MOTION_SPACING_S = 4
HOLDOUT_EVERY = 3


def source_audit(robot, scratch, dataset):
    bags = {}
    for kind,data in dataset[robot].items():
        if kind not in ('RGB','DEPTH'):
            continue
        database = scratch/Path(data['archive']['member']).name
        with closing(sqlite3.connect(f'{database.resolve().as_uri()}?mode=ro&immutable=1',uri=True)) as db:
            topics = db.execute('SELECT name,type FROM topics ORDER BY name').fetchall()
        bags[kind] = dict(topics=topics,tf_topics=[name for name,_ in topics if name.rsplit('/',1)[-1] in ('tf','tf_static')])
    imu_name = f'{robot}_main_campus_imu_gps'
    with zipfile.ZipFile(RAW/robot/f'{imu_name}.zip') as archive:
        metadata = archive.read(f'{imu_name}/metadata.yaml').decode()
        imu_topics = re.findall(r'^\s*name:\s*(\S+)\s*$',metadata,re.MULTILINE)
        assert imu_topics
        bags['IMU_GPS'] = dict(topics=imu_topics,tf_topics=[
            name for name in imu_topics if name.rsplit('/',1)[-1] in ('tf','tf_static')])
    with zipfile.ZipFile(RAW.parent/'calib/robot_description.zip') as archive:
        urdf=ET.fromstring(archive.read(f'robot_description/urdf/{robot}.urdf'))
        joint=[]
        for item in urdf.findall('joint'):
            child = item.find('child').get('link')
            parent = item.find('parent').get('link')
            if 'camera' in child or 'camera' in parent or 'os_sensor' in child:
                origin=item.find('origin')
                joint.append(dict(parent=parent,child=child,joint_type=item.get('type'),
                                  origin=dict(origin.attrib) if origin is not None else {}))
        camera_configs=[n for n in archive.namelist() if
                        'realsense' in n.lower() and n.endswith(('.launch.py','.yaml','.json'))]
    return dict(bags=bags,urdf_mounting_joints=joint,recording_specific_driver_configs=camera_configs,
                recorded_T_color_from_depth=dataset[robot]['DEPTH']['extra']['T_color_from_depth'],
                missing='Run-specific camera_link to optical frame origin and rotation')


def relative(poses,a,b):
    return np.linalg.inv(poses[a])@poses[b]


def pairs(camera, lidar, offset):
    origin = int(lidar[0]['timestamp'])
    tl = (np.array([int(r['timestamp']) for r in lidar])-origin)/1e9
    tc = (np.array([int(r['rgb_bag_ns']) for r in camera])-origin)/1e9 + offset
    cam = matrices(camera,['tx','ty','tz','qx','qy','qz','qw'])
    lid = matrices(lidar,['tx','ty','tz','qx','qy','qz','qw'])
    search = np.searchsorted(tl,tc,side='right')
    idx = np.flatnonzero((search>0)&(search<len(tl)))
    assert len(idx)
    gaps = tl[search[idx]]-tl[search[idx]-1]
    near = np.minimum(tc[idx]-tl[search[idx]-1],tl[search[idx]]-tc[idx])
    idx = idx[gaps<=.65]
    interpolated = np.repeat(np.eye(4)[None],len(camera),axis=0)
    interpolated[idx,:3,:3]=Slerp(tl,Rotation.from_matrix(lid[:,:3,:3]))(tc[idx]).as_matrix()
    interpolated[idx,:3,3]=np.column_stack([np.interp(tc[idx],tl,lid[:,d,3]) for d in range(3)])
    windows = {}
    for i in idx:
        key = (int(camera[i]['map_epoch']),int(tc[i]//30))
        windows.setdefault(key,[]).append(i)
    measurements = []
    for key,ids in windows.items():
        previous = ids[0]
        motions=[]
        for i in ids[1:]:
            dt=tc[i]-tc[previous]
            if dt<MOTION_SPACING_S:
                continue
            if dt<=MOTION_SPACING_S+.35:
                a,b=relative(interpolated,previous,i),relative(cam,previous,i)
                if np.linalg.norm(a[:3,3])>.25 and np.linalg.norm(b[:3,3])>.25:
                    motions.append((a,b))
            previous=i
        if len(motions)>=3:
            measurements.append((key,motions))
    stamps = dict(valid_camera_frames=len(camera),bracketed=int(len(search[(search>0)&(search<len(tl))])),
                  eligible_brackets=len(idx),last_minus_first_overlap_s=float(tc[idx[-1]]-tc[idx[0]]),
                  bracket_median_ms=float(np.median(gaps)*1000),nearest_median_ms=float(np.median(near)*1000),
                  nearest_p95_ms=float(np.quantile(near,.95)*1000))
    return measurements,stamps


def transform(vector):
    matrix=np.eye(4)
    matrix[:3,:3]=Rotation.from_rotvec(vector[:3]).as_matrix()
    matrix[:3,3]=vector[3:]
    return matrix


def residual(vector, measurements):
    x=transform(vector)
    a,b=measurements.transpose(1,0,2,3)
    error = np.linalg.inv(a@x)@(x@b)
    return Rotation.from_matrix(error[:,:3,:3]).as_rotvec(),error[:,:3,3]


def fit(windows):
    dataset=np.array([pair for _,motions in windows for pair in motions])
    if len(dataset)<10:
        return None
    def cost(vector):
        rot, trans = residual(vector,dataset)
        return np.column_stack([3*rot,trans]).ravel()
    seeds=([0,0,0],[np.pi/2,0,0],[-np.pi/2,0,0],[0,np.pi/2,0],[0,-np.pi/2,0],[0,0,np.pi])
    bounds=([-4]*3+[-2.5]*3,[4]*3+[2.5]*3)
    solutions=[least_squares(cost,[*r,0,0,0],bounds=bounds,loss='soft_l1',f_scale=.2,max_nfev=80) for r in seeds]
    return min(solutions,key=lambda sol:sol.cost)


def metrics(solution, windows):
    records=[]
    for (epoch,window),motions in windows:
        r,t=residual(solution.x,np.array(motions))
        records.append(dict(epoch=epoch,window=window,pairs=len(t),translation_median=float(np.median(np.linalg.norm(t,axis=1))),
                            rotation_median_deg=float(np.degrees(np.median(np.linalg.norm(r,axis=1))))))
    return records


def run(robot,camera,lidar):
    results=[]
    for offset in OFFSETS_MS:
        windows,sync=pairs(camera,lidar,offset/1000)
        train=[w for w in windows if w[0][1]%HOLDOUT_EVERY!=0]
        held=[w for w in windows if w[0][1]%HOLDOUT_EVERY==0]
        fit_result=fit(train)
        report=dict(offset_ms=offset,sync=sync,train_windows=len(train),holdout_windows=len(held))
        if fit_result is not None and held:
            train_scores,held_scores=metrics(fit_result,train),metrics(fit_result,held)
            independent = [fit(train[::2]),fit(train[1::2])]
            repeatability=None
            if all(result is not None for result in independent):
                a,b=[transform(result.x) for result in independent]
                relative_fit=np.linalg.inv(a)@b
                repeatability=dict(translation_m=float(np.linalg.norm(relative_fit[:3,3])),
                                   rotation_deg=float(np.degrees(Rotation.from_matrix(relative_fit[:3,:3]).magnitude())))
            report.update(T_lidar_from_color_optical_candidate=transform(fit_result.x).tolist(),
                          training=train_scores,holdout=held_scores,repeatability=repeatability,
                          holdout_translation_median=float(np.median([m['translation_median'] for m in held_scores])),
                          holdout_rotation_median_deg=float(np.median([m['rotation_median_deg'] for m in held_scores])))
        results.append(report)
    return results


def main():
    p=argparse.ArgumentParser()
    p.add_argument('--scratch',type=Path,required=True)
    args=p.parse_args()
    dataset=lookup(OUT/'dataset_manifest.json')['robots']
    with (BASE/'12d_clean_backend/clean_all75_gtsam_trajectory.csv').open() as stream:
        lidar=list(csv.DictReader(stream))
    records={}
    for robot in ROBOTS:
        with (OUT/robot/'camera_poses.csv').open() as stream:
            camera=[r for r in csv.DictReader(stream) if r['valid_pose']=='1']
        records[robot]=dict(source=source_audit(robot,args.scratch,dataset),
                            hand_eye=run(robot,camera,[r for r in lidar if r['robot_id']==robot]))
        print(robot,[(r['offset_ms'],r.get('holdout_translation_median')) for r in records[robot]['hand_eye']],flush=True)
    write_csv(REPORT/'camera_lidar_timestamp_windows.csv',[
        dict(robot=robot,offset_ms=trial['offset_ms'],**trial['sync'])
        for robot,record in records.items() for trial in record['hand_eye']])
    dump(REPORT/'camera_lidar_calibration.json',dict(status='NO_GO_NO_VALIDATED_MOUNTING_EXTRINSIC',robots=records,
         note='Hand-eye hypotheses use original times and held-out motion windows. No independently validated rigid mount identity or scene projection. Never used for map placement.'))


if __name__=='__main__':
    main()
