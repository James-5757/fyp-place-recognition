#!/usr/bin/env python3
"""Validate an explicit XYZ transport adapter without altering frozen GICP code."""
import numpy as np,pandas as pd
import run_stage9_integrated_latency as s9
import run_stage2_sc_baseline as s2
from prepare_stage21_policy import OUT,PROC,dump
import stage21_wire as w
def main():
    import json,open3d as o3d
    p=json.loads((OUT/'stage21_measurement_policy.json').read_text());checks=[]
    for robot in ['robot1','robot3']:
        f=pd.read_csv(PROC/robot/'keyframes.csv',usecols=['keyframe_id','lidar_file']).set_index('keyframe_id')
        for k in p['cloud_samples'][robot]:
            original=np.load(PROC/robot/'lidar'/f.loc[k,'lidar_file']);xyz=w.parse_cloud(w.cloud_body(original[:,:3]));adapted=np.column_stack([xyz,np.zeros(len(xyz),np.float32)])
            assert adapted.shape==original.shape and np.array_equal(adapted[:,:3],original[:,:3])
            def preprocess(a):
                xyz=a[:,:3];radius=np.hypot(xyz[:,0],xyz[:,1]);keep=(radius>=2)&(radius<=50)&(xyz[:,2]>=-3)&(xyz[:,2]<=5);c=o3d.geometry.PointCloud();c.points=o3d.utility.Vector3dVector(xyz[keep].astype(np.float64));return np.asarray(c.voxel_down_sample(.75).points)
            assert np.array_equal(preprocess(original),preprocess(adapted));assert np.array_equal(s2.descriptor(original),s2.descriptor(adapted));checks.append(dict(robot_id=robot,keyframe_id=k,exact_consumed_XYZ=True,preprocessed_points_bitwise_equal=True,SC_bitwise_equal=True))
    team=s9.team_module();a=p['frozen_gicp_attempts'][0];row=type('Row',(),dict(qid=a['query'],cid=a['candidate'],rank=1,score=0.,shift=a['shift']));out=OUT/'consumed_field_validation';out.mkdir()
    normal=s9.call(team,row,out/'original.csv');original_load=np.load
    def transported(*args,**kwargs):
        a=original_load(*args,**kwargs);xyz=w.parse_cloud(w.cloud_body(a[:,:3]));return np.column_stack([xyz,np.zeros(len(xyz),np.float32)])
    try:team.np.load=transported;adapted=s9.call(team,row,out/'XYZ_adapter.csv')
    finally:team.np.load=original_load
    delta=float(np.max(abs(np.array(json.loads(normal.gicp_transform))-np.array(json.loads(adapted.gicp_transform)))));qd=abs(float(normal.gicp_quality)-float(adapted.gicp_quality));assert delta<1e-8 and qd<1e-10
    dump('gicp_consumed_field_equivalence.json',dict(status='PASS',checks=checks,stored_shape_contract='frozen loader requires Nx4; receiver adapter restores a fourth zero intensity column before calling unchanged loader',transport='XYZ float32 lossless for all consumed coordinate fields; NOT lossless preservation of unused original intensity',actual_backend_pair=[a['query'],a['candidate']],actual_backend_max_matrix_difference=delta,actual_backend_quality_difference=qd,voxel_crop_parameters_unchanged=True,XYZI_alternative='separately measured, preserves all original four fields',no_algorithm_file_modified=True))
    print('CONSUMED_FIELD_EQUIVALENCE_PASS',len(checks),delta,flush=True)
if __name__=='__main__':main()
