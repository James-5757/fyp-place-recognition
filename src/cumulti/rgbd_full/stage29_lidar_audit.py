"""Contrast frozen LiDAR height excursions with offline GNSS altitude observations."""
import csv
import json
from math import isfinite
from pathlib import Path


def rows(path):
    with path.open(newline='',encoding='utf-8-sig') as stream:
        return list(csv.DictReader(stream))


def audit(base):
    lidar=base/'12d_clean_backend'
    local=rows(lidar/'clean_local_trajectory.csv')
    pgo=rows(lidar/'clean_all75_gtsam_trajectory.csv')
    report={}
    for robot in ('robot1','robot3'):
        early=rows(base/'28_multisensor'/f'{robot}_lidar_continuity.csv')
        raw=[float(r['tz']) for r in local if r['robot_id']==robot]
        opt=[float(r['tz']) for r in pgo if r['robot_id']==robot]
        start=min(int(r['keyframe_id']) for r in local if r['robot_id']==robot)
        source=rows(base/'12c_dataset_stability'/f'{robot}_full_ekf_stability.csv')
        gnss=[float(r['gnss1_altitude']) for r in source[start:]
              if r['gnss1_match_status']=='MATCHED' and r['gnss_both_valid']=='True'
              and r['gnss1_altitude'] and isfinite(float(r['gnss1_altitude']))]
        report[robot]=dict(largest_prefix_steps=sorted(early,key=lambda r:float(r['translation_m']),reverse=True)[:3],
                           local_z_range_m=max(raw)-min(raw),optimized_z_range_m=max(opt)-min(opt),
                           matched_gnss_altitude_range_m=max(gnss)-min(gnss),matched_gnss_samples=len(gnss),
                           gnss_used_for_display=False,action='KEEP_STAGE28_AND_FROZEN_STAGE12D')
    (base/'29_spatial_audit'/'lidar_continuity_diagnosis.json').write_text(
        json.dumps(report,indent=2)+'\n',encoding='utf-8')


if __name__=='__main__':
    audit(Path(__file__).resolve().parents[3]/'outputs/cumulti_v1')
