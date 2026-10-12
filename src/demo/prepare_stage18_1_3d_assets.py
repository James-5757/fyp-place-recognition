#!/usr/bin/env python3
"""New display metadata only; accepted Stage18 XYZ/measurements never rewritten."""
import json,hashlib
from pathlib import Path
import numpy as np,pandas as pd
ROOT=Path(__file__).resolve().parents[2];DATA=ROOT/'demo/progressive_multi_robot/data';OUT=ROOT/'outputs/cumulti_v1/18_1_presentation_upgrade'
def sha(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    OUT.mkdir(parents=True,exist_ok=True);meta={};inputs=[]
    for p in DATA.glob('*.json'):
        if p.name in ['display_bounds.json','geometry_manifest.json']:continue
        d=json.loads(p.read_text());inputs.append(dict(path=str(p),sha256=sha(p),bytes=p.stat().st_size))
        if 'maps' not in d:continue
        bounds={}
        for robot in ['robot1','robot3']:
            xyz=np.asarray(d['maps'][robot]+[r[2:5] for r in d['trajectory'][robot]]);bounds[robot]=[float(x) for x in np.r_[xyz.min(axis=0),xyz.max(axis=0)]]
        lo=np.minimum(bounds['robot1'][:3],bounds['robot3'][:3]);hi=np.maximum(bounds['robot1'][3:],bounds['robot3'][3:]);meta[p.name]={'robots':bounds,'global':[float(x) for x in np.r_[lo,hi]]}
    (DATA/'display_bounds.json').write_text(json.dumps(meta,separators=(',',':'))+'\n')
    local=pd.read_csv(ROOT/'outputs/cumulti_v1/12d_clean_backend/clean_local_trajectory.csv',usecols=['robot_id','keyframe_id','timestamp','tx','ty','tz','qx','qy','qz','qw'])
    row=local[local.robot_id.eq('robot1')&local.keyframe_id.eq(283)].iloc[0]
    (DATA/'local_first_encounter_pose.json').write_text(json.dumps([283,str(row.timestamp),*[float(row[k]) for k in ['tx','ty','tz','qx','qy','qz','qw']]])+'\n')
    (DATA/'geometry_manifest.json').write_text(json.dumps(dict(files=inputs,state_xyz='unchanged accepted Stage18 values; GPU display conversion only',binary_map_conversion=False),separators=(',',':'))+'\n')
    provenance=dict(accepted_base='0447a62210d46a685285b8f6d649d49fd880960d',clean75_sha256=sha(ROOT/'outputs/cumulti_v1/12d_clean_backend/ordered_rank1_clean_loops.csv'),map_sources=inputs,pose_stream_method='one exact historical Stage13B reproduction; current Robot1 pose only; 8 checkpoint gates',three_version='0.170.0 (r170)',point_downsampling='existing Stage18 subsets; playback <=2500/source, final<=10000/source; no new rounding',map_coordinates_interpolated=False,camera_interpolation='presentation only',robot_marker_interpolation='presentation-only between consecutive recorded causal poses; no pre-acceptance use of post-loop pose',crossfade='optional visual opacity transition between real recorded states; never an optimizer state',GT_used=False,old_demo={str(p):sha(p) for p in (ROOT/'demo/final_replay').rglob('*') if p.is_file()},frozen_decisions={str(p):sha(p) for p in [ROOT/'outputs/cumulti_v1/15_informative_loop_selection/summary.json',ROOT/'outputs/cumulti_v1/17_relative_measurement_geometry/stage17_geometry_decision.json']})
    provenance['GPU_coordinate_representation']='Float32 vertex buffers for display; scientific JSON values unchanged'
    provenance['frontend_release']='18.1.10'
    provenance['frontend_sha256']={n:sha(ROOT/'demo/progressive_multi_robot'/n) for n in ['index.html','styles.css','replay_controller.js','map3d_view.js','system_view.js']}
    (OUT/'presentation_data_provenance.json').write_text(json.dumps(provenance,indent=2)+'\n')
    vendor=ROOT/'demo/progressive_multi_robot/vendor'
    urls={'three.module.js':'https://raw.githubusercontent.com/mrdoob/three.js/r170/build/three.module.js','OrbitControls.js':'https://raw.githubusercontent.com/mrdoob/three.js/r170/examples/jsm/controls/OrbitControls.js','LICENSE_THREE':'https://raw.githubusercontent.com/mrdoob/three.js/r170/LICENSE'}
    perf=json.loads((OUT/'performance_after.json').read_text())
    loading={'label':'BROWSER PRESENTATION PERFORMANCE','version':'Three.js0.170.0/r170','runtime_CDN':False,'dependencies':[{ 'file':name,'source':url,'bytes':(vendor/name).stat().st_size,'sha256':sha(vendor/name)} for name,url in urls.items()],'runtime_loads':perf['loading'],'time_until_Play_enabled_ms':perf['time_until_play_enabled_ms'],'total_JSON_bytes_observed':sum(x['body_bytes'] for x in perf['loading']),'state_fetch_waits':perf['counters']['state_fetch_waits'],'essential_files_count':9,'prefetch_next_states':3,'transport':'after local HTTP; baseline SSH tunnel, timings not a controlled network comparison'}
    (OUT/'asset_loading_summary.json').write_text(json.dumps(loading,indent=2)+'\n')
    print('BOUNDS/MANIFEST READY',len(meta))
if __name__=='__main__':main()
