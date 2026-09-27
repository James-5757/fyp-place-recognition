#!/usr/bin/env python3
"""Static preview companions for the frozen replay scenes."""
import json
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
R=Path('/home/cas/fyp_place_recognition');d=json.loads((R/'demo/final_replay/data/replay.json').read_text());o=R/'outputs/final_demo/previews';o.mkdir(parents=True,exist_ok=True);C={'robot1':'#4cc9f0','robot3':'#f8961e'}
def plot(name,title,kind):
 f,a=plt.subplots(figsize=(10,6),facecolor='#08111f');a.set_facecolor('#07101c');a.set_title(title,color='white');a.tick_params(colors='white')
 if kind=='local':
  for r,x in d['local_trajectories'].items():q=x[:max(2,len(x)//3)];a.plot([z['x'] for z in q],[z['y'] for z in q],c=C[r],label=r+' local')
 elif kind in ('pre','post'):
  q=d['pre_pgo_common_frame'] if kind=='pre' else d['post_pgo_global'];
  for r in C:x=[z for z in q if z['robot_id']==r];a.plot([z['x'] for z in x],[z['y'] for z in x],c=C[r],label=r)
 elif kind=='gicp':
  e=d['events'][40];q=np.array(e['query_cloud_xy']);c=np.array(e['candidate_raw_xy']);g=np.array(e['candidate_gicp_aligned_xy']);a.scatter(q[:,0],q[:,1],s=3,c=C['robot1']);a.scatter(c[:,0],c[:,1],s=3,c=C['robot3']);a.scatter(g[:,0],g[:,1],s=3,c='#57cc99')
 else:
  for r,x in d['maps']['post_pgo'].items():q=np.array(x);a.scatter(q[:,0],q[:,1],s=.2,c=C[r])
 a.legend();f.tight_layout();f.savefig(o/name,dpi=150,facecolor=f.get_facecolor());plt.close(f)
for n,t,k in [('01_independent_local_mapping.png','Scene 1 — Independent Local Mapping','local'),('02_scan_context_candidate.png','Scene 2 — Scan Context Candidate Retrieval','local'),('03_gicp_before_after.png','Scene 3 — Frozen GICP SE(3)','gicp'),('04_initial_common_frame.png','Scene 4 — Initial Common Frame','pre'),('05_pgo_transition.png','Scene 5 — GTSAM PGO transition','post'),('06_merged_map_final.png','Scene 6 — Merged Map','map')]:plot(n,t,k)
