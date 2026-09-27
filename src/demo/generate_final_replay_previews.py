#!/usr/bin/env python3
"""Static presentation previews corresponding to three frozen replay scenes."""
import json
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
ROOT=Path('/home/cas/fyp_place_recognition'); data=json.loads((ROOT/'demo/final_replay/data/replay.json').read_text()); out=ROOT/'outputs/final_demo/previews';out.mkdir(parents=True,exist_ok=True)
colors={'robot1':'#4cc9f0','robot3':'#f8961e'}
def save(n,title,progress,mapview=False):
 fig,ax=plt.subplots(figsize=(11,6),facecolor='#08111f');ax.set_facecolor('#07101c');ax.set_title(title,color='white');ax.tick_params(colors='#9ab0c9')
 if mapview:
  p=np.array([x[:3] for x in data['map_points']]); ids=[x[3] for x in data['map_points']]
  for rid,c in colors.items():
   q=p[np.array([x==rid for x in ids])];ax.scatter(q[:,0],q[:,1],s=.3,c=c,label=rid)
  ax.legend();ax.text(.02,.03,'Collaborative Map — Rank1 + GICP + GTSAM',transform=ax.transAxes,color='white',bbox={'facecolor':'#102238'})
 else:
  for a in data['agents']:
   q=[p for p in a['trajectory'] if p['progress']<=progress/.78]
   if q:
    x=[p['x'] for p in q];y=[p['y'] for p in q];ax.plot(x,y,c=colors[a['robot_id']],label=a['robot_id']);ax.scatter(x[-1],y[-1],c=colors[a['robot_id']],s=55)
  ax.legend();ax.text(.02,.03,f'Replay time {progress:.0%}',transform=ax.transAxes,color='white',bbox={'facecolor':'#102238'})
 fig.tight_layout();fig.savefig(out/n,dpi=160,facecolor=fig.get_facecolor());plt.close(fig)
save('01_trajectory_animation.png','Scene 1 — Separate local trajectories',.14)
save('02_gicp_verification.png','Scene 3 — GICP verification / accepted loop',.62)
save('03_merged_map_final.png','Scene 5 — Merged map result',1,True)
