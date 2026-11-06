"""Post-freeze visual inspection examples; never influences camera selection."""
import cv2,pandas as pd
from common import *

def run():
    access=[];camera_guard(access);man=json.loads((B/'camera_data_manifest.json').read_text());df=pd.read_csv(B/'visual_geometric_pilot.csv')
    cases=df.sort_values(['status','inliers'],ascending=[True,False]).head(1).to_dict('records')+df[df.status.str.startswith('REJECT')].sort_values('matches',ascending=False).head(2).to_dict('records')
    for i,row in enumerate(cases):
        r,c=row['query_robot'],row['candidate_robot']
        q=next(f for f in man['robots'][r]['pilot_frames'] if f['rgb_message_index']==int(row['query_RGB_message_index']))
        d=next(f for f in man['robots'][c]['pilot_frames'] if f['rgb_message_index']==int(row['candidate_RGB_message_index']))
        a=cv2.imread(str(ROOT/q['file']));b=cv2.imread(str(ROOT/d['file']));canvas=cv2.vconcat([cv2.hconcat([a,b]),np.full((55,a.shape[1]*2,3),20,np.uint8)])
        cv2.putText(canvas,f"{r} RGB#{q['rgb_message_index']} vs {c} RGB#{d['rgb_message_index']} | {row['status']} | matches {row['matches']} inliers {row['inliers']}",(12,canvas.shape[0]-30),cv2.FONT_HERSHEY_SIMPLEX,.48,(230,230,230),1)
        dest=B/'pilot_contact_sheets';dest.mkdir(exist_ok=True);cv2.imwrite(str(dest/f'case_{i}.jpg'),canvas)

if __name__=='__main__':run()
