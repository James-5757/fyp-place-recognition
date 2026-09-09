import argparse, csv, json, os, time
from pathlib import Path
from openai import OpenAI
from run_vlm_verification_pilot import TASK_PROMPT, image_to_data_url, normalize_base_url, normalize_decision, parse_json_response

def main():
 p=argparse.ArgumentParser();p.add_argument('--manifest',type=Path,default=Path('outputs/selective_vlm_v2/manifest.csv'));p.add_argument('--output-dir',type=Path,default=Path('outputs/selective_vlm_v2'));p.add_argument('--force',action='store_true');a=p.parse_args()
 key=os.environ.get('TEST_API_KEY');base=os.environ.get('TEST_API_BASE')
 if not key or not base: raise RuntimeError('TEST_API_KEY and TEST_API_BASE must be nonempty')
 client=OpenAI(api_key=key,base_url=normalize_base_url(base),timeout=120,max_retries=0);model=os.environ.get('VLM_MODEL') or 'gpt-5.6-sol'
 manifest=list(csv.DictReader(a.manifest.open()));raw=a.output_dir/'raw_responses.jsonl';decisions=a.output_dir/'vlm_decisions.csv';latest={}
 if raw.is_file():
  for line in raw.read_text(errors='ignore').splitlines():
   try:r=json.loads(line);latest[tuple(r['request_key'])]=r
   except:pass
 fields=['query_frame','case_type','anchor_frame','challenger_frame','order','candidate_A_frame','candidate_B_frame','choice','chosen_frame','confidence','matched_landmarks','contradictions_A','contradictions_B','viewpoint_difference','reason','parse_success','api_success','raw_response','error']
 def save():
  tmp=decisions.with_suffix('.tmp')
  with tmp.open('w',newline='',encoding='utf8') as f:
   w=csv.DictWriter(f,fieldnames=fields);w.writeheader()
   for r in latest.values():
    row={x:r.get(x,'') for x in fields}
    for x in ['matched_landmarks','contradictions_A','contradictions_B']:
     if isinstance(row[x],list):row[x]=json.dumps(row[x],ensure_ascii=False)
    w.writerow(row)
  os.replace(tmp,decisions)
 print('manifest_requests',len(manifest),'existing_success',sum(bool(r.get('api_success')) for r in latest.values()))
 for i,row in enumerate(manifest,1):
  rk=(int(row['query_frame']),str(row['case_type']),int(row['anchor_frame']),int(row['challenger_frame']),row['order'])
  if rk in latest and latest[rk].get('api_success') and not a.force:continue
  rec={'request_key':list(rk),'query_frame':int(row['query_frame']),'case_type':row['case_type'],'anchor_frame':int(row['anchor_frame']),'challenger_frame':int(row['challenger_frame']),'order':row['order'],'candidate_A_frame':int(row['candidate_A_frame']),'candidate_B_frame':int(row['candidate_B_frame'])}
  for attempt in range(3):
   try:
    resp=client.chat.completions.create(model=model,messages=[{'role':'user','content':[{'type':'text','text':TASK_PROMPT},{'type':'image_url','image_url':{'url':image_to_data_url(Path(row['collage_path']))}}]}],temperature=0)
    raw_text=resp.choices[0].message.content or '';parsed,ok=parse_json_response(raw_text);d=normalize_decision(parsed);chosen='' if d['choice'] not in {'A','B'} else rec['candidate_A_frame'] if d['choice']=='A' else rec['candidate_B_frame'];rec.update(d);rec.update({'chosen_frame':chosen,'parse_success':ok,'api_success':True,'raw_response':raw_text,'error':''});break
   except Exception as e:
    if attempt<2:time.sleep(2**attempt)
    else:rec.update({'choice':'UNCERTAIN','chosen_frame':'','confidence':0,'matched_landmarks':[],'contradictions_A':[],'contradictions_B':[],'viewpoint_difference':'unknown','reason':'API request failed','parse_success':False,'api_success':False,'raw_response':'','error':f'{type(e).__name__}: {e}'})
  with raw.open('a',encoding='utf8') as f:f.write(json.dumps(rec,ensure_ascii=False)+'\n');f.flush();os.fsync(f.fileno())
  latest[rk]=rec;save();print('processed',i,'/',len(manifest))
 save()
if __name__=='__main__':main()
