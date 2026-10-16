// Images are decoded on demand, maximum two in flight / 24 cached Image objects.
// Tokens invalidate stale seek requests. No all-frame startup decode.
export class RGBReplay {
 constructor(manifest){this.manifest=manifest;this.cache=new Map();this.pending=new Map();this.failures=new Map();this.queue=[];this.active=0;this.generations={robot1:0,robot3:0};this.wanted={};this.metrics={decodes:0,cache_hits:0,errors:0,decode_ms:[],swaps:0,max_cache:0,max_active:0,missing:0};this.current={};}
 floor(robot,ns){const rows=this.manifest.frames[robot];let lo=0,hi=rows.length;while(lo<hi){const mid=(lo+hi)>>1;if(BigInt(rows[mid].rgb_timestamp_ns)<=ns)lo=mid+1;else hi=mid;}while(lo&&BigInt(rows[lo-1].header_timestamp_ns??rows[lo-1].rgb_timestamp_ns)>ns)lo--;return lo?rows[lo-1]:null;}
 reference(kf){const j=this.manifest.robots.robot3.keyframe_rgb_indices[kf];return this.manifest.frames.robot3.find(r=>r.rgb_message_index===j)||null;}
 load(row){
  if(this.cache.has(row.file)){this.metrics.cache_hits++;const v=this.cache.get(row.file);this.cache.delete(row.file);this.cache.set(row.file,v);return Promise.resolve(v);}
  if(this.pending.has(row.file))return this.pending.get(row.file);
  if(performance.now()-(this.failures.get(row.file)??-Infinity)<5000)return Promise.reject(Error('Camera frame unavailable (retry backoff)'));
  if(this.queue.length>=6){const old=this.queue.shift();this.pending.delete(old.row.file);old.reject(Error('Superseded queued image request'));}
  const p=new Promise((resolve,reject)=>this.queue.push({row,resolve,reject}));this.pending.set(row.file,p);this.pump();return p;
 }
 pump(){while(this.active<2&&this.queue.length){const job=this.queue.shift(),start=performance.now();this.active++;this.metrics.max_active=Math.max(this.metrics.max_active,this.active);const image=new Image();image.decoding='async';image.src=job.row.file;
  image.decode().then(()=>{this.metrics.decodes++;this.metrics.decode_ms.push(performance.now()-start);this.cache.set(job.row.file,image);while(this.cache.size>24){const k=this.cache.keys().next().value;this.cache.delete(k);}this.metrics.max_cache=Math.max(this.metrics.max_cache,this.cache.size);job.resolve(image);}).catch(e=>{this.metrics.errors++;this.failures.set(job.row.file,performance.now());if(this.failures.size>24)this.failures.delete(this.failures.keys().next().value);job.reject(e);}).finally(()=>{this.pending.delete(job.row.file);this.active--;this.pump();});}}
 async show(robot,row,sensorNs,referenceKF){
  const suffix=robot==='robot1'?'1':'3',img=document.getElementById('rgb'+suffix),missing=document.getElementById('missing'+suffix),label=document.getElementById('rgb'+suffix+'time');
  const identity=row?.file||'missing';if(this.wanted[robot]===identity)return;this.wanted[robot]=identity;const gen=++this.generations[robot];
  img.hidden=true;missing.hidden=false;missing.textContent=row?'Loading authentic frame…':'MISSING CAMERA FRAME — no substitution';
  label.textContent=row?`Selected RGB bag ns ${row.rgb_timestamp_ns} · nearest grid KF${row.keyframe_id}`:'No recorded RGB available at this time';
  this.current[robot]={status:row?'LOADING':'MISSING',requested:identity};
  if(!row){this.metrics.missing++;return;}
  try{const decoded=await this.load(row);if(gen!==this.generations[robot])return;img.src=decoded.src;img.hidden=false;missing.hidden=true;this.metrics.swaps++;
   label.textContent=robot==='robot1'?`RGB ${row.rgb_timestamp_ns} · age ${Number(sensorNs-BigInt(row.rgb_timestamp_ns))/1e6|0} ms · LiDAR association |Δ| ${(Number(row.abs_sync_error_ns)/1e6).toFixed(2)} ms`:`Historical KF${referenceKF} · RGB ${row.rgb_timestamp_ns} · |Δ LiDAR| ${(Number(row.abs_sync_error_ns)/1e6).toFixed(2)} ms`;
   this.current[robot]={status:'READY',...row,referenceKF};
  }catch(e){if(gen!==this.generations[robot])return;missing.textContent='MISSING / FAILED CAMERA FRAME — no substitution';label.textContent=e.message;this.current[robot]={status:'ERROR',file:row.file};}
 }
 update(ns,referenceKF){const row=this.floor('robot1',ns);this.show('robot1',row,ns,null);this.show('robot3',this.reference(referenceKF),ns,referenceKF);
  // Bounded lookahead by camera index, with no display of prefetched future frames.
  if(row){const rows=this.manifest.frames.robot1,j=rows.indexOf(row);if(this.queue.length<3)for(const next of rows.slice(j+1,j+3))this.load(next).catch(()=>{});}
 }
}
