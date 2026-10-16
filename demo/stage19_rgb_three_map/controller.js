import {ThreeMaps} from './three_maps.js';
import {RGBReplay} from './rgb_replay.js';
let lastAudit=-Infinity;
const $=id=>document.getElementById(id),BASE='../progressive_multi_robot/data/';
const metrics={geometry_builds:0,map_state_swaps:0,gpu_render_frames:0,webgl_resizes:0,loads:[],frame_ms:[],cpu_ms:[],seek_ms:[],state_waits:0};
const cache=new Map();let replay,poses,local,bounds,world,rgb,frames=[],cursor=0,elapsed=0,playing=false,last=0,context,state,serial=0,seen=-1,sensorNs=0n,referenceKF=234,ready=0,measure;
function text(id,v){if($(id).textContent!==String(v))$(id).textContent=String(v);}
async function fetchJSON(url){const t=performance.now(),r=await fetch(url);if(!r.ok)throw Error(`${url}: HTTP${r.status}`);const data=await r.json();metrics.loads.push({url,ms:performance.now()-t,bytes:Number(r.headers.get('Content-Length'))});return data;}
function asset(file){if(!cache.has(file))cache.set(file,fetchJSON(BASE+file).then(s=>{s.file=file;return s;}).catch(e=>{cache.delete(file);throw e;}));return cache.get(file);}
function fail(e){playing=false;text('play','Play');$('error').hidden=false;text('error',`Replay failed: ${e.message}. No scientific data substituted.`);}
function index(){let K=0,total=0;const events=new Map(replay.events.map(e=>[e.query,e])),snaps=new Map();for(const s of replay.snapshots)if(!snaps.has(s.K))snaps.set(s.K,s);
 for(let kf=150;kf<=1999;kf++){const event=events.get(kf);for(const phase of event?['QUERY ARRIVED','SCAN CONTEXT CANDIDATE','GICP VERIFYING','LOOP ACCEPTED','BACKEND UPDATE']:['LOCAL ODOMETRY']){
  const accepted=K+(event&&['LOOP ACCEPTED','BACKEND UPDATE'].includes(phase)?1:0),applied=K+(event&&phase==='BACKEND UPDATE'?1:0),duration=event?(event.index===1?({'QUERY ARRIVED':800,'SCAN CONTEXT CANDIDATE':1200,'GICP VERIFYING':1200,'LOOP ACCEPTED':1600,'BACKEND UPDATE':1800}[phase]):180):120;
  const file=kf===1999?'FINAL_STREAM_END.json':accepted===0?'PRE_FIRST_LOOP.json':applied===0?'COMMON_INITIAL.json':snaps.get(applied).file;
  frames.push({kf,event,phase,K:accepted,applied,file,pre:accepted===0,final:kf===1999,duration,start:total});total+=duration;
 }if(event)K++;}
 const names=['PRE_FIRST_LOOP','FIRST_LOOP','K2','K5','K10','K20','K40','K75','FINAL'];
 for(const name of names){const b=document.createElement('button');b.textContent=name;b.dataset.jump=name;b.onclick=()=>jump(name);$('jumps').append(b);}
 for(const [name,label]of [['CANDIDATE','SC candidate'],['GICP','GICP']]){const b=document.createElement('button');b.textContent=label;b.dataset.jump=name;b.onclick=()=>jump(name);$('jumps').append(b);}
}
function target(name){if(name==='PRE_FIRST_LOOP')return 0;if(name==='FINAL')return frames.length-1;if(name==='CANDIDATE'||name==='GICP')return frames.findIndex(f=>f.event?.index===1&&f.phase===(name==='GICP'?'GICP VERIFYING':'SCAN CONTEXT CANDIDATE'));return frames.findIndex(f=>f.phase==='BACKEND UPDATE'&&f.K===(name==='FIRST_LOOP'?1:Number(name.slice(1))));}
function title(f){if(f.final)return 'FINAL GLOBAL MAP · Top-down visualization of 3D Pose3 states';if(f.event?.index===1)return {'QUERY ARRIVED':'First query · no candidate exposed yet','SCAN CONTEXT CANDIDATE':'FIRST CROSS-ROBOT ENCOUNTER · First detected common place','GICP VERIFYING':'GICP VERIFICATION · R1 KF283 ↔ R3 KF252','LOOP ACCEPTED':'FIRST LOOP ACCEPTED → COMMON GLOBAL FRAME','BACKEND UPDATE':'FIRST LOOP · Persistent iSAM2 joint estimate'}[f.phase];return f.pre?'TWO LOCAL MAPS · Independent Local Frames':`Progressive Map Fusion · ${f.phase}`;}
async function update(){const id=++serial,f=frames[cursor],start=performance.now();if(!cache.has(f.file))metrics.state_waits++;const s=await asset(f.file);if(id!==serial)return;state=s;context=f;world.setState(s,f);world.setInset(f.phase==='GICP VERIFYING'?f.event:null,$('alignment').value==='after');
 text('scene',title(f));text('status',`Current accepted clean loops: ${f.K} / 75 · R1 KF${f.kf}`);text('globalLabel',f.pre?'UNALIGNED — no overlay / no future global map':`${s.name} · actual frozen state KF${s.kf} / K${s.K} (step-held)`);
 const visibleEvent=f.event&&f.phase!=='QUERY ARRIVED',verified=visibleEvent&&['GICP VERIFYING','LOOP ACCEPTED','BACKEND UPDATE'].includes(f.phase);text('eventTitle',visibleEvent?f.phase:'Independent local observation');text('eventInfo',visibleEvent?`R1 KF${f.kf} ↔ R3 historical KF${f.event.candidate} · SC ${f.event.score.toFixed(4)} · rank ${f.event.rank} · shift ${f.event.shift}`+(verified?` · frozen GICP ${f.event.gicp_ms.toFixed(2)} ms`:' · GICP not yet revealed'):'No future candidate or accepted-loop event displayed.');$('quality').value=verified?f.event.quality:0;text('qualityValue',verified?f.event.quality.toFixed(4):'—');
 referenceKF=visibleEvent?f.event.candidate:(replay.events.filter(e=>e.query<f.kf).at(-1)?.candidate??234);
 $('timeline').value=cursor;text('timeLabel',`KF${f.kf} · K${f.K}`);seen=cursor;metrics.seek_ms.push(performance.now()-start);
 // Only next two real states. Decoded geometry is not rebuilt until the selected state changes.
 for(const n of frames.slice(cursor+1,cursor+15).map(f=>f.file).filter((v,i,a)=>a.indexOf(v)===i).slice(0,2))asset(n).catch(()=>{});
 sync();}
function sync(){if(!context)return;const f=context,localRow=local.trajectory.robot1[f.kf-150],t=BigInt(localRow[1]);
 // Scientific state is step-held at this KF. Camera may advance strictly before next KF.
 // Event phases freeze exactly at the query timestamp; seeking never anticipates the next KF.
 const next=BigInt(local.trajectory.robot1[Math.min(1849,f.kf-149)][1]),fraction=playing&&!f.event&&seen===cursor?Math.min(1,elapsed/f.duration):0;
 sensorNs=t+BigInt(Math.min(Math.max(0,Number(next-t)-1),Math.round(Number(next-t)*fraction)));rgb.update(sensorNs,referenceKF);
 text('clock',`Recorded R1 bag ns: ${sensorNs} · demo ${(f.start+elapsed)/1000|0}s · event-paced, NOT sensor seconds`);
 text('rgb1label',f.event?'Event sensor time held':'Causal recorded camera timeline');
 // Current global marker uses last arrived pose only, never interpolation of graph coordinates.
 const safe=f.event&&f.phase!=='BACKEND UPDATE'&&!f.pre?f.kf-1:f.kf;
 const pose=f.pre?local.trajectory.robot1[f.kf-150]:poses.rows[safe-150];world.animate(pose);
}
function tick(t){const start=performance.now(),dt=last?t-last:0;last=t;if(dt)metrics.frame_ms.push(dt);
 if(playing&&seen===cursor){elapsed+=Math.min(dt,1000)*Number($('speed').value);while(elapsed>=frames[cursor].duration){elapsed-=frames[cursor].duration;if(cursor<frames.length-1)cursor++;else{playing=false;elapsed=0;text('play','Play');break;}}if(cursor!==seen)update().catch(fail);}
 sync();metrics.cpu_ms.push(performance.now()-start);if(new URLSearchParams(location.search).has('debug')&&t-lastAudit>=250){$('audit').textContent=JSON.stringify(audit());lastAudit=t;}requestAnimationFrame(tick);}
function audit(){return {ready:!!ready,playing,kf:context?.kf,K:context?.K,phase:context?.phase,state:state?.name,state_kf:state?.kf,state_K:state?.K,sensor_ns:String(sensorNs),referenceKF,rgb:rgb?.current,metrics,rgb_metrics:rgb?.metrics,point_count:world?.pointCount(),local_panels_visible:true,local_coordinates:'independent frozen local, never global optimized',scientific_interpolation:false};}
function jump(name){playing=false;elapsed=0;cursor=target(name);text('play','Play');update().catch(fail);}
function summary(a){const v=[...a].sort((a,b)=>a-b);return {count:v.length,mean:v.reduce((a,b)=>a+b,0)/Math.max(1,v.length),p95:v[Math.floor((v.length-1)*.95)]??null,max:v.at(-1)??null};}
function measureEnd(){playing=false;text('play','Play');const seconds=(performance.now()-measure.start)/1000;const report={label:'BROWSER PRESENTATION ONLY; not SLAM',seconds,start_kf:measure.kf,end_kf:context.kf,raf:summary(metrics.frame_ms.slice(measure.frames)),cpu_ms:summary(metrics.cpu_ms.slice(measure.cpu)),approx_fps:(metrics.frame_ms.length-measure.frames)/seconds,loads:metrics.loads,seek_ms:summary(metrics.seek_ms),rgb:rgb.metrics,geometry_builds:metrics.geometry_builds,visibility:document.visibilityState,startup_ms:ready,caveat:'Automated browser RAF may be throttled; no 60FPS or realtime claim.'};$('performance').textContent=JSON.stringify(report,null,2);}
window.stage19Audit=audit;
$('play').onclick=()=>{if(cursor===frames.length-1){cursor=0;elapsed=0;}playing=!playing;last=0;text('play',playing?'Pause':'Play');update().catch(fail);};$('reset').onclick=()=>jump('PRE_FIRST_LOOP');$('timeline').oninput=()=>{playing=false;elapsed=0;cursor=Number($('timeline').value);text('play','Play');update().catch(fail);};
$('alignment').onchange=()=>update().catch(fail);$('pointSize').oninput=()=>world.setPointSize(Number($('pointSize').value));$('loopStyle').onchange=()=>{world.loopMode=$('loopStyle').value;world.applyVisibility();};for(const id of ['mapR1','mapR3','mapTraj'])$(id).onchange=()=>world.applyVisibility();for(const b of document.querySelectorAll('[data-camera]'))b.onclick=()=>world.preset(b.dataset.camera);
$('measure').onclick=()=>{measure={start:performance.now(),frames:metrics.frame_ms.length,cpu:metrics.cpu_ms.length,kf:context.kf};playing=true;text('play','Pause');setTimeout(measureEnd,20000);};
window.addEventListener('unhandledrejection',e=>fail(e.reason));window.addEventListener('error',e=>fail(e.error||Error(e.message)));
(async()=>{const all=await Promise.all([fetchJSON(BASE+'replay.json'),fetchJSON(BASE+'robot1_causal_pose_stream.json'),fetchJSON('local_maps.json'),fetchJSON(BASE+'display_bounds.json'),fetchJSON('rgb_manifest.json')]);[replay,poses,local,bounds]=all;rgb=new RGBReplay(all[4]);index();world=new ThreeMaps($('worldCanvas'),bounds,metrics,local);world.setEvents(replay.events);$('timeline').max=frames.length-1;await update();ready=performance.now();$('play').disabled=false;$('reset').disabled=false;$('timeline').disabled=false;requestAnimationFrame(tick);})().catch(fail);
