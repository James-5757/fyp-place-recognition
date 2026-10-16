import {DualMaps} from './dual_maps.js';
import {RGBReplay} from '../stage19_rgb_three_map/rgb_replay.js';
const $=id=>document.getElementById(id),json=async p=>{const r=await fetch(p);if(!r.ok)throw Error(`${p}: HTTP ${r.status}`);return r.json();};
const perf={webgl_resizes:0,geometry_builds:0,map_state_swaps:0,gpu_render_frames:0,frame_ms:[],state_load_ms:[],startup_ms:0};
let replay,view,rgb,event=0,state,playing=false,busy=false,generation=0,lastFrame=0,replayNs=0n,lastLabel='',lastAudit=0;
const cache=new Map();
function error(e){$('error').hidden=false;$('error').textContent=String(e.stack||e);playing=false;$('play').textContent='Play';}
class DualRGB extends RGBReplay {
 async show(robot,row,ns){await super.show(robot,row,ns,null);const id=robot==='robot1'?'1':'3';if(this.current[robot]?.status==='READY'&&this.current[robot]?.file===row?.file)$('rgb'+id+'time').textContent=`Own-clock RGB ${row.rgb_timestamp_ns} · age ${(Number(ns-BigInt(row.rgb_timestamp_ns))/1e6).toFixed(1)} ms · |Δ LiDAR| ${(Number(row.abs_sync_error_ns)/1e6).toFixed(2)} ms`;}
 updateDual(e){for(const [robot,last]of [['robot1',e[4]],['robot3',e[5]]]){const first=replay.limits[robot][0];if(last<first){this.show(robot,null,BigInt(replay.origins[robot]));continue;}const end=this.manifest.endpoints[robot],ns=BigInt(replay.origins[robot])+BigInt(e[1]);const target=ns>BigInt(end)?BigInt(end):ns,row=this.floor(robot,target);this.show(robot,row,target);if(row&&this.queue.length<3){const rows=this.manifest.frames[robot],j=rows.indexOf(row);for(const v of rows.slice(j+1,j+3))this.load(v).catch(()=>{});}}}
}
async function loadState(s){if(cache.has(s.file))return cache.get(s.file);const start=performance.now(),data=await json('states/'+s.file);cache.set(s.file,data);while(cache.size>3)cache.delete(cache.keys().next().value);perf.state_load_ms.push(performance.now()-start);return data;}
function selectedState(i){let result=replay.states[0];for(const s of replay.states){if(s.event_index>i)break;result=s;}return result;}
async function seek(i,label=''){
 const token=++generation;busy=true;i=Math.max(0,Math.min(5795,i));
 try{const e=replay.schedule[i],s=selectedState(i);let data;
  if(!e[8])data={name:'UNALIGNED',file:'UNALIGNED',K:0,kf:e[4],trajectory:{robot1:[],robot3:[]},maps:{robot1:[],robot3:[]}};
  else data=await loadState(s);
  if(token!==generation)return;event=i;state=data;lastLabel=label;replayNs=BigInt(e[1]);view.update(data,e);rgb.updateDual(e);labels(e,s,label);
 }catch(e){error(e);}finally{if(token===generation){busy=false;audit();}}
}
function labels(e,s,label){const k=e[6],end=e[0]===5795;
 $('scene').textContent=`${label?label+' · ':''}${!e[8]?'TWO INDEPENDENT LOCAL MAPS':k===1?'FIRST BRIDGE / COMMON GLOBAL FRAME':end?'FINAL DUAL-STREAM GLOBAL MAP':'PROGRESSIVE MAP FUSION'} · actual K=${k}/75`;
 $('status').textContent=`R1 KF${e[4]} · R3 KF${e[5]} · components ${e[7]} · global ${e[8]?'initialized':'not initialized'}`;
 $('r1status').textContent=`Independent Local Frame · KF${e[4]} · ${Math.max(0,e[4]-149)} nodes${e[4]===1999?' · RECORDING COMPLETE':''}`;
 $('r3status').textContent=`Independent Local Frame · KF${e[5]} · ${Math.max(0,e[5]-233)} nodes${e[5]===4179?' · RECORDING COMPLETE':''}`;
 $('globalLabel').textContent=!e[8]?'UNALIGNED / WAITING FOR FIRST INTER-ROBOT LOOP':`Real state @ ${(Number(s.normalized_ns)/1e9).toFixed(3)}s · R1 KF${s.kf}, R3 KF${s.r3kf} · step-held until next saved state`;
 const latest=replay.events.filter(l=>l.event_index<=event).at(-1);
 $('eventTitle').textContent=latest?`Archived loop ID${latest.frozen_loop_id} · eligible #${latest.index}`:'Two independent local chains';
 $('eventInfo').textContent=latest?`R1 KF${latest.query} ↔ R3 KF${latest.candidate}. Both endpoints arrived. SC candidate → frozen GICP acceptance → archived loop → persistent central iSAM2. No frontend recomputation.`:'Each robot extends only its own arrived prefix. No disconnected central graph is created.';
 $('clock').textContent=`Normalized replay ${(Number(e[1])/1e9).toFixed(3)} s · original R1 ns ${(BigInt(replay.origins.robot1)+minNs(e,'robot1')).toString()} · original R3 ns ${(BigInt(replay.origins.robot3)+minNs(e,'robot3')).toString()}`;
 $('timeline').value=event;$('timeLabel').textContent=`${(Number(e[1])/1e9).toFixed(1)} / ${(Number(replay.schedule.at(-1)[1])/1e9).toFixed(1)} s`;
}
function minNs(e,r){return BigInt(e[1])>BigInt(replay.endNormalized[r])?BigInt(replay.endNormalized[r]):BigInt(e[1]);}
function pause(){playing=false;$('play').textContent='Play';}
function audit(){if(!replay||!view)return;const e=replay.schedule[event];const a={status:'READY',event_index:event,checkpoint:lastLabel,normalized_ns:e[1],robot1_kf:e[4],robot3_kf:e[5],K:e[6],components:e[7],global_initialized:e[8],local1_draw_count:view.parts[0].geometry.drawRange.count,local3_draw_count:view.parts[1].geometry.drawRange.count,local1_visible:view.local1.visible,local3_visible:view.local3.visible,global_visible:view.scene.visible,global_state_event:state?.event_index??null,global_state_file:state?.file,playing,busy,local1_cached_chunks:view.geometryCache.size,local3_cached_chunks:view.r3cache.size,rgb:rgb.current,image_cache:rgb.cache.size,image_active:rgb.active,point_count:view.pointCount(),metrics:{...perf,frame_ms:perf.frame_ms.slice(-120),state_load_ms:perf.state_load_ms.slice(-100)},image_metrics:rgb.metrics};$('audit').textContent=JSON.stringify(a);$('performance').textContent=`Startup ${perf.startup_ms.toFixed(1)} ms · render frames ${perf.gpu_render_frames} · map swaps ${perf.map_state_swaps} · image cache ${rgb.cache.size}/24 · decodes in flight ${rgb.active}/2. Scientific timing is recorded separately on the Linux backend.`;}
function frame(now){requestAnimationFrame(frame);try{const dt=lastFrame?Math.min(now-lastFrame,250):0;lastFrame=now;if(!view)return;if(dt>0){perf.frame_ms.push(dt);if(perf.frame_ms.length>1200)perf.frame_ms.shift();}if(playing&&!busy){replayNs+=BigInt(Math.round(dt*10*Number($('speed').value)*1e6));let j=event;while(j+1<replay.schedule.length&&BigInt(replay.schedule[j+1][1])<=replayNs)j++;if(j!==event){const target=replayNs;seek(j).then(()=>{if(playing)replayNs=target;});}if(j===5795)pause();}view.animate();if(now-lastAudit>300){lastAudit=now;audit();}}catch(e){error(e);}}
async function init(){const start=performance.now();[replay]=await Promise.all([json('replay.json')]);const [local,bounds,manifest]=await Promise.all([json('local_maps.json'),json('bounds.json'),json('rgb_manifest.json')]);
 replay.states.sort((a,b)=>a.event_index-b.event_index);replay.endNormalized={};manifest.endpoints={};for(const r of ['robot1','robot3']){const e=replay.schedule.filter(e=>e[2]===r).at(-1);replay.endNormalized[r]=e[1];manifest.endpoints[r]=(BigInt(replay.origins[r])+BigInt(e[1])).toString();}
 bounds['FINAL_STREAM_END.json']=bounds['FINAL.json'];view=new DualMaps($('worldCanvas'),bounds,perf,local);view.setEvents(replay.events);rgb=new DualRGB(manifest);
 $('play').onclick=()=>{if(playing)pause();else{if(event===5795)seek(0);playing=true;$('play').textContent='Pause';lastFrame=performance.now();}};
 $('reset').onclick=()=>{pause();seek(0,'RESET');};$('timeline').max=5795;$('timeline').oninput=()=>{pause();seek(Number($('timeline').value),'SEEK');};
 for(const name of ['PRE_FIRST_BRIDGE','FIRST_BRIDGE','FIRST_POST_BRIDGE_UPDATE','DUAL_STREAM_K2','DUAL_STREAM_K5','DUAL_STREAM_K10','DUAL_STREAM_K20','DUAL_STREAM_K40','DUAL_STREAM_K75','ROBOT1_FINISHED','ROBOT3_FINISHED','FINAL']){const b=document.createElement('button');b.textContent=name;b.onclick=()=>{pause();seek(replay.jumps[name],name);};$('jumps').append(b);}
 document.querySelectorAll('[data-camera]').forEach(b=>b.onclick=()=>view.preset(b.dataset.camera));for(const id of ['local1On','local3On','globalOn','mapR1','mapR3','mapTraj'])$(id).onchange=()=>view.applyVisibility();$('loopStyle').onchange=()=>{view.loopMode=$('loopStyle').value;view.applyVisibility();};
 await seek(1,'START');for(const id of ['play','reset','timeline'])$(id).disabled=false;perf.startup_ms=performance.now()-start;audit();
}
window.addEventListener('error',e=>error(e.error||e.message));window.addEventListener('unhandledrejection',e=>error(e.reason));requestAnimationFrame(frame);init().catch(error);
