import {createCausalMap} from './causal-map.js';
import * as THREE from 'three';
import {OrbitControls} from '../progressive_multi_robot/vendor/OrbitControls.js';
const $=s=>document.querySelector(s),load=async u=>{const r=await fetch(u);if(!r.ok)throw Error(`${r.status}: ${u}`);return r.json();};
const upper=(a,x,key)=>{let lo=0,hi=a.length;while(lo<hi){const m=(lo+hi)>>>1;if(key(a[m])<=x)lo=m+1;else hi=m;}return lo;};
const atlasReplay = new URLSearchParams(location.search).get('atlas') === '1';
const manifest=await load(atlasReplay?'atlas/manifest.json':'manifest.json');
const data=await Promise.all(Object.entries(manifest.robots).map(async([id,path])=>[id,await load(path)]));
let time=0,playing=false,last=performance.now(),generation=0,renderFrames=0;
const stats={rgbRequests:0,geometryLoads:0,errors:[],maxGeometryCache:0,maxImageCache:0};
const panels=[];
const buffers=async(path,Type)=>{const r=await fetch(path);if(!r.ok)throw Error(`${r.status}: ${path}`);return new Type(await r.arrayBuffer());};
const causalMap=await createCausalMap($("#causal-map"),data,upper,buffers,atlasReplay?'atlas/bridge_summary.json':'causal_summary.json');
class Panel{
 constructor(id,d){
  this.id=id;this.d=d;this.cache=new Map();this.geometryPending=new Map();this.images=new Map();this.pending=new Map();this.active=null;this.currentEpoch=-1;this.inspection=null;this.rgbToken=0;this.lastRGB=-1;this.loadToken=0;
  for(const e of d.epochs)e.startIndex=d.frames.findIndex(f=>f.epoch===e.epoch);
  const a=document.createElement('article');a.id=id;a.innerHTML=`<div class="heading"><h2>${id==='robot1'?'Robot1':'Robot3'} Local RGB-D Map</h2><span class="state">Independent camera gauge</span></div><div class="map"></div><div class="tools"><button class="fit">Fit visible map</button><label>Epoch <select class="epoch"><option value="auto">Current (automatic)</option></select></label><label><input class="points" type="checkbox" checked> RGB map</label><label><input class="trajectory" type="checkbox" checked> Camera path</label><span>Left drag orbit · right drag pan · wheel zoom</span></div><div class="losses"><div class="bar"><i class="cursor"></i></div></div><div class="camera"><div><small>Recorded forward RGB · not live</small><img alt="Authentic recorded camera RGB" class="rgb"></div><div class="info"><div class="trackInfo"></div><div class="rgbInfo"></div></div></div>`;$('#panels').append(a);this.el=a;
  const box=a.querySelector('.map');this.scene=new THREE.Scene();this.scene.background=new THREE.Color('#060f18');this.camera=new THREE.PerspectiveCamera(48,1,.01,2000);this.camera.up.set(0,-1,0);this.camera.position.set(3,-3,-5);
  this.renderer=new THREE.WebGLRenderer({antialias:true,powerPreference:'low-power'});this.renderer.setPixelRatio(Math.min(devicePixelRatio,1.5));box.append(this.renderer.domElement);
  this.controls=new OrbitControls(this.camera,this.renderer.domElement);this.controls.enableDamping=true;this.controls.dampingFactor=.12;
  this.marker=new THREE.Mesh(new THREE.SphereGeometry(.08,10,8),new THREE.MeshBasicMaterial({color:id==='robot1'?0x38bfff:0xffa82c}));this.marker.visible=false;this.scene.add(this.marker);
  new ResizeObserver(()=>{const w=box.clientWidth,h=box.clientHeight;this.renderer.setSize(w,h,false);this.camera.aspect=w/h;this.camera.updateProjectionMatrix();}).observe(box);
  a.querySelector('.fit').onclick=()=>this.fit();a.querySelector('.epoch').onchange=e=>{this.inspection=e.target.value==='auto'?null:+e.target.value;generation++;this.update(time);};
  for(const c of a.querySelectorAll('input'))c.onchange=()=>this.update(time);
  const bar=a.querySelector('.bar');let begin=null;
  for(let i=0;i<=d.frames.length;i++){const bad=i<d.frames.length&&!d.frames[i].valid;if(bad&&begin===null)begin=i;if(!bad&&begin!==null){const lo=d.frames[begin].t,hi=i<d.frames.length?d.frames[i].t:d.duration_s;const s=document.createElement('i');s.className='loss';s.style.left=`${100*lo/d.duration_s}%`;s.style.width=`${Math.max(.1,100*(hi-lo)/d.duration_s)}%`;s.title=`Tracking unavailable ${lo.toFixed(2)}–${hi.toFixed(2)} s`;bar.append(s);begin=null;}}
 }
 async geometry(epoch){
  if(this.cache.has(epoch)){const g=this.cache.get(epoch);this.cache.delete(epoch);this.cache.set(epoch,g);return g;}
  if(this.geometryPending.has(epoch))return this.geometryPending.get(epoch);
  const task=this.loadGeometry(epoch);this.geometryPending.set(epoch,task);try{return await task;}finally{this.geometryPending.delete(epoch);}
 }
 async loadGeometry(epoch){
  const e=this.d.epochs.find(e=>e.epoch===epoch);if(!e)return null;
  const [xyz,rgb,first]=await Promise.all([buffers(e.files['xyz.f32'],Float32Array),buffers(e.files['rgb.u8'],Uint8Array),buffers(e.files['first.u32'],Uint32Array)]);
  if(xyz.length!==first.length*3||rgb.length!==xyz.length)throw Error('Invalid point buffer schema');
  const linear=new Float32Array(rgb.length),color=new THREE.Color();for(let i=0;i<rgb.length;i+=3){color.setRGB(rgb[i]/255,rgb[i+1]/255,rgb[i+2]/255,THREE.SRGBColorSpace);linear.set([color.r,color.g,color.b],i);}
  const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(xyz,3));geometry.setAttribute('color',new THREE.BufferAttribute(linear,3));geometry.setDrawRange(0,0);
  const cloud=new THREE.Points(geometry,new THREE.PointsMaterial({size:.035,vertexColors:true,sizeAttenuation:true}));cloud.frustumCulled=false;
  const lines=[],ends=[];for(let i=1;i<this.d.frames.length;i++){const a=this.d.frames[i-1],b=this.d.frames[i];if(a.valid&&b.valid&&!b.visual_discontinuity&&a.epoch===epoch&&b.epoch===epoch){lines.push(...a.pose.slice(0,3),...b.pose.slice(0,3));ends.push(i);}}
  const tg=new THREE.BufferGeometry();tg.setAttribute('position',new THREE.Float32BufferAttribute(lines,3));tg.setDrawRange(0,0);const path=new THREE.LineSegments(tg,new THREE.LineBasicMaterial({color:this.id==='robot1'?0x38bfff:0xffa82c}));path.frustumCulled=false;
  const asset={cloud,path,first,ends,xyz,epoch};this.cache.set(epoch,asset);stats.geometryLoads++;
  while(this.cache.size>2){const [key,g]=this.cache.entries().next().value;if(g===this.active){this.cache.delete(key);this.cache.set(key,g);continue;}this.dispose(g);this.cache.delete(key);}
  stats.maxGeometryCache=Math.max(stats.maxGeometryCache,this.cache.size);return asset;
 }
 dispose(g){this.scene.remove(g.cloud,g.path);g.cloud.geometry.dispose();g.cloud.material.dispose();g.path.geometry.dispose();g.path.material.dispose();}
 rgbLabel(t){const p=this.d.previews[this.lastRGB];this.el.querySelector('.rgbInfo').textContent=p?`RGB shown: original frame ${p.index} @ ${p.t.toFixed(3)} s\nRGB header ns: ${this.d.frames[p.index].rgb_header_ns}\nPreview age: ${((Math.min(t,this.d.duration_s)-p.t)*1000).toFixed(1)} ms`:'RGB observation loading / unavailable';}
 fit(){if(!this.active)return;const n=this.active.cloud.geometry.drawRange.count;if(!n)return;const box=new THREE.Box3();for(let i=0;i<n;i++)box.expandByPoint(new THREE.Vector3().fromArray(this.active.xyz,i*3));const center=box.getCenter(new THREE.Vector3()),size=Math.max(box.getSize(new THREE.Vector3()).length(),2);this.controls.target.copy(center);this.camera.position.copy(center).add(new THREE.Vector3(size*.45,-size*.5,-size*.7));this.controls.update();}
 async image(index){
  if(index<0||index>=this.d.previews.length)return null;const p=this.d.previews[index],url=p.file;
  if(this.images.has(url)){const image=this.images.get(url);this.images.delete(url);this.images.set(url,image);return image;}
  if(this.pending.has(url))return this.pending.get(url);
  if(this.pending.size>=2)return null;
  stats.rgbRequests++;const task=(async()=>{const image=new Image();image.src=url;await image.decode();this.images.set(url,image);while(this.images.size>12)this.images.delete(this.images.keys().next().value);stats.maxImageCache=Math.max(stats.maxImageCache,this.images.size);return image;})();this.pending.set(url,task);
  try{return await task;}finally{this.pending.delete(url);}
 }
 async update(t){
  const index=Math.max(0,upper(this.d.frames,Math.min(t,this.d.duration_s),f=>f.t)-1),f=this.d.frames[index];this.frame=f;
  const available=this.d.epochs.filter(e=>e.startIndex<=index);
  const select=this.el.querySelector('.epoch');if(this.optionCount!==available.length){for(const option of [...select.options])if(option.value!=='auto')option.remove();for(const e of available)select.add(new Option(`${atlasReplay?'Connected component':'Epoch'} ${e.epoch}`,String(e.epoch)));this.optionCount=available.length;}
  if(this.inspection!==null&&!available.some(e=>e.epoch===this.inspection))this.inspection=null;select.value=this.inspection===null?'auto':String(this.inspection);
  let epoch=f.epoch;if(epoch<0){for(let i=index;i>=0;i--)if(this.d.frames[i].valid){epoch=this.d.frames[i].epoch;break;}}
  if(this.inspection!==null)epoch=this.inspection;
  // A backward seek must hide stale future observations synchronously, before I/O.
  const preview=Math.max(0,upper(this.d.previews,Math.min(t,this.d.duration_s),p=>p.t)-1),rgbToken=++this.rgbToken;
  if(this.lastRGB>preview){this.el.querySelector('.rgb').removeAttribute('src');this.lastRGB=-1;}
  this.marker.visible=false;
  if(this.active){const same=this.active.epoch===epoch,n=same?upper(this.active.first,index,x=>x):0;this.active.cloud.geometry.setDrawRange(0,n);this.active.path.geometry.setDrawRange(0,same?2*upper(this.active.ends,index,x=>x):0);this.visiblePoints=n;}
  this.el.querySelector('.state').textContent=(t>=this.d.duration_s?'RECORDING COMPLETE · ':f.valid?'TRACKING · ':'TRACKING UNAVAILABLE · ')+`Map ID ${f.map_id} ${atlasReplay?'· Connected component':'· Epoch'} ${epoch}`;
  this.el.querySelector('.cursor').style.left=`${100*Math.min(t,this.d.duration_s)/this.d.duration_s}%`;
  this.el.querySelector('.trackInfo').textContent=`Recorded ${Math.min(t,this.d.duration_s).toFixed(2)} / ${this.d.duration_s.toFixed(2)} s\nTracking input frame ${index} / ${this.d.frames.length-1}\nTracking input header ns: ${f.rgb_header_ns}\nDepth paired: ${f.paired?'yes':'NO'} · |Δ bag time| ${f.sync_ms.toFixed(3)} ms\n${atlasReplay?'Points accumulate within this connected component':this.inspection===null?'Current epoch only':'Inspecting a past epoch in its own gauge'}`;this.rgbLabel(t);
  this.el.querySelector('.info').style.whiteSpace='pre-line';
  const token=++this.loadToken;
  try{const g=epoch<0?null:await this.geometry(epoch);if(token!==this.loadToken)return;
   if(this.active!==g){if(this.active)this.scene.remove(this.active.cloud,this.active.path);this.active=g;if(g)this.scene.add(g.cloud,g.path);this.needsFit=true;}
   if(g){const count=upper(g.first,index,x=>x);g.cloud.geometry.setDrawRange(0,count);g.path.geometry.setDrawRange(0,2*upper(g.ends,index,x=>x));g.cloud.visible=this.el.querySelector('.points').checked;g.path.visible=this.el.querySelector('.trajectory').checked;this.visiblePoints=count;if(this.needsFit&&count){this.fit();this.needsFit=false;}}
   this.marker.visible=f.valid&&f.epoch===epoch;if(this.marker.visible)this.marker.position.fromArray(f.pose);
  }catch(e){stats.errors.push(e.message);$('#status').textContent='ASSET ERROR: '+e.message;}
  if(preview!==this.lastRGB){try{const image=await this.image(preview);if(rgbToken===this.rgbToken&&image){this.el.querySelector('.rgb').src=image.src;this.lastRGB=preview;this.rgbLabel(t);}}catch(e){stats.errors.push(e.message);if(rgbToken===this.rgbToken){this.el.querySelector('.rgb').removeAttribute('src');this.el.querySelector('.rgbInfo').textContent='RGB FRAME UNAVAILABLE';}}}
  // Only historical previews are prefetched; no future camera observations are revealed.
  this.image(preview-1).catch(()=>{});
 }
 render(){this.controls.update();this.renderer.render(this.scene,this.camera);}
}
if(atlasReplay){
 document.querySelector('h1').textContent=manifest.registered_epochs?'RGB-D mapping across verified epochs':'RGB-D mapping from optimized Atlas poses';
 document.querySelector('footer p').textContent='Each connected component accumulates the RGB-D points observed by the selected time. Unconnected components use separate coordinates. '+(manifest.registered_epochs?'This offline replay aligns epochs using verified RGB-D observations.':'This offline replay uses final optimized Atlas poses.');
}
for(const[id,d]of data)panels.push(new Panel(id,d));
$('#seek').max=manifest.duration_s;for(const id of ['play','reset','seek','stage20','final'])$('#'+id).disabled=false;
$('#status').textContent=atlasReplay?'Offline replay of connected RGB-D components':causalMap?'Full recordings · dual-direction causal RGB-D replay':`Full available recordings · independent local Map Epochs · bridge diagnostic: ${manifest.global_fusion} · no complete global map claimed`;
$('#qualification').textContent=data.map(([id,d])=>`${id}: ${d.tracking.valid_frames}/${d.tracking.frames} valid native camera poses; ${d.tracking.map_epochs} ${atlasReplay?'connected RGB-D components':'independent Map Epochs'}.`).join(' ')+' Rendering success does not establish continuous tracking or inter-robot fusion.';
$('#provenance').textContent=JSON.stringify(manifest,null,2);
function setTime(t){time=Math.max(0,Math.min(+t,manifest.duration_s));generation++;$('#seek').value=time;$('#time').textContent=`${time.toFixed(1)} / ${manifest.duration_s.toFixed(1)} s`;for(const p of panels)p.update(time);causalMap?.update(time);}
$('#play').onclick=()=>{playing=!playing;$('#play').textContent=playing?'Pause':'Play';last=performance.now();};
$('#reset').onclick=()=>{playing=false;$('#play').textContent='Play';setTime(0);};$('#seek').oninput=e=>setTime(e.target.value);$('#stage20').onclick=()=>setTime(manifest.stage20_reference_s);$('#final').onclick=()=>setTime(manifest.duration_s);
let lastUpdate=0;function animate(now){const dt=Math.min((now-last)/1000,.1);last=now;if(playing&&now-lastUpdate>80){setTime(time+(now-lastUpdate)/1000*10*+$('#speed').value);lastUpdate=now;if(time>=manifest.duration_s){playing=false;$('#play').textContent='Play';}}else if(!playing)lastUpdate=now;for(const p of panels)p.render();causalMap?.render();renderFrames++;requestAnimationFrame(animate);}setTime(0);requestAnimationFrame(animate);
window.rgbdFull={setTime,get playing(){return playing;},snapshot:()=>({time,renderFrames,stats,panels:panels.map(p=>({id:p.id,index:p.frame?.index,epoch:p.active?.epoch,visiblePoints:p.visiblePoints||0,maxVisibleFirst:p.active&&p.visiblePoints?p.active.first[p.visiblePoints-1]:null,maxPathEnd:p.active&&p.active.path.geometry.drawRange.count?p.active.ends[p.active.path.geometry.drawRange.count/2-1]:null,rgbPreviewIndex:p.lastRGB,cache:p.cache.size,imageCache:p.images.size,camera:p.camera.position.toArray(),target:p.controls.target.toArray(),rgb:p.el.querySelector('.rgb').currentSrc,state:p.el.querySelector('.state').textContent}))})};
