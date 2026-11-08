import * as THREE from '../progressive_multi_robot/vendor/three.module.js';
import {OrbitControls} from '../progressive_multi_robot/vendor/OrbitControls.js';
const $=id=>document.getElementById(id);
const response=await fetch('data.json');if(!response.ok)throw Error('data.json: HTTP '+response.status);
const data=await response.json(),views=[];let index=0,playing=false,last=0,clock=0,renderFrames=0,geometryBuilds=0;
const limited=Object.entries(data.robots).filter(([,r])=>r.status!=='PASS').map(([r])=>r);
$('trackingNotice').textContent=limited.length?`Tracking-limited prototype: ${limited.join(' / ')} did not pass the full-window continuity test. Lost frames remain untracked; reset submaps are kept separate.`:'Both short-window tracking tests passed. Full-run robustness has not been evaluated.';
const expectedBuilds=Object.values(data.robots).reduce((n,r)=>n+r.submaps.length,0);
async function binary(path){const r=await fetch(path);if(!r.ok)throw Error(path+': HTTP '+r.status);return r.arrayBuffer();}
for(const robot of ['robot1','robot3']){
 const s=data.robots[robot],root=$(robot),container=root.querySelector('.map');
 const renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,1.5));renderer.setClearColor(0x070f17);container.append(renderer.domElement);
 const scene=new THREE.Scene(),camera=new THREE.PerspectiveCamera(50,1,.02,1000);camera.up.set(0,-1,0);
 const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;const maps=new Map();
 for(const spec of s.submaps){
  const [rawxyz,rawrgb]=await Promise.all([binary(spec.positions),binary(spec.colors)]),xyz=new Float32Array(rawxyz),rgb=new Uint8Array(rawrgb);
  if(xyz.length!==spec.points*3||rgb.length!==xyz.length)throw Error(robot+' buffer mismatch');
  const group=new THREE.Group();scene.add(group);group.visible=false;
  const geo=new THREE.BufferGeometry();geo.setAttribute('position',new THREE.BufferAttribute(xyz,3));
  const linear=new Float32Array(rgb.length),col=new THREE.Color();for(let i=0;i<rgb.length;i+=3){col.setRGB(rgb[i]/255,rgb[i+1]/255,rgb[i+2]/255,THREE.SRGBColorSpace);linear[i]=col.r;linear[i+1]=col.g;linear[i+2]=col.b;}
  geo.setAttribute('color',new THREE.BufferAttribute(linear,3));geo.setDrawRange(0,0);geometryBuilds++;
  const points=new THREE.Points(geo,new THREE.PointsMaterial({vertexColors:true,size:2,sizeAttenuation:false}));group.add(points);
  const path=[];let previous=null;for(const f of s.frames){if(f.valid_pose&&f.tracking_epoch===spec.epoch){if(previous)path.push(...previous.pose.slice(0,3),...f.pose.slice(0,3));previous=f;}else previous=null;}
  const lg=new THREE.BufferGeometry();lg.setAttribute('position',new THREE.Float32BufferAttribute(path,3));lg.setDrawRange(0,0);
 const trajectory=new THREE.LineSegments(lg,new THREE.LineBasicMaterial({color:robot==='robot1'?0x45baff:0xffb048,depthTest:false}));trajectory.renderOrder=2;group.add(trajectory);
  geo.computeBoundingBox();const box=geo.boundingBox;if(box.isEmpty())box.set(new THREE.Vector3(-1,-1,-1),new THREE.Vector3(1,1,1));
  maps.set(spec.epoch,{spec,group,points,trajectory,center:box.getCenter(new THREE.Vector3()),size:box.getSize(new THREE.Vector3())});
 }
 const marker=new THREE.Mesh(new THREE.SphereGeometry(.09,12,8),new THREE.MeshBasicMaterial({color:robot==='robot1'?0x45baff:0xffb048}));marker.visible=false;scene.add(marker);
 const v={robot,s,root,renderer,scene,camera,controls,maps,marker,activeEpoch:null};
 v.fit=()=>{const m=maps.get(v.activeEpoch??s.submaps[0].epoch),distance=Math.max(m.size.x,m.size.y,m.size.z,2)/(2*Math.tan(THREE.MathUtils.degToRad(25)))/Math.min(camera.aspect,1)*.95;controls.target.copy(m.center);camera.position.copy(m.center).add(new THREE.Vector3(distance*.55,-distance*.65,-distance*.7));camera.lookAt(m.center);controls.update();};
 const resize=new ResizeObserver(()=>{const w=container.clientWidth,h=container.clientHeight;renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();});resize.observe(container);v.resize=resize;camera.aspect=container.clientWidth/container.clientHeight;
 root.querySelector('.fit').onclick=v.fit;
 root.querySelector('.showPoints').onchange=e=>{for(const m of maps.values())m.points.visible=e.target.checked;};
 root.querySelector('.showTrajectory').onchange=e=>{for(const m of maps.values())m.trajectory.visible=e.target.checked;marker.visible=e.target.checked&&s.frames[index].valid_pose;};
 root.querySelector('.stats').textContent=`${s.tracking.valid_poses}/${s.tracking.frames} tracked · ${s.submaps.length} independent tracking epoch(s) · ${s.points.toLocaleString()} cached RGB points · median bag sync ${s.bag_sync_ms.median.toFixed(2)} ms · Full-window test: ${s.status}`;
 views.push(v);
}
function update(i){
 index=Math.max(0,Math.min(59,i));$('seek').value=index;
 for(const v of views){
  const f=v.s.frames[index],m=v.maps.get(f.tracking_epoch);
  for(const [epoch,map] of v.maps)map.group.visible=epoch===f.tracking_epoch;
  if(v.activeEpoch!==f.tracking_epoch){v.activeEpoch=f.tracking_epoch;v.fit();}
  m.points.geometry.setDrawRange(0,f.points_end);
  let segments=0,previous=null;for(const row of v.s.frames.slice(0,index+1)){if(row.valid_pose&&row.tracking_epoch===f.tracking_epoch){if(previous)segments++;previous=row;}else previous=null;}
  m.trajectory.geometry.setDrawRange(0,segments*2);v.marker.visible=f.valid_pose&&v.root.querySelector('.showTrajectory').checked;if(f.pose)v.marker.position.set(...f.pose.slice(0,3));
  const state=v.root.querySelector('.state');const capped=m.spec.cap_reached&&f.points_end===m.spec.points;state.textContent=`SUBMAP ${f.tracking_epoch+1} · `+(f.valid_pose?'TRACKING':'LOST / MAP FROZEN')+(capped?' · DISPLAY CAP':'');state.style.color=f.valid_pose&&!capped?'#8bdcc1':'#ffb048';
  v.root.querySelector('.rgb').src=f.rgb;v.root.querySelector('.depth').src=f.depth;
  v.root.querySelector('.frameInfo').textContent=`Frame ${index+1}/60 · RGB original #${f.rgb_original_index} · ${f.elapsed_s.toFixed(3)} s · depth coverage ${(f.valid_depth_fraction*100).toFixed(1)}% · |Δ bag| ${f.bag_sync_ms.toFixed(2)} ms · RGB bag ns ${f.rgb_bag_ns} · depth bag ns ${f.depth_bag_ns}`;
 }
 $('time').textContent=`${views[0].s.frames[index].elapsed_s.toFixed(1)} / ${views[0].s.frames.at(-1).elapsed_s.toFixed(1)} s`;audit();
}
function pause(){playing=false;$('play').textContent='Play';}
function audit(){const a={ready:true,index,playing,render_frames:renderFrames,geometry_builds:geometryBuilds,expected_geometry_builds:expectedBuilds,LiDAR:false,GT:false,semantics:false,merged_map:false,robots:Object.fromEntries(views.map(v=>{const m=v.maps.get(v.activeEpoch);return [v.robot,{active_epoch:v.activeEpoch,visible_submaps:[...v.maps.values()].filter(s=>s.group.visible).length,points_visible:m.points.geometry.drawRange.count,points_total:m.spec.points,valid_pose:v.s.frames[index].valid_pose,camera:v.camera.position.toArray(),target:v.controls.target.toArray()}];}))};$('audit').textContent=JSON.stringify(a,null,2);return a;}
window.rgbdDemoAudit=audit;
$('play').onclick=()=>{if(playing){pause();return;}if(index===59)update(0);clock=views[0].s.frames[index].elapsed_s;last=performance.now();playing=true;$('play').textContent='Pause';};
$('reset').onclick=()=>{pause();clock=0;update(0);};$('seek').oninput=e=>{pause();update(Number(e.target.value));};$('final').onclick=()=>{pause();update(59);};
let stopped=false;function animate(now){if(stopped)return;requestAnimationFrame(animate);if(playing){clock+=Math.min((now-last)/1000,.25)*Number($('speed').value);let i=index;while(i<59&&views[0].s.frames[i+1].elapsed_s<=clock)i++;if(i!==index)update(i);if(i===59)pause();}last=now;for(const v of views){v.controls.update();v.renderer.render(v.scene,v.camera);}renderFrames++;}requestAnimationFrame(animate);
for(const id of ['play','reset','seek','final'])$(id).disabled=false;$('loading').hidden=true;update(0);
window.addEventListener('pagehide',()=>{stopped=true;for(const v of views){v.resize.disconnect();v.controls.dispose();v.scene.traverse(o=>{o.geometry?.dispose();o.material?.dispose();});v.renderer.dispose();}});
