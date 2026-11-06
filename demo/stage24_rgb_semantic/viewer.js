import * as THREE from '../progressive_multi_robot/vendor/three.module.js';
import {OrbitControls} from '../progressive_multi_robot/vendor/OrbitControls.js';
const $=id=>document.getElementById(id),fetchJSON=async p=>{const r=await fetch(p);if(!r.ok)throw Error(`${p}: HTTP ${r.status}`);return r.json();};
const data=await fetchJSON('data.json');$('status').textContent=data.status;$('reason').textContent=data.reason;$('coordinates').textContent=data.metric_coordinate_policy;
const renderer=new THREE.WebGLRenderer({antialias:true});renderer.setPixelRatio(Math.min(devicePixelRatio,1.5));renderer.setClearColor(0x07111a);$('canvas').append(renderer.domElement);
const scene=new THREE.Scene(),camera=new THREE.PerspectiveCamera(48,1,.1,10000);camera.up.set(0,0,1);const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;
const clouds={},paths={},positions={},robotColors={robot1:0x39bbff,robot3:0xffa43a};let builds=0,frames=0;
function line(rows,color){return new THREE.Line(new THREE.BufferGeometry().setFromPoints(rows.map(p=>new THREE.Vector3(...p))),new THREE.LineBasicMaterial({color}));}
for(const robot of ['robot1','robot3']){
 const spec=data.maps[robot],response=await fetch(spec.file);if(!response.ok)throw Error(spec.file+' missing');const xyz=new Float32Array(await response.arrayBuffer());if(xyz.length!==spec.points*3)throw Error('Point count mismatch');
 const geometry=new THREE.BufferGeometry();geometry.setAttribute('position',new THREE.BufferAttribute(xyz,3));const colors=new Float32Array(xyz.length);geometry.setAttribute('color',new THREE.BufferAttribute(colors,3));
 clouds[robot]=new THREE.Points(geometry,new THREE.PointsMaterial({size:2,sizeAttenuation:false,vertexColors:true}));scene.add(clouds[robot]);builds++;
 paths[robot]=line(data.trajectory[robot].map(r=>r.slice(1,4)),robotColors[robot]);scene.add(paths[robot]);positions[robot]=new Map(data.trajectory[robot].map(r=>[r[0],r.slice(1,4)]));
}
const loopXYZ=[];for(const[q,c]of data.loops)loopXYZ.push(...positions.robot1.get(q),...positions.robot3.get(c));const lg=new THREE.BufferGeometry();lg.setAttribute('position',new THREE.Float32BufferAttribute(loopXYZ,3));const loops=new THREE.LineSegments(lg,new THREE.LineBasicMaterial({color:0x4fe59a,transparent:true,opacity:.4}));scene.add(loops);
const box=new THREE.Box3();for(const p of Object.values(clouds)){p.geometry.computeBoundingBox();box.union(p.geometry.boundingBox);}const center=box.getCenter(new THREE.Vector3()),size=box.getSize(new THREE.Vector3());
function fit(mode){const distance=Math.max(size.x,size.y,size.z)/(2*Math.tan(THREE.MathUtils.degToRad(24)))/Math.min(camera.aspect,1)*1.15;
 camera.position.copy(center).add(mode==='top'?new THREE.Vector3(0,-.001,distance):new THREE.Vector3(.45*distance,-.65*distance,.7*distance));controls.target.copy(center);camera.lookAt(center);controls.update();}
function audit(){const a={ready:true,mode:$('mode').value,geometry_builds:builds,render_frames:frames,observed_RGB_3D:0,labeled_semantic_3D:0,unknown_fraction:1,camera_position:camera.position.toArray(),camera_target:controls.target.toArray(),
 displayed_points:Object.fromEntries(Object.entries(clouds).map(([r,p])=>[r,p.visible?p.geometry.attributes.position.count:0])),metric_XYZ_changed:false,projection_status:'BLOCKED_MISSING_OPTICAL_EXTRINSIC'};$('audit').textContent=JSON.stringify(a,null,2);window.stage24Audit=()=>({...a,render_frames:frames,camera_position:camera.position.toArray(),camera_target:controls.target.toArray()});}
function apply(){const mode=$('mode').value;for(const[r,p]of Object.entries(clouds)){
 p.visible=$(r==='robot1'?'r1':'r3').checked;paths[r].visible=p.visible&&$('traj').checked;
 const color=new THREE.Color(mode==='robot'?robotColors[r]:0x77838f),a=p.geometry.attributes.color;for(let i=0;i<a.count;i++)a.setXYZ(i,color.r,color.g,color.b);a.needsUpdate=true;}
 loops.visible=$('loops').checked;$('modeLabel').textContent=mode==='robot'?'Robot provenance: blue R1 / orange R3':'UNKNOWN attributes only · no verified 3D '+(mode==='rgb'?'RGB colors':'semantic labels');audit();}
for(const id of ['mode','r1','r3','traj','loops'])$(id).onchange=apply;
$('perspective').onclick=()=>fit('perspective');$('top').onclick=()=>fit('top');$('fit').onclick=()=>fit('perspective');
new ResizeObserver(()=>{const w=$('canvas').clientWidth,h=$('canvas').clientHeight;renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();}).observe($('canvas'));
const resizeWidth=$('canvas').clientWidth,resizeHeight=$('canvas').clientHeight;renderer.setSize(resizeWidth,resizeHeight,false);camera.aspect=resizeWidth/resizeHeight;camera.updateProjectionMatrix();fit('perspective');
$('counts').textContent=['robot1','robot3'].map(r=>`${r}: ${data.maps[r].points.toLocaleString()} display points / ${data.maps[r].full_voxel_points.toLocaleString()} full voxels`).join(' · ');
const colors=['#6e7882','#d2aa6e','#a05abe','#64dc41','#198237','#f04b41','#9b6e3c'];for(const[label,i]of data.categories.map((n,i)=>[n,i])){const d=document.createElement('div'),s=document.createElement('span');s.className='swatch';s.style.background=colors[i];d.append(s,document.createTextNode(label));$('legend').append(d);}
data.samples.forEach((s,i)=>{const o=document.createElement('option');o.value=i;o.textContent=`${s.robot_id} · RGB #${s.original_RGB_message_index}`;$('sample').append(o);});
function sample(){const s=data.samples[Number($('sample').value)];$('rgb').src=s.rgb;$('semanticImage').src=s.semantic;$('imageInfo').textContent=`${s.robot_id} · camera bag ns ${s.camera_bag_timestamp_ns} · model unknown ${(s.unknown_fraction*100).toFixed(1)}%`;}
$('sample').onchange=sample;
// Presentation-only default: the third member of the fixed twelve-image
// cohort. Opening overexposure/unknown predictions remain selectable.
$('sample').value=String(Math.min(2,data.samples.length-1));sample();apply();controls.addEventListener('change',audit);
let disposed=false;function animate(){if(disposed)return;requestAnimationFrame(animate);controls.update();renderer.render(scene,camera);frames++;}animate();
window.addEventListener('pagehide',()=>{disposed=true;controls.dispose();scene.traverse(o=>{o.geometry?.dispose();o.material?.dispose();});renderer.dispose();});
