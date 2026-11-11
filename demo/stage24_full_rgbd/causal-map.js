import * as THREE from 'three';
import {OrbitControls} from '../progressive_multi_robot/vendor/OrbitControls.js';

export async function createCausalMap(root, datasets, upper, buffers, summaryPath='causal_summary.json') {
 const response = await fetch(summaryPath, {cache:'no-cache'});
 if (response.status === 404) return null;
 if (!response.ok) throw Error('Causal replay summary could not be loaded');
 const summary = await response.json();
 root.hidden = false;
 const state = root.querySelector('.state');
 const box = root.querySelector('.map');
 const select = root.querySelector('select');
 let renderer, scene, camera, controls, clouds = [], active = -1, token = 0;
 let time = 0, previousCount = -1;
 function clear() {
  for (const {cloud} of clouds) {
   scene.remove(cloud);
   cloud.geometry.dispose();
   cloud.material.dispose();
  }
  clouds = [];
 }
 function initialize() {
  if (renderer) return;
  box.hidden = false;
  scene = new THREE.Scene();
  scene.background = new THREE.Color('#060f18');
  camera = new THREE.PerspectiveCamera(48,1,.01,2000);
  camera.up.set(0,-1,0);
  renderer = new THREE.WebGLRenderer({antialias:true});
  box.append(renderer.domElement);
  controls = new OrbitControls(camera,renderer.domElement);
  new ResizeObserver(()=>{
   const {clientWidth:w,clientHeight:h} = box;
   if (!w || !h) return;
   renderer.setSize(w,h);
   camera.aspect = w/h;
   camera.updateProjectionMatrix();
  }).observe(box);
 }
 function reveal() {
  for (const {cloud, first, data} of clouds) {
   const index = upper(data.frames,time,f=>f.t)-1;
   cloud.geometry.setDrawRange(0,upper(first,index,x=>x));
  }
 }
 async function update(t) {
  time = t;
  const attempts = upper(summary.attempts_ns,Math.round(t*1e9),x=>x);
  const count = upper(summary.bridges,t,b=>b.available_s);
  if (count !== previousCount) {
   const previous = select.value;
   select.replaceChildren();
   for (let i=0;i<count;i++) {
    const b = summary.bridges[i];
    select.add(new Option(`Robot1 epoch ${b.robot1_epoch} + Robot3 epoch ${b.robot3_epoch}`,i));
   }
   if (+previous<count) select.value = previous || '0';
   previousCount = count;
  }
  select.hidden = count === 0;
  box.hidden = count === 0;
  state.textContent = count ? `${count} verified component pair(s) · showing the selected shared map only`
    : `Waiting for a verified RGB-D bridge · ${attempts}/${summary.attempts_ns.length} scheduled candidates reached`;
  if (!count) {
   if (attempts === summary.attempts_ns.length) state.textContent = `No verified RGB-D bridge after ${summary.geometry_tested ?? attempts} checks. The local maps remain independent.`;
   token++;
   active = -1;
   clear();
   return;
  }
  const chosen = +select.value;
  if (chosen === active) { reveal(); return; }
  initialize();
  const bridge = summary.bridges[chosen];
  const request = ++token;
  active = chosen;
  clear();
  try {
   const loaded = await Promise.all(datasets.map(async ([id,data])=>{
    const epoch = data.epochs.find(e=>e.epoch===bridge[id+'_epoch']);
    const [xyz,rgb,first] = await Promise.all([
     buffers(epoch.files['xyz.f32'],Float32Array),
     buffers(epoch.files['rgb.u8'],Uint8Array),
     buffers(epoch.files['first.u32'],Uint32Array)]);
    return {id,data,xyz,rgb,first};
   }));
   if (request !== token) return;
   const transform = new THREE.Matrix4().set(...bridge.transform.flat());
   for (const {id,data,xyz,rgb,first} of loaded) {
    const colors = new Float32Array(rgb.length), color = new THREE.Color();
    for (let i=0;i<rgb.length;i+=3) {
     color.setRGB(rgb[i]/255,rgb[i+1]/255,rgb[i+2]/255,THREE.SRGBColorSpace);
     colors.set([color.r,color.g,color.b],i);
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position',new THREE.BufferAttribute(xyz,3));
    geometry.setAttribute('color',new THREE.BufferAttribute(colors,3));
    if (id==='robot3') geometry.applyMatrix4(transform);
    const cloud = new THREE.Points(geometry,new THREE.PointsMaterial({size:.035,vertexColors:true}));
    cloud.frustumCulled = false;
    scene.add(cloud);
    clouds.push({cloud,first,data});
   }
   reveal();
   const bounds = new THREE.Box3(), point = new THREE.Vector3();
   for (const {cloud} of clouds) {
    const xyz = cloud.geometry.attributes.position;
    for (let i=0;i<cloud.geometry.drawRange.count;i++) bounds.expandByPoint(point.fromBufferAttribute(xyz,i));
   }
   const center = bounds.getCenter(new THREE.Vector3()), size = Math.max(2,bounds.getSize(new THREE.Vector3()).length());
   controls.target.copy(center);
   camera.position.copy(center).add(new THREE.Vector3(size*.45,-size*.5,-size*.7));
   controls.update();
  } catch (error) {
   if (request === token) state.textContent = 'Verified pair could not be rendered: '+error.message;
  }
 }
 select.onchange = ()=>update(time);
 return {update,render(){if (renderer&&!box.hidden) renderer.render(scene,camera);}};
}
