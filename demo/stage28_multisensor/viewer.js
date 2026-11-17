import * as THREE from 'three';
import {OrbitControls} from '../progressive_multi_robot/vendor/OrbitControls.js';

const $ = selector => document.querySelector(selector);
async function load(path, binary=false) {
  const response = await fetch(path);
  if (!response.ok) throw Error(`${path} (${response.status})`);
  return binary ? response.arrayBuffer() : response.json();
}
function upper(items, value) {
  let first=0, last=items.length;
  while (first<last) {
    const i=(first+last)>>>1;
    if (items[i][0]<=value) first=i+1;
    else last=i;
  }
  return first;
}

const parameters=new URLSearchParams(location.search);
const sensor=parameters.get('sensor')==='rgbd'?'rgbd':'lidar';
const selected=['robot1','robot3'].includes(parameters.get('robot'))?parameters.get('robot'):'combined';
const folder=sensor==='lidar'?'lidar/':'../stage24_full_rgbd/continuous/';
const manifest=await load(folder+'manifest.json');
const allTrajectories=await load(folder+manifest.trajectory);
const calibration=await load('gravity.json');
const alignment=calibration[sensor][selected];
manifest.robots=manifest.robots.filter(r=>selected==='combined'||r===selected);
manifest.chunks=manifest.chunks.filter(c=>manifest.robots.includes(c.robot));
manifest.boundaries=manifest.boundaries.filter(c=>manifest.robots.includes(c.robot));
manifest.transforms=manifest.transforms.filter(c=>manifest.robots.includes(c.robot));
manifest.points=manifest.chunks.reduce((n,c)=>n+c.points,0);
const trajectories=Object.fromEntries(manifest.robots.map(r=>[r,allTrajectories[r]]));
manifest.duration=Math.max(...manifest.robots.map(r=>sensor==='lidar'?trajectories[r].at(-1)[0]:
  Math.max(...manifest.chunks.filter(c=>c.robot===r).map(c=>c.end))));
const label=sensor==='lidar'?'LiDAR':'RGB-D';
$('#title').textContent=(selected==='combined'?'Combined':selected==='robot1'?'Robot1':'Robot3')+' '+label;
$('#sensor').value=sensor;
$('#robot').value=selected;
$('#gravity').textContent=alignment.method==='ROBUST_GROUND_NORMAL'
  ? 'Z up · fitted from consistent local ground planes'
  : 'Z up by axis convention · RGB-D gravity unverified';
$('#sensor').onchange=()=>switchView($('#sensor').value,selected);
$('#robot').onchange=()=>switchView(sensor,$('#robot').value);
for (const opt of [...$('#follow').options]) {
  if (opt.value && !manifest.robots.includes(opt.value)) opt.remove();
}
function switchView(nextSensor,nextRobot) {
  location.search=new URLSearchParams({sensor:nextSensor,robot:nextRobot}).toString();
}
const scene=new THREE.Scene();
scene.background=new THREE.Color('#0d1824');
const camera=new THREE.PerspectiveCamera(48,1,.02,4000);
camera.up.set(0,0,1);
const root=new THREE.Group();
root.matrixAutoUpdate=false;
root.matrix.fromArray(alignment.T_ground_from_display.flat()).transpose();
root.updateMatrixWorld(true);
scene.add(root);
const grid=new THREE.GridHelper(1500,750,0x536d86,0x294359);
grid.rotation.x=Math.PI/2;
scene.add(grid);
const groundProjector=new THREE.Group();
scene.add(groundProjector);
const renderer=new THREE.WebGLRenderer({antialias:true});
renderer.setPixelRatio(Math.min(devicePixelRatio,1.5));
$('#map').append(renderer.domElement);
const orbit=new OrbitControls(camera,renderer.domElement);
orbit.enableDamping=true;
const colors={robot1:0x61caff,robot3:0xffa84f};
const material=new THREE.PointsMaterial({size:sensor==='lidar'?.10:.12,vertexColors:true});
const robotMaterials=Object.fromEntries(Object.entries(colors).map(([robot,color])=>
  [robot,new THREE.PointsMaterial({size:material.size,color})]));
const rgbLut=Float32Array.from({length:256},(_,i)=>THREE.ColorManagement.toWorkingColorSpace(new THREE.Color().setRGB(i/255,0,0),THREE.SRGBColorSpace).r);
const paths={}, marks={}, bots={}, epochSelect=$('#epoch');
const connectors=[];
let time=0, desired=0, playing=false, applying=false, failed=false;
let displayedPoints=0, displayedChunks=0, lastTick=performance.now();
let seekSerial=0;

for (const robot of manifest.robots) {
  const label=document.createElement('label');
  label.className=robot==='robot1'?'r1':'r3';
  const visible=document.createElement('input');
  visible.type='checkbox';
  visible.checked=true;
  visible.addEventListener('change',renderTime);
  label.append(visible,robot==='robot1'?' Robot1':' Robot3');
  $('#robots').append(label);
  bots[robot]=visible;
  const rows=trajectories[robot], parts=new Map();
  const marker=new THREE.Mesh(new THREE.SphereGeometry(.35,12,8),
                              new THREE.MeshBasicMaterial({color:colors[robot]}));
  root.add(marker);
  marks[robot]=marker;
  for (const row of rows) {
    if (!parts.has(row[2])) parts.set(row[2],[]);
    parts.get(row[2]).push(row);
  }
  paths[robot]=[];
  for (const [epoch,part] of parts) {
    const positions=[], steps=[];
    for (let i=1;i<part.length;i++) {
      const a=part[i-1], b=part[i];
      if (b[1]-a[1]===1 && b[0]-a[0]<.35) positions.push(...a.slice(3,6),...b.slice(3,6));
      steps.push([b[0],positions.length/3]);
    }
    const geo=new THREE.BufferGeometry();
    geo.setAttribute('position',new THREE.Float32BufferAttribute(positions,3));
    const line=new THREE.LineSegments(geo,new THREE.LineBasicMaterial({color:colors[robot]}));
    root.add(line);
    const projected=[];
    const projectedPoint=new THREE.Vector3();
    for (let i=0;i<positions.length;i+=3) {
      projectedPoint.fromArray(positions,i).applyMatrix4(root.matrix);
      projected.push(projectedPoint.x,projectedPoint.y,0);
    }
    const flat=new THREE.BufferGeometry();
    flat.setAttribute('position',new THREE.Float32BufferAttribute(projected,3));
    const groundLine=new THREE.LineSegments(flat,line.material);
    groundProjector.add(groundLine);
    paths[robot].push({epoch,line,groundLine,steps});
    epochSelect.add(new Option(`${robot} · Epoch ${epoch}`,`${robot}:${epoch}`));
  }
}

const orderedBoundaries=[...manifest.boundaries].sort((a,b)=>a.t-b.t);
for (const entry of orderedBoundaries) {
  const rows=trajectories[entry.robot];
  const last=rows[upper(rows,entry.last_t)-1], first=rows[upper(rows,entry.t)-1];
  const geo=new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(...last.slice(3,6)),new THREE.Vector3(...first.slice(3,6))]);
  const verified=entry.uncertainty==='VERIFIED_RELATIVE_ONLY';
  const bridge=new THREE.Line(geo,new THREE.LineDashedMaterial({
    color:verified?0x67e4a5:0xffca72,dashSize:.45,gapSize:.28,
  }));
  bridge.computeLineDistances();
  root.add(bridge);
  const jumpIndex=connectors.length;
  connectors.push({entry,bridge});
  $('#jump').add(new Option(`${entry.robot} · ${entry.previous_epoch} → ${entry.epoch} · ${entry.t.toFixed(1)} s`,jumpIndex));
}

function visible(robot,epoch) {
  return bots[robot].checked && (!epochSelect.value || epochSelect.value===`${robot}:${epoch}`);
}
async function loadChunk(chunk) {
  if (chunk.cloud) return;
  const raw=await load(folder+chunk.file,true);
  if (raw.byteLength!==chunk.points*15) throw Error('Invalid map chunk '+chunk.file);
  const source=new DataView(raw), positions=new Float32Array(chunk.points*3), rgb=new Float32Array(positions.length);
  for (let i=0;i<chunk.points;i++) {
    for (let axis=0;axis<3;axis++) {
      positions[i*3+axis]=source.getFloat32(i*15+axis*4,true);
      rgb[i*3+axis]=rgbLut[source.getUint8(i*15+12+axis)];
    }
  }
  const geo=new THREE.BufferGeometry();
  geo.setAttribute('position',new THREE.BufferAttribute(positions,3));
  geo.setAttribute('color',new THREE.BufferAttribute(rgb,3));
  geo.setDrawRange(0,0);
  const lod=new THREE.BufferGeometry();
  const sparsePositions=[], sparseColors=[];
  for (let i=0;i<chunk.points;i+=4) {
    sparsePositions.push(...positions.subarray(i*3,i*3+3));
    sparseColors.push(...rgb.subarray(i*3,i*3+3));
  }
  lod.setAttribute('position',new THREE.Float32BufferAttribute(sparsePositions,3));
  lod.setAttribute('color',new THREE.Float32BufferAttribute(sparseColors,3));
  lod.setDrawRange(0,0);
  chunk.lod=new THREE.Points(lod,material);
  chunk.lod.visible=false;
  root.add(chunk.lod);
  chunk.cloud=new THREE.Points(geo,material);
  root.add(chunk.cloud);
}

function renderTime() {
  displayedPoints=0;
  displayedChunks=0;
  for (const chunk of manifest.chunks) {
    if (!chunk.cloud) continue;
    const count=chunk.steps[upper(chunk.steps,time)-1]?.[1]??0;
    const shown=visible(chunk.robot,chunk.epoch) && count>0;
    const reduced=$('#density').value==='reduced';
    const pointMaterial=$('#colorMode').value==='robot'?robotMaterials[chunk.robot]:material;
    chunk.cloud.material=chunk.lod.material=pointMaterial;
    chunk.cloud.visible=shown && !reduced;
    chunk.lod.visible=shown && reduced;
    chunk.cloud.geometry.setDrawRange(0,count);
    chunk.lod.geometry.setDrawRange(0,Math.ceil(count/4));
    if (shown) {
      displayedPoints+=count;
      displayedChunks++;
    }
  }
  for (const robot of manifest.robots) {
    const rows=trajectories[robot], index=upper(rows,time)-1;
    const row=rows[index];
    marks[robot].visible=!!row && visible(robot,row[2]) && $('#trajectories').checked;
    if (row) marks[robot].position.fromArray(row,3);
    for (const path of paths[robot]) {
      const layered=visible(robot,path.epoch) && $('#trajectories').checked;
      path.line.visible=layered && !$('#hybrid').checked;
      path.groundLine.visible=layered && $('#hybrid').checked;
      const count=path.steps[upper(path.steps,time)-1]?.[1]??0;
      path.line.geometry.setDrawRange(0,count);
      path.groundLine.geometry.setDrawRange(0,count);
    }
  }
  for (const {entry,bridge} of connectors) bridge.visible=time>=entry.t && visible(entry.robot,entry.epoch);
  const event=[...orderedBoundaries].reverse().find(entry=>entry.t<=time && visible(entry.robot,entry.epoch));
  grid.visible=$('#hybrid').checked;
  const activeRobot=manifest.robots.find(r=>marks[r].visible);
  if (activeRobot) {
    grid.position.z=marks[activeRobot].getWorldPosition(new THREE.Vector3()).z-1;
    groundProjector.position.z=grid.position.z+.02;
  }
  $('#transition').textContent=event
    ? `${event.robot} · ${event.previous_epoch} → ${event.epoch} · ${event.source} · ${event.translation_m.toFixed(2)} m / ${event.rotation_deg.toFixed(1)}° boundary gap`
    : 'No epoch boundary reached.';
  if (event?.evidence) $('#transition').append(document.createElement('br'),event.evidence);
  $('#time').value=time;
  $('#clock').textContent=`${time.toFixed(1)} / ${manifest.duration.toFixed(1)} s`;
  const epochs=manifest.robots.map(r=> {
    const rows=trajectories[r], row=rows[upper(rows,time)-1];
    return row?`${r} E${row[2]}`:`${r} waiting`;
  });
  $('#status').textContent=`Exploratory map · ${displayedPoints.toLocaleString()} points · ${epochs.join(' · ')}`;
}

async function seek(target) {
  desired=Math.max(0,Math.min(manifest.duration,target));
  const requested=++seekSerial;
  if (applying) return requested;
  applying=true;
  try {
    let completed;
    do {
      completed=seekSerial;
      const selected=desired;
      const missing=manifest.chunks.filter(c=>c.start<=selected && !c.cloud);
      for (let i=0;i<missing.length && completed===seekSerial;i+=6) {
        await Promise.all(missing.slice(i,i+6).map(loadChunk));
      }
      if (completed!==seekSerial) continue;
      time=selected;
      renderTime();
    } while (completed!==seekSerial);
  } catch (error) {
    playing=false;
    failed=true;
    $('#play').disabled=true;
    $('#status').textContent='Replay stopped · '+error.message;
  } finally { applying=false; }
  return requested;
}

function fit(top=false, full=false) {
  const bounds=new THREE.Box3();
  const v=new THREE.Vector3();
  for (const robot of manifest.robots) {
    const rows=trajectories[robot];
    const count=full?rows.length:Math.max(1,upper(rows,time));
    for (let i=0;i<count;i++) bounds.expandByPoint(v.fromArray(rows[i],3).applyMatrix4(root.matrix));
  }
  if (time>=manifest.duration) {
    for (const chunk of manifest.chunks) {
      if (!chunk.cloud || !visible(chunk.robot,chunk.epoch)) continue;
      const geo=chunk.cloud.geometry;
      if (!geo.boundingBox) geo.computeBoundingBox();
      bounds.union(geo.boundingBox.clone().applyMatrix4(root.matrix));
    }
  }
  bounds.expandByScalar(full?5:8);
  const center=bounds.getCenter(v), size=bounds.getSize(new THREE.Vector3()).length();
  orbit.target.copy(center);
  camera.position.copy(center).add(new THREE.Vector3(...(top?[0,.01,size*1.1]:[size*.60,-size*.72,size*.69])));
  orbit.update();
}

function stop() { playing=false; $('#play').textContent='Play'; }
$('#play').onclick=()=>{ if (playing) stop(); else {
  if (time===manifest.duration) seek(0);
  playing=true;
  lastTick=performance.now();
  $('#play').textContent='Pause';
}};
$('#time').max=manifest.duration;
$('#time').oninput=event=>{stop();seek(Number(event.target.value));};
$('#jump').onchange=event=>{if (event.target.value!=='') {stop();seek(orderedBoundaries[Number(event.target.value)].t);}};
$('#top').onclick=()=>fit(true);
$('#orbit').onclick=()=>fit();
$('#resetView').onclick=()=>fit(false,true);
$('#hybrid').onchange=renderTime;
$('#trajectories').onchange=renderTime;
if (sensor==='lidar') $('#density').value='reduced';
if (parameters.get('bev')==='1') $('#hybrid').checked=true;
$('#reset').onclick=async ()=>{stop();await seek(0);fit();};
$('#end').onclick=async ()=>{stop();await seek(manifest.duration);fit(true,true);};
epochSelect.onchange=renderTime;
$('#density').onchange=renderTime;
$('#colorMode').onchange=renderTime;
new ResizeObserver(([item])=>{
  const {width,height}=item.contentRect;
  if (!width || !height) return;
  renderer.setSize(width,height,false);
  camera.aspect=width/height;
  camera.updateProjectionMatrix();
}).observe($('#map'));

renderer.setAnimationLoop(now=>{
  const dt=Math.min(.3,(now-lastTick)/1000);
  lastTick=now;
  if (playing && !applying) {
    if (time>=manifest.duration) stop();
    else seek(time+dt*manifest.compression*Number($('#speed').value));
  }
  const robot=$('#follow').value;
  if (robot && marks[robot].visible) {
    const offset=marks[robot].getWorldPosition(new THREE.Vector3()).sub(orbit.target).multiplyScalar(1-Math.exp(-6*dt));
    orbit.target.add(offset);
    camera.position.add(offset);
  }
  orbit.update();
  renderer.render(scene,camera);
});

await seek(0);
fit(parameters.get('bev')==='1');
for (const id of ['play','reset','time','end']) $(`#${id}`).disabled=failed;
if (new URLSearchParams(location.search).has('debug')) {
  window.stage28={manifest,seek,fit,scene,state:()=>({
    camera:camera.position.toArray(), target:orbit.target.toArray(),
    time,applying,failed,displayedPoints,displayedChunks,playing,
    marks:Object.fromEntries(Object.entries(marks).map(([id,m])=>[id,{visible:m.visible,xyz:m.getWorldPosition(new THREE.Vector3()).toArray()}])),
    chunks:manifest.chunks.filter(c=>c.cloud?.visible||c.lod?.visible).map(c=>[c.file,c.cloud.geometry.drawRange.count,c.cloud.uuid]),
  })};
}
