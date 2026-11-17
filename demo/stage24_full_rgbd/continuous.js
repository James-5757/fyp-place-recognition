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

const manifest=await load('continuous/manifest.json');
const trajectories=await load('continuous/'+manifest.trajectory);
const scene=new THREE.Scene();
scene.background=new THREE.Color('#0d1824');
const camera=new THREE.PerspectiveCamera(48,1,.02,4000);
const renderer=new THREE.WebGLRenderer({antialias:true});
renderer.setPixelRatio(Math.min(devicePixelRatio,1.5));
$('#map').append(renderer.domElement);
const orbit=new OrbitControls(camera,renderer.domElement);
orbit.enableDamping=true;
const colors={robot1:0x61caff,robot3:0xffa84f};
const material=new THREE.PointsMaterial({size:.12,vertexColors:true});
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
  scene.add(marker);
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
    scene.add(line);
    paths[robot].push({epoch,line,steps});
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
  scene.add(bridge);
  const jumpIndex=connectors.length;
  connectors.push({entry,bridge});
  $('#jump').add(new Option(`${entry.robot} · ${entry.previous_epoch} → ${entry.epoch} · ${entry.t.toFixed(1)} s`,jumpIndex));
}

function visible(robot,epoch) {
  return bots[robot].checked && (!epochSelect.value || epochSelect.value===`${robot}:${epoch}`);
}
async function loadChunk(chunk) {
  if (chunk.cloud) return;
  const raw=await load('continuous/'+chunk.file,true);
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
  scene.add(chunk.lod);
  chunk.cloud=new THREE.Points(geo,material);
  scene.add(chunk.cloud);
}

function renderTime() {
  displayedPoints=0;
  displayedChunks=0;
  for (const chunk of manifest.chunks) {
    if (!chunk.cloud) continue;
    const count=chunk.steps[upper(chunk.steps,time)-1]?.[1]??0;
    const shown=visible(chunk.robot,chunk.epoch) && count>0;
    const reduced=$('#density').value==='reduced';
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
    marks[robot].visible=!!row && visible(robot,row[2]);
    if (row) marks[robot].position.fromArray(row,3);
    for (const path of paths[robot]) {
      path.line.visible=visible(robot,path.epoch);
      path.line.geometry.setDrawRange(0,path.steps[upper(path.steps,time)-1]?.[1]??0);
    }
  }
  for (const {entry,bridge} of connectors) bridge.visible=time>=entry.t && visible(entry.robot,entry.epoch);
  const event=[...orderedBoundaries].reverse().find(entry=>entry.t<=time);
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
    for (let i=0;i<count;i++) bounds.expandByPoint(v.fromArray(rows[i],3));
  }
  bounds.expandByScalar(full?5:8);
  const center=bounds.getCenter(v), size=bounds.getSize(new THREE.Vector3()).length();
  orbit.target.copy(center);
  camera.position.copy(center).add(new THREE.Vector3(...(top?[0,size,.01]:[size*.55,size*.54,size*.65])));
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
$('#reset').onclick=async ()=>{stop();await seek(0);fit();};
$('#end').onclick=async ()=>{stop();await seek(manifest.duration);fit(true,true);};
epochSelect.onchange=renderTime;
$('#density').onchange=renderTime;
new ResizeObserver(([item])=>{
  const {width,height}=item.contentRect;
  if (!width || !height) return;
  renderer.setSize(width,height,false);
  camera.aspect=width/height;
  camera.updateProjectionMatrix();
}).observe($('#map'));

renderer.setAnimationLoop(now=>{
  const dt=Math.min(.1,(now-lastTick)/1000);
  lastTick=now;
  if (playing && !applying) {
    if (time>=manifest.duration) stop();
    else seek(time+dt*manifest.compression*Number($('#speed').value));
  }
  const robot=$('#follow').value;
  if (robot && marks[robot].visible) {
    const offset=marks[robot].position.clone().sub(orbit.target).multiplyScalar(1-Math.exp(-6*dt));
    orbit.target.add(offset);
    camera.position.add(offset);
  }
  orbit.update();
  renderer.render(scene,camera);
});

await seek(0);
fit();
for (const id of ['play','reset','time','end']) $(`#${id}`).disabled=failed;
if (new URLSearchParams(location.search).has('debug')) {
  window.stage27={manifest,seek,fit,scene,state:()=>({
    time,applying,failed,displayedPoints,displayedChunks,playing,
    marks:Object.fromEntries(Object.entries(marks).map(([id,m])=>[id,{visible:m.visible,xyz:m.position.toArray()}])),
    chunks:manifest.chunks.filter(c=>c.cloud?.visible).map(c=>[c.file,c.cloud.geometry.drawRange.count,c.cloud.uuid]),
  })};
}
