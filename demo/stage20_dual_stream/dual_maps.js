import * as THREE from '../progressive_multi_robot/vendor/three.module.js';
import {ThreeMaps} from '../stage19_rgb_three_map/three_maps.js';
const robots=['robot1','robot3'],starts=[150,234],colours=[0x39bbff,0xffa43a];
export class DualMaps extends ThreeMaps {
 constructor(host,bounds,metrics,local){
  super(host,bounds,metrics,local);this.r3cache=new Map();this.fullBounds=bounds;this.mode='perspective';
  // Pinned OrbitControls.connect() takes no element argument. Bind explicitly
  // to the overlay panel that actually receives pointer and wheel events.
  this.controls.disconnect();this.controls.domElement=document.getElementById('panelGlobal');this.controls.connect();
  for(const control of [this.controls,...this.localControls])control.addEventListener('start',()=>{
   this.mode='manual';this.cameraGoal=null;
   document.querySelectorAll('[data-camera]').forEach(b=>b.setAttribute('aria-pressed','false'));
  });
 }
 update(s,e){
  const [index,ns,r,k,k1,k3,K,components,initialized]=e;
  // The inherited view is reused only for scene/buffer management. Both local
  // prefixes and both current markers are explicitly set from the event below.
  const changed=this.lastGlobal!==s.file;
  const manual=this.mode==='manual',cameraPosition=this.camera.position.clone(),cameraTarget=this.controls.target.clone();
  super.setState(s,{pre:!initialized,kf:k1,final:index===5795,event:null,phase:'STREAM'});
  if(manual){this.camera.position.copy(cameraPosition);this.controls.target.copy(cameraTarget);this.camera.lookAt(cameraTarget);}
  // Stage19 deliberately frames FINAL from above. In Stage20, keep the
  // selected camera preset across saved-state changes, including FINAL.
  if(changed&&initialized&&['perspective','top'].includes(this.mode)){
   const b=this.bounds[s.file]?.global||this.bounds['FINAL_STREAM_END.json'].global;
   this.fitCamera(this.camera,b,this.mode,false);
  }
  this.dualEnds=[k1,k3];this.currentEvent=e;
  for(const [i,robot]of robots.entries()){
   const end=this.dualEnds[i],n=Math.max(0,end-starts[i]+1),rows=this.local.trajectory[robot];
   this.parts[i].geometry.setDrawRange(0,n);this.localHeads[i].visible=n>0;
   if(n){this.localHeads[i].position.fromArray(rows[n-1].slice(2,5));this.localHeads[i].quaternion.fromArray(this.local.quaternions[robot][n-1]);}
  }
  for(const chunk of this.local.robot3_chunks){
   if(chunk.kf<=k3&&!this.r3cache.has(chunk.kf)){const p=this.points(chunk.points,colours[1],180);this.r3cache.set(chunk.kf,p);this.local3.add(p);this.metrics.geometry_builds++;}
   if(this.r3cache.has(chunk.kf))this.r3cache.get(chunk.kf).visible=chunk.kf<=k3&&document.getElementById('mapR3').checked;
  }
  if(initialized){const a=s.trajectory.robot1.at(-1),b=s.trajectory.robot3.at(-1);this.head.position.fromArray(a.slice(2,5));this.head.quaternion.fromArray(a.slice(5,9));this.reference.position.fromArray(b.slice(2,5));}
  const first=this.events[0];for(const [i,h]of this.encounterHalos.entries()){h.visible=K>0;h.position.fromArray(this.local.trajectory[robots[i]][(i?first.candidate:first.query)-starts[i]].slice(2,5));}
  this.applyVisibility();
 }
 applyVisibility(){super.applyVisibility();if(!this.r3cache)return;for(const [kf,p]of this.r3cache)p.visible=kf<=(this.dualEnds?.[1]??233)&&document.getElementById('mapR3').checked;
  this.local1.visible=document.getElementById('local1On').checked;this.local3.visible=document.getElementById('local3On').checked;this.scene.visible=document.getElementById('globalOn').checked;
 }
 animate(){
  if(this.mode==='follow1'||this.mode==='follow3'){
   const i=this.mode==='follow1'?0:1,target=this.localHeads[i].position,c=this.localCameras[i];c.position.copy(target).add(new THREE.Vector3(30,-55,40));this.localControls[i].target.copy(target);
   if(!this.pre){const p=i?this.reference.position:this.head.position;this.camera.position.copy(p).add(new THREE.Vector3(30,-55,40));this.controls.target.copy(p);}
  }
  super.animate(null);
 }
 pointCount(){return super.pointCount()+[...this.r3cache.values()].filter(p=>p.visible).reduce((a,p)=>a+p.userData.pointCount,0);}
}
