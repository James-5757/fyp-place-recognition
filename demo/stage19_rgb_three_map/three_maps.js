import * as THREE from '../progressive_multi_robot/vendor/three.module.js';
import {OrbitControls} from '../progressive_multi_robot/vendor/OrbitControls.js';
import {Map3DView} from '../progressive_multi_robot/map3d_view.js';
const colours={robot1:0x39bbff,robot3:0xffa43a};
function dispose(g){g.traverse(o=>{o.geometry?.dispose();o.material?.dispose();});}
export class ThreeMaps extends Map3DView {
 constructor(host,bounds,metrics,local){
  super(host,bounds,metrics);this.local=local;this.parts=[];this.geometryCache=new Map();this.localKey=-1;
  this.controls.disconnect();this.controls.connect(document.getElementById('panelGlobal'));
  this.localControls=this.localCameras.map((c,i)=>{const control=new OrbitControls(c,document.getElementById(i?'panel3':'panel1'));control.enableDamping=true;return control;});
  for(const [i,r]of ['robot1','robot3'].entries()){
   const scene=[this.local1,this.local3][i],rows=local.trajectory[r];
   const geometry=new THREE.BufferGeometry().setFromPoints(rows.map(v=>new THREE.Vector3(...v.slice(2,5))));
   const path=new THREE.Line(geometry,new THREE.LineBasicMaterial({color:colours[r]}));scene.add(path);this.parts.push(path);
   const b=local.bounds[r];this.fitCamera(this.localCameras[i],b,'perspective',false);this.localControls[i].target.set((b[0]+b[3])/2,(b[1]+b[4])/2,(b[2]+b[5])/2);
  }
  this.referenceLocal=this.points(local.robot3_points,colours.robot3,6000);this.local3.add(this.referenceLocal);
  // Saved global map rows contain position only; a sphere avoids inventing heading.
  this.reference.geometry.dispose();this.reference.geometry=new THREE.SphereGeometry(1.5,12,8);
  this.encounterHalos=[this.local1,this.local3].map(s=>{const h=new THREE.Mesh(new THREE.SphereGeometry(3,16,10),new THREE.MeshBasicMaterial({color:0x48ff9f,wireframe:true}));h.visible=false;s.add(h);return h;});
  this.localCameras.forEach(c=>{c.aspect=this.w/3/this.h;c.updateProjectionMatrix();});
 }
 resize(){super.resize();this.camera.aspect=this.w/3/this.h;this.camera.updateProjectionMatrix();for(const c of this.localCameras){c.aspect=this.w/3/this.h;c.updateProjectionMatrix();}}
 setState(s,c){
  this.context=c;this.pre=c.pre;
  const n=c.kf-149;this.parts[0].geometry.setDrawRange(0,n);
  for(const chunk of this.local.robot1_chunks){
   if(chunk.kf<=c.kf&&!this.geometryCache.has(chunk.kf)){const p=this.points(chunk.points,colours.robot1,180);this.geometryCache.set(chunk.kf,p);this.local1.add(p);this.metrics.geometry_builds++;}
   if(this.geometryCache.has(chunk.kf))this.geometryCache.get(chunk.kf).visible=chunk.kf<=c.kf&&document.getElementById('mapR1').checked;
  }
  const localRow=this.local.trajectory.robot1[n-1];this.localHeads[0].position.fromArray(localRow.slice(2,5));
  this.localHeads[0].quaternion.fromArray(this.local.quaternions.robot1[n-1]);
  const candidate=c.event&&c.phase!=='QUERY ARRIVED'?c.event.candidate:234;
  const row=this.local.trajectory.robot3[candidate-234];this.localHeads[1].position.fromArray(row.slice(2,5));
  this.localHeads[1].quaternion.fromArray(this.local.quaternions.robot3[candidate-234]);
  for(const [i,r]of ['robot1','robot3'].entries()){const h=this.encounterHalos[i];h.visible=!!c.event&&c.phase!=='QUERY ARRIVED';if(h.visible)h.position.fromArray((i?row:localRow).slice(2,5));}
  this.localControls.forEach(x=>x.update());
  if(this.pre){if(this.group)this.group.visible=false;this.head.visible=false;this.reference.visible=false;}
  else{
   const key=s.file||s.name;
   if(this.lastGlobal!==key){
    if(this.group){this.scene.remove(this.group);dispose(this.group);}
    this.group=this.makeGroup(s,s.K,c.final?10000:2500);this.scene.add(this.group);this.lastGlobal=key;this.metrics.map_state_swaps++;
    const b=this.bounds[s.file]?.global||this.bounds['FINAL_STREAM_END.json'].global;
    this.fitCamera(this.camera,b,c.final?'top':'perspective',false);
   }
   this.group.visible=true;this.head.visible=true;this.reference.visible=true;
   const look=this.group.userData.lookup.robot3;this.reference.position.fromArray(look.get(candidate)||look.values().next().value);
  }
  this.applyVisibility();
 }
 applyVisibility(){
  if(!this.parts)return;
  for(const [i,r]of ['robot1','robot3'].entries())this.parts[i].visible=document.getElementById('mapTraj').checked;
  if(this.referenceLocal)this.referenceLocal.visible=document.getElementById('mapR3').checked;
  for(const [kf,p]of this.geometryCache)p.visible=kf<=this.context.kf&&document.getElementById('mapR1').checked;
  if(this.group){const d=this.group.userData;for(const r of ['robot1','robot3']){d.maps[r].visible=document.getElementById(r==='robot1'?'mapR1':'mapR3').checked;d.trajectories[r].visible=document.getElementById('mapTraj').checked;}d.history.visible=this.loopMode==='all';if(d.newest)d.newest.visible=this.loopMode!=='off';}
 }
 preset(mode){
  this.mode=mode;for(const [i,r]of ['robot1','robot3'].entries()){const b=this.local.bounds[r];this.fitCamera(this.localCameras[i],b,mode==='top'?'top':'perspective',false);this.localControls[i].target.set((b[0]+b[3])/2,(b[1]+b[4])/2,(b[2]+b[5])/2);}
  if(this.group){const b=this.bounds[this.group.userData.file]?.global||this.bounds['FINAL_STREAM_END.json'].global;this.fitCamera(this.camera,b,mode==='top'?'top':'perspective',false);}
 }
 animate(pose){
  if(!this.pre&&pose){this.head.position.fromArray(pose.slice(2,5));if(pose.length>=9)this.head.quaternion.fromArray(pose.slice(5,9));}
  this.controls.update();this.localControls.forEach(x=>x.update());
  this.renderer.setScissorTest(true);
  for(const [i,scene]of [this.local1,this.local3,this.scene].entries()){
   const x=Math.floor(i*this.w/3),w=Math.floor((i+1)*this.w/3)-x;
   this.renderer.setViewport(x,0,w,this.h);this.renderer.setScissor(x,0,w,this.h);
   this.renderer.render(scene,i<2?this.localCameras[i]:this.camera);
  }
  this.renderer.setScissorTest(false);
  if(this.insetEvent){this.inset.setSize(200,115,false);this.inset.render(this.insetScene,this.insetCamera);}
  this.metrics.gpu_render_frames++;
 }
 pointCount(){return super.pointCount()+[...this.geometryCache.values()].filter(p=>p.visible).reduce((a,p)=>a+p.userData.pointCount,0)+(this.referenceLocal?.userData.pointCount||0);}
}
