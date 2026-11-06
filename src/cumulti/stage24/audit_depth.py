"""Read camera-depth metadata and one real PointCloud2; never use Ouster data."""
import shutil,sqlite3,tempfile,time,zipfile
import yaml
from rosbags.typesys import Stores,get_typestore,get_types_from_msg
from common import *

def run():
    out=B/'depth_camera_audit.json'
    if out.exists():raise RuntimeError('Refuse to overwrite depth audit')
    B.mkdir(parents=True,exist_ok=True);access=[];camera_guard(access);reports={};store=get_typestore(Stores.ROS2_HUMBLE)
    for robot in ['robot1','robot2','robot3']:
        archive=RAW/'main_campus'/robot/f'{robot}_main_campus_camera_depth.zip'
        if not archive.exists():reports[robot]=dict(archive=str(archive),available=False);continue
        with zipfile.ZipFile(archive) as z:
            meta=yaml.safe_load(z.read(next(n for n in z.namelist() if n.endswith('metadata.yaml'))))['rosbag2_bagfile_information']
            report=dict(archive=str(archive),available=True,archive_bytes=archive.stat().st_size,metadata=meta,
                depth_image_topic_available=any(t['topic_metadata']['type']=='sensor_msgs/msg/Image' for t in meta['topics_with_message_count']),
                message_decode='NOT_RUN_METADATA_ONLY' if robot=='robot2' else 'PENDING')
            if robot=='robot1':
                start=time.monotonic();print('EXPANDING_CAMERA_DEPTH_SQLITE',flush=True)
                member=next(n for n in z.namelist() if n.endswith('.db3'))
                # Preserve a completed read-only work copy if decoding fails;
                # avoid repeating large ZIP expansion for an engineering repair.
                temp=B/'depth_work';temp.mkdir(exist_ok=True);db=temp/'camera_depth.db3'
                if not db.exists() or db.stat().st_size!=z.getinfo(member).file_size:
                    with z.open(member) as src,db.open('wb') as target:shutil.copyfileobj(src,target,32*1024**2)
                    dump(temp/'extraction_receipt.json',dict(archive=str(archive),member=member,bytes=db.stat().st_size,CRC_validated_at_EOF=True))
                with sqlite3.connect(f'file:{db}?mode=ro&immutable=1',uri=True) as c:
                    definitions=c.execute('select topic_type,encoding,encoded_message_definition from message_definitions').fetchall()
                    for datatype,encoding,definition in definitions:
                        if encoding=='ros2msg':store.register(get_types_from_msg(definition,datatype))
                    report['embedded_message_definitions']=[dict(type=t,encoding=e,definition=d) for t,e,d in definitions]
                    topics=c.execute('select id,name,type from topics').fetchall();report['topics']=topics
                    for tid,name,typ in topics:
                        if not name.endswith(('depth_to_color','camera_info','/points')):continue
                        row=c.execute('select id,timestamp,data from messages where topic_id=? order by id limit 1',(tid,)).fetchone()
                        if not row:continue
                        m=store.deserialize_cdr(row[2],typ)
                        if name.endswith('depth_to_color'):
                            R=m.rotation.reshape(3,3,order='F');report['T_color_optical_from_depth_optical']=dict(topic=name,rotation_serialized=m.rotation.tolist(),translation_m=m.translation.tolist(),rotation_matrix=R.tolist(),determinant=float(np.linalg.det(R)),orthogonality_error=float(np.max(abs(R.T@R-np.eye(3)))),matrix_storage='column-major per realsense2_camera/msg/Extrinsics; not a mounting transform')
                        elif name.endswith('camera_info'):
                            report['camera_info']=dict(topic=name,width=int(m.width),height=int(m.height),K=m.k.tolist(),D=m.d.tolist(),distortion_model=m.distortion_model,frame_id=m.header.frame_id)
                        else:
                            fields=[dict(name=f.name,offset=int(f.offset),datatype=int(f.datatype),count=int(f.count)) for f in m.fields]
                            names={f.name:f for f in m.fields};dtype=np.dtype({'names':['x','y','z'],'formats':['<f4']*3,'offsets':[names[n].offset for n in ['x','y','z']],'itemsize':int(m.point_step)})
                            xyz=np.ndarray((int(m.height),int(m.width)),dtype=dtype,buffer=m.data,strides=(int(m.row_step),int(m.point_step)))
                            v=np.stack([xyz[n].ravel() for n in ['x','y','z']],axis=1);v=v[np.isfinite(v).all(1)]
                            report['sample_pointcloud']=dict(topic=name,width=int(m.width),height=int(m.height),point_step=int(m.point_step),row_step=int(m.row_step),frame_id=m.header.frame_id,fields=fields,
                                finite_points=len(v),xyz_min=v.min(0).tolist(),xyz_max=v.max(0).tolist(),positive_z_fraction=float(np.mean(v[:,2]>0)),bag_timestamp_ns=str(row[1]),header_timestamp_ns=str(int(m.header.stamp.sec)*10**9+int(m.header.stamp.nanosec)),
                                depth_units='ROS camera PointCloud2 float32 XYZ in metres; raw uint16 depth scale not recorded in an Image topic',depth_image_reconstruction='requires audited camera model/frame/z-buffer; NOT performed')
                            index=c.execute('select id,timestamp from messages where topic_id=? order by timestamp,id',(tid,)).fetchall()
                            write_csv(B/'robot1_depth_pointcloud_timestamp_index.csv',[dict(message_id=i,depth_bag_timestamp_ns=str(t)) for i,t in index]);report['depth_cloud_rate_hz']=1e9/float(np.median(np.diff([t for _,t in index])))
                    c.close()
                report.update(message_decode='PASS',db_member_CRC_validated=True,elapsed_s=time.monotonic()-start)
            reports[robot]=report
    dump(out,dict(robots=reports,GT_read=False,LiDAR_read=False,original_depth_image_scale='UNAVAILABLE_NO_UINT16_DEPTH_TOPIC'))
    dump(B/'depth_camera_access_audit.json',dict(opens=access))
    # Remove only this new stage's temporary expanded DB after reports persist.
    db=B/'depth_work/camera_depth.db3'
    if db.exists():db.unlink()
    print('DEPTH_AUDIT_COMPLETE',flush=True)

if __name__=='__main__':run()
