"""Versioned lossless cost-study protocol; not a deployed network transport."""
import struct, time, zlib
import numpy as np
TYPES=['RING_KEY_UPLOAD','FULL_SC_DESCRIPTOR_UPLOAD','FULL_SC_DESCRIPTOR_REQUEST','FULL_SC_DESCRIPTOR_RESPONSE','CANDIDATE_REQUEST','POINT_CLOUD_REQUEST','POINT_CLOUD_RESPONSE','ODOMETRY_UPDATE','LOOP_CONSTRAINT','GLOBAL_POSE_CORRECTION','MAP_CHUNK','ACK_OR_CONTROL']
HEADER=struct.Struct('<4sBBBBB3xqqqQI')
CLOUD=struct.Struct('<IB3x')
FACTOR=struct.Struct('<Bqq')

def encode(kind,sender,receiver,robot,kf,event,stamp,payload):
    return HEADER.pack(b'FYP1',1,TYPES.index(kind),sender,receiver,robot,int(kf),int(event),int(stamp),len(payload),zlib.crc32(payload))+payload

def decode(wire):
    magic,version,kind,sender,receiver,robot,kf,event,stamp,n,crc=HEADER.unpack_from(wire)
    assert magic==b'FYP1' and version==1 and kind<len(TYPES) and len(wire)==HEADER.size+n
    body=wire[HEADER.size:];assert zlib.crc32(body)==crc
    return dict(message_type=TYPES[kind],sender=sender,receiver=receiver,robot_id=robot,original_keyframe_id=kf,logical_event_id=event,source_timestamp_ns=stamp),body

def cloud_body(p):
    p=np.asarray(p,dtype='<f4',order='C');assert p.ndim==2 and p.shape[1] in (3,4)
    return CLOUD.pack(len(p),p.shape[1])+p.tobytes()

def parse_cloud(b):
    n,fields=CLOUD.unpack_from(b);assert fields in (3,4) and len(b)==CLOUD.size+n*fields*4
    return np.frombuffer(b,offset=CLOUD.size,dtype='<f4').reshape(n,fields)

def factor_body(candidate_robot,query,candidate,matrix,sigmas,huber):
    return FACTOR.pack(candidate_robot,int(query),int(candidate))+np.asarray(matrix,dtype='<f8').reshape(16).tobytes()+np.asarray(sigmas,dtype='<f8').tobytes()+struct.pack('<d',huber)

def parse_factor(b):
    robot,q,c=FACTOR.unpack_from(b);a=np.frombuffer(b,offset=FACTOR.size,dtype='<f8');assert a.size==23
    return robot,q,c,a[:16].reshape(4,4),a[16:22],float(a[22])

def measured(kind,sender,receiver,robot,kf,event,stamp,body):
    t=time.perf_counter_ns();cpu=time.process_time_ns();wire=encode(kind,sender,receiver,robot,kf,event,stamp,body);ser=time.perf_counter_ns()-t;scpu=time.process_time_ns()-cpu
    t=time.perf_counter_ns();meta,restored=decode(wire);des=time.perf_counter_ns()-t;assert restored==body
    t=time.perf_counter_ns();compressed=zlib.compress(wire,1);comp=time.perf_counter_ns()-t
    t=time.perf_counter_ns();back=zlib.decompress(compressed);decomp=time.perf_counter_ns()-t;assert back==wire
    return dict(payload_bytes=len(body),framing_bytes=HEADER.size,serialized_total_bytes=len(wire),compression_method='zlib1_whole_message_lossless',compressed_total_bytes=len(compressed),serialization_time_ms=ser/1e6,deserialization_time_ms=des/1e6,compression_time_ms=comp/1e6,decompression_time_ms=decomp/1e6,serialization_cpu_ms=scpu/1e6)

def schema():
    return dict(status='COST_STUDY_PROTOCOL_NOT_DEPLOYED',byte_order='little endian',header_format=HEADER.format,header_bytes=HEADER.size,header_fields=['magic4','version_u8','type_u8','sender_u8','receiver_u8','robot_u8','padding3','keyframe_i64','event_i64','original_source_timestamp_ns_i64','payload_length_u64','CRC32_u32'],addressing={'0':'central','1':'robot1','3':'robot3'},payloads={
        'RING_KEY_UPLOAD':'20 float32 values; dimensions fixed by schema',
        'FULL_SC_DESCRIPTOR_UPLOAD':'20x60 float32 row-major',
        'FULL_SC_DESCRIPTOR_RESPONSE':'20x60 float32 row-major',
        'FULL_SC_DESCRIPTOR_REQUEST':'empty body; target robot/KF in header',
        'CANDIDATE_REQUEST':'<HBB: M=1000, TopK=3, database robot=3',
        'POINT_CLOUD_REQUEST':'u8 requested field count=3 (XYZ); target in header',
        'POINT_CLOUD_RESPONSE':'<IB3x count/fields/pad + original float32 XYZ; XYZI alternative benchmarked, intensity not consumed by frozen GICP',
        'LOOP_CONSTRAINT':'<Bqq candidate robot/query KF/candidate KF + 16 float64 SE3 + 6 float64 tangent sigmas [rx,ry,rz,tx,ty,tz] + float64 Huber delta',
        'ODOMETRY_UPDATE':'<Bq kind/previousKF + 16 float64 SE3 + 6 float64 tangent sigmas; kind0 relative odom,1 global prior/initial R1,2 initial R3 local pose (not central prior)',
        'GLOBAL_POSE_CORRECTION':'7 float64 XYZ+xyzw; header identifies robot/KF; schema/microtest only, not simulated send',
        'MAP_CHUNK':'same point encoding as cloud, from actual Stage20 map; microbenchmark only, no assumed distribution schedule',
        'ACK_OR_CONTROL':'u8 status; synthetic schema test only; no unnecessary ACK invented'},network_bytes_actually_transmitted='NOT_MEASURED',transport_packet_headers='NOT_MEASURED; header is application framing only',compression='independent zlib level1 entire message; uncompressed is primary',source_hash='offline trace provenance only; not an uncounted transmitted field')
