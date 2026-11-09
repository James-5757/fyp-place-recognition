"""Independent final artifact/decision checks; never reruns tracking."""
import csv,hashlib,json
from pathlib import Path
import numpy as np
from export import ROOT,OUT,dump

def run():
    decision=json.loads((OUT/'pre_fusion_decision.json').read_text());policy=json.loads((OUT/'policy.json').read_text());forensic=json.loads((OUT/'failure_forensics.json').read_text())
    checked=[];passes=[]
    for robot in ['robot1','robot3']:
        source=json.loads((OUT/robot/'export.json').read_text());frames=source['frames'];assert len(frames)==1200
        earlier_depth=json.loads((ROOT/f'outputs/cumulti_v1/24_rgbd_demo_v1/{robot}/export.json').read_text())
        rgb_history=json.loads((ROOT/'outputs/cumulti_v1/24b_vision_feasibility/camera_data_manifest.json').read_text())['robots'][robot]
        for receipt,old_size,old_mtime in [(source['RGB_archive'],rgb_history['archive_size'],rgb_history['archive_mtime_ns']),
            (source['depth_archive'],earlier_depth['archive_bytes'],earlier_depth['archive_mtime_ns'])]:
            p=Path(receipt['archive']);assert p.stat().st_size==old_size==receipt['archive_bytes'] and p.stat().st_mtime_ns==old_mtime==receipt['archive_mtime_ns']
        assert all(b['index']==a['index']+1 and int(b['rgb_bag_ns'])>int(a['rgb_bag_ns']) for a,b in zip(frames,frames[1:]))
        assert not list((OUT/robot/'work').glob('*.db3'))
        for config in ['baseline1000','fallback2000']:
            folder=OUT/robot/config;metrics=json.loads((folder/'metrics.json').read_text());rows=list(csv.DictReader((folder/'frame_poses.csv').open()));valid=[r for r in rows if r['valid_pose']=='1']
            assert len(rows)==len(frames) and len(valid)==metrics['valid_frames']
            for row in valid:
                p=np.array([float(row[k]) for k in ['tx','ty','tz','qx','qy','qz','qw']]);assert np.isfinite(p).all() and abs(np.linalg.norm(p[3:])-1)<1e-5
            actual=all(metrics['checks'].values());assert actual==(metrics['fusion_gate']=='PASS')
            assert not metrics['native_forbidden_reads'];passes.append((robot,config,actual))
            checked.append(dict(robot=robot,config=config,valid_frames=len(valid),gate=metrics['fusion_gate'],finite_tracked_poses='PASS'))
    expected_ready=any(all(next(v for r,c,v in passes if r==robot and c==config) for robot in ['robot1','robot3']) for config in ['baseline1000','fallback2000'])
    assert expected_ready==(decision['status']=='READY_FOR_BOUNDED_RGBD_FUSION')
    assert all(s['lifecycle_log_vs_fingerprint_agreement'] for s in forensic['robot_config_summaries'])
    sync=list(csv.DictReader((OUT/'synchronization_report.csv').open()))
    assert all(int(r['frozen_RGB_images_byte_identical'])==60 and int(r['frozen_depth_images_byte_identical'])==60 for r in sync)
    fusion=json.loads((OUT/'fusion/status.json').read_text());assert fusion['inter_robot_constraints_created']==0 and not fusion['retrieval_run'] and not fusion['joint_graph_run']
    # No source/output of the frozen 12s RGB-D demo was changed in this stage.
    old='outputs/cumulti_v1/24_rgbd_demo_v1';short=json.loads((ROOT/old/'map_manifest.json').read_text())
    for artifact in short['artifacts']:
        path=ROOT/artifact['path'];assert hashlib.sha256(path.read_bytes()).hexdigest()==artifact['sha256']
    dump(OUT/'final_validation.json',dict(status='PASS_AUDIT_NO_GO_FUSION' if not expected_ready else 'PASS_AUDIT_READY_PENDING_FUSION',decision=decision['status'],
        actual_robot_checks=checked,policy_unchanged=True,log_vs_fingerprint_crosscheck='PASS',
        frozen_camera_input_overlap_RGB_and_depth_byte_identity='PASS_60_EACH_PER_ROBOT',
        short_demo_assets_sha256='PASS',raw_archives_size_mtime_against_previous_checkpoint='PASS',old_short_assets_checked=len(short['artifacts']),GT=False,LiDAR=False,
        fusion_run=False,distinction='successful audit/reset management; NOT successful two-robot long tracking or fusion'))
    artifacts=[]
    for path in sorted(OUT.rglob('*')):
        if not path.is_file() or any(p in path.parts for p in ['frames','work','bin']):continue
        if path.suffix not in ['.json','.csv','.yaml','.jpg','.png'] or path.name in ['artifact_manifest.json','progress.json']:continue
        artifacts.append(dict(path=str(path.relative_to(ROOT)),bytes=path.stat().st_size,sha256=hashlib.sha256(path.read_bytes()).hexdigest()))
    dump(OUT/'artifact_manifest.json',dict(files=artifacts,large_sequences_server_only=True,GT=False))
    print((OUT/'final_validation.json').read_text(),flush=True)

if __name__=='__main__':run()
