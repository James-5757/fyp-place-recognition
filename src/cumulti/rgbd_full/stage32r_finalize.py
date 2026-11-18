"""Merge Stage32R search results, verify historical identities and enforce recovery gate."""
import argparse
import csv
import gzip
import hashlib
import json
import subprocess
from pathlib import Path, PurePosixPath

from stage32r_search import EXPECTED

PARENT='cd0e269495a41a2210d86fca8d001c5a95fd3f8c'
CORE=('System.cc','System.h','rgbd_full','libORB_SLAM3.so')
ROOT=Path(__file__).resolve().parents[3]
OUT=ROOT/'outputs/cumulti_v1/32r_runtime_restoration'


def load(path):
    data=path.read_bytes() if path.exists() else gzip.decompress(Path(str(path)+'.gz').read_bytes())
    return json.loads(data.decode('utf-8-sig'))


def dump(path,value):
    path.parent.mkdir(parents=True,exist_ok=True)
    path.write_text(json.dumps(value,indent=2)+'\n',encoding='utf-8')


def digest(data):
    return hashlib.sha256(data).hexdigest()


def blob(path):
    return subprocess.check_output(['git','-C',str(ROOT),'show',PARENT+':'+path])


def main():
    args=argparse.ArgumentParser()
    args.add_argument('--work',type=Path,required=True)
    parsed=args.parse_args()
    OUT.mkdir(parents=True,exist_ok=True)
    rows=[]
    coverage={}
    for machine in ('server','local'):
        directory=OUT/(machine+'_search')
        source=load(directory/'search_candidates.json')
        coverage[machine]=load(directory/'search_coverage.json')
        for r in source:
            row=dict(machine=machine,**r)
            rows.append(row)
    statuses={}
    for dependency,expected in EXPECTED.items():
        found=[r for r in rows if r['dependency']==dependency]
        matched=[r for r in found if r['sha256']==expected]
        # A mismatch is an available candidate only. It never satisfies the original dependency.
        classification=('RECOVERED_AND_HASH_VERIFIED' if matched else
                        'RECOVERED_BUT_HASH_MISMATCH' if found else 'NOT_FOUND')
        statuses[dependency]=dict(classification=classification,expected_sha256=expected,
                                  original_identity_verified=bool(matched),matching_locations=[
                                      dict(machine=r['machine'],location=r['location'],sha256=r['sha256']) for r in matched],
                                  candidate_count=len(found),
                                  candidate_hashes=sorted({r['sha256'] for r in found}),
                                  available_candidates_are_original=bool(matched))
    if all(statuses[k]['original_identity_verified'] for k in CORE):
        raise RuntimeError('Critical originals recovered. Continue with verified linkage and separate instrumentation build before finalizing.')
    receipt=load(ROOT/'outputs/cumulti_v1/32_native_tracking_forensics/stage32_run_provenance.json')
    history=receipt['historical_native_receipt']
    build_path='scripts/build_rgbd_full.sh'
    settings_path='outputs/cumulti_v1/24_rgbd_full_v1/robot3/camera.yaml'
    build_data=blob(build_path)
    settings_data=blob(settings_path)
    run_receipt=load(ROOT/'outputs/cumulti_v1/24_rgbd_full_v1/run_provenance.json')
    expected_build_hash=run_receipt['source_sha256'][build_path]
    statuses['camera.yaml']=dict(classification='RECOVERED_AND_HASH_VERIFIED',sha256=digest(settings_data),
                                verification_basis='Frozen project Git blob at '+PARENT+':'+settings_path,
                                historical_runtime_file_hash=None,settings=history['settings'])
    statuses['build_rgbd_full.sh']=dict(
        classification='RECOVERED_AND_HASH_VERIFIED' if digest(build_data)==expected_build_hash else 'RECOVERED_BUT_HASH_MISMATCH',
        sha256=digest(build_data),expected_sha256=expected_build_hash,source_path=build_path)
    sync_path='outputs/cumulti_v1/24_rgbd_full_v1/robot3/sync_pairs.csv'
    import io
    sync=list(csv.DictReader(io.StringIO(blob(sync_path).decode('utf-8-sig'))))
    original_root=PurePosixPath('/home/cas/fyp_place_recognition')
    lines=''.join(f"{r['relative_ns']} {original_root/r['rgb']} "
                  f"{original_root/'outputs/cumulti_v1/24_rgbd_full_v1/robot3/depth'/f'{i:06d}.png'}\n"
                  for i,r in enumerate(sync))
    reconstructed=parsed.work/'sequence.reconstructed.txt'
    reconstructed.parent.mkdir(parents=True,exist_ok=True)
    reconstructed.write_text(lines,encoding='utf-8',newline='\n')
    reconstructed_sha=digest(lines.encode())
    assert reconstructed_sha==receipt['reconstructed_full_sequence_sha256']
    statuses['sequence.txt']=dict(classification='RECONSTRUCTED_NOT_ORIGINAL',original_file_found=False,
        reconstructed_sha256=reconstructed_sha,historical_original_sha256=None,frames=len(sync),
        reconstructed_path=str(reconstructed),
        matching_named_candidates=[r['location'] for r in rows if r['dependency']=='sequence.txt'],
        verification_basis='Frozen ordered sync rows and historical path-generation procedure only')
    environment=load(OUT/'server_environment.json')
    commands=environment['commands']
    environment.update(historical_sdk_revision=history['sdk_revision'],historical_native_opencv='4.5.4',
        current_native_opencv_matches_historical=commands['native_opencv_version'].get('stdout','').strip()=='4.5.4',
        historical_compiler_exact_version=None,historical_elf_linkage=None,
        historical_cxx_abi_verification='UNAVAILABLE_NO_MATCHED_ORIGINAL_ELF',
        original_sdk_local_modifications_recovered=False,
        historical_camera_settings=history['settings'],
        original_rgbd_depth_scale_m_per_uint16=.001,original_orb_feature_count=1000,
        frozen_build_recipe=dict(sha256=digest(build_data),historical_sha256=expected_build_hash,
                                 content=build_data.decode(),reexecuted=False))
    dump(OUT/'stage32r_build_environment.json',environment)
    dump(OUT/'stage32r_hash_verification.json',dict(parent_commit=PARENT,dependencies=statuses))
    missing=[dict(dependency=name,expected_sha256=EXPECTED[name],
                  reason='No accessible candidate matched the historical identity',
                  upstream_substitution_permitted=False) for name in CORE if not statuses[name]['original_identity_verified']]
    dump(OUT/'stage32r_missing_dependencies.json',dict(critical_missing=missing,
         minimum_additional_files=missing,
         pre_replay_sequence_requirement='Original sequence.txt or an independently recorded historical checksum is still needed to authenticate sequence-file identity.',
         known_available=['hash-verified vocabulary','four hash-verified SDK source files',
                          'frozen camera settings','frozen native build recipe','original RGB/depth bags and sync rows'],
         access_limits=environment['snapshot_and_backup_access']))
    gate=dict(status='BLOCKED_ORIGINAL_RUNTIME_NOT_RECOVERED',exact_reproduction=False,
              original_sdk_revision=history['sdk_revision'],
              original_modified_system_sources_verified=False,baseline_executable_verified=False,baseline_library_verified=False,
              vocabulary_verified=statuses['ORBvoc.txt']['original_identity_verified'],
              camera_settings_verified=True,original_sequence_file_verified=False,
              current_opencv_matches_recorded=environment['current_native_opencv_matches_historical'],
              historical_compiler_abi_and_linkage_verified=False,
              ready_for_instrumented_build=False,instrumented_sdk_copy_created=False,
              instrumented_executable_built=False,full_replay_started=False,
              algorithms_or_parameters_changed=False)
    dump(OUT/'stage32r_reproduction_readiness.json',gate)
    fields=['machine','dependency','location','origin','classification','bytes','sha256','expected_sha256',
            'lf_sha256','normalization_can_match_historical','recovered_copy','note']
    with (OUT/'stage32r_dependency_inventory.csv').open('w',newline='',encoding='utf-8') as stream:
        writer=csv.DictWriter(stream,fields,extrasaction='ignore',lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)
        for dep,info in statuses.items():
            writer.writerow(dict(machine='summary',dependency=dep,classification=info['classification'],
                                 sha256=info.get('sha256'),expected_sha256=info.get('expected_sha256'),
                                 note=info.get('verification_basis','Required historical dependency identity')))
    archive_summary={}
    git_summary={}
    for machine in ('server','local'):
        directory=OUT/(machine+'_search')
        archive=load(directory/'archive_search.json')
        git=load(directory/'git_search.json')
        archive_summary[machine]=dict(total=len(archive),inspected=sum(r['status']=='INSPECTED' for r in archive),
                                     limitations=[r for r in archive if r['status']!='INSPECTED'])
        git_summary[machine]=dict(total_entries=len(git),repos=[dict(repository=r.get('repository'),head=r.get('head'),
                                                   original_bundle=r.get('original_bundle'),status=r.get('status')) for r in git])
    compact_coverage={}
    for machine,value in coverage.items():
        compact_coverage[machine]={k:v for k,v in value.items() if k not in ('filesystem_errors','skipped_windows_junctions')}
        for key in ('filesystem_errors','skipped_windows_junctions'):
            compact_coverage[machine][key+'_count']=len(value.get(key,[]))
            compact_coverage[machine][key+'_examples']=value.get(key,[])[:20]
        compact_coverage[machine]['complete_evidence']=machine+'_search/search_coverage.json.gz'
    dump(OUT/'search_summary.json',dict(coverage=compact_coverage,archives=archive_summary,git=git_summary))
    protected=[
        'outputs/cumulti_v1/24_rgbd_full_v1/robot3/camera_poses.csv',
        'outputs/cumulti_v1/24d_rgbd_pose_graph/pose_graph.json',
        'outputs/cumulti_v1/26_rgbd_reconstruction/registration_comparison.csv',
        'outputs/cumulti_v1/27_continuous_rgbd/epoch_display_transforms.json',
        'outputs/cumulti_v1/28_multisensor/gravity_alignment.json',
        'outputs/cumulti_v1/29_spatial_audit/stage29_epoch_transforms.json',
        'outputs/cumulti_v1/30_rgbd_pose_forensics/stage30_pose_jump_audit.csv',
        'outputs/cumulti_v1/31_epoch9_persistence/preservation_audit.json',
        'outputs/cumulti_v1/32_native_tracking_forensics/stage32_run_provenance.json']
    hashes={path:digest(blob(path).replace(b'\r\n',b'\n')) for path in protected}
    working_hashes={path:digest((ROOT/path).read_bytes().replace(b'\r\n',b'\n')) for path in protected}
    assert hashes==working_hashes
    dump(OUT/'stage32r_preservation_audit.json',dict(status='PASS',baseline_commit=PARENT,
        preserved_artifact_canonical_lf_sha256=hashes,
        working_artifact_byte_sha256={path:digest((ROOT/path).read_bytes()) for path in protected},baseline_binaries_written=False,original_sdk_written=False,
        tracking_or_map_runs_started=False))
    report=['# Stage32R historical runtime restoration','',
            '**BLOCKED_ORIGINAL_RUNTIME_NOT_RECOVERED.** The expanded accessible search recovered no matching original '
            'System.cc/System.h, libORB_SLAM3.so or rgbd_full. Verified inputs and the original camera/build recipe remain available. '
            'Instrumented SDK copying, building and replay did not start.','',
            '## Search coverage','',
            '| Machine | Roots | Directories visited | Files visited | Archives inspected | Git repository/bundle entries |',
            '|---|---|---:|---:|---:|---:|']
    for machine,data in coverage.items():
        report.append(f"| {machine} | {', '.join(data['roots'])} | {data['counts']['directories']:,} | "
                      f"{data['counts']['files']:,} | {archive_summary[machine]['inspected']} / {archive_summary[machine]['total']} | "
                      f"{git_summary[machine]['total_entries']} |")
    report+=['',
             'The server scan includes accessible home/workspace/backup/build/cache/install directories across the guest root. '
             'Virtual /proc, /sys, /dev, /run and OS snap application mounts are excluded from recursive file walking. '
             'Live process executables and ORB library mappings are inspected separately for recoverable deleted files. '
             'Git worktrees, named historical objects, reflog-reachable objects, all object-store blobs at the historical '
             'System/header/library sizes, and discovered Git bundles are checked. Compressed archive members are hashed in memory '
             'without unpacking unrelated files. Additional renamed ELF executables under one megabyte and files at known historical '
             'sizes are checked. The complete runtime search is broader than a filename-only SDK-path check.',
             '',
             'Windows C/D/E roots were searched with junction pruning to avoid repeated traversal of aliases into already available drive trees. '
             'The scanner retains access errors and archive/Git limitations. This is an accessible search under the current accounts.',
             '',
             'Docker socket/storage and /root are inaccessible to cas. Docker reports permission denied. No saved snap snapshots were reported. '
             'No mounted guest snapshot containing the runtime was located. Provider-side snapshots are not exposed through this account. '
             'Archive failures and unsupported formats are listed in search_summary.json and were not counted as inspected.',
             '', '## Required identities','',
             '| Dependency | Status | Original identity verified |',
             '|---|---|---|']
    for dep in ('Tracking.cc','Frame.cc','ORBextractor.cc','Tracking.h','System.cc','System.h',
                'libORB_SLAM3.so','rgbd_full','ORBvoc.txt','camera.yaml','build_rgbd_full.sh','sequence.txt'):
        item=statuses[dep]
        verified=item.get('original_identity_verified',dep in ('camera.yaml','build_rgbd_full.sh'))
        report.append(f"| {dep} | {item['classification']} | {'yes' if verified else 'no'} |")
    report+=['',
             'Source files matching the old receipts and the vocabulary include Stage32 recovered copies and/or their archive members. '
             'Public System variants are available but fail the historical hashes. They are candidates rather than restorations of the modified SDK. '
             'The exact historical hashes and every candidate location are recorded in stage32r_hash_verification.json and the inventory CSV.',
             '',
             'The camera YAML is authenticated by the frozen project Git blob, and the native build script is compared to its historical SHA receipt. '
             'The reconstructed sequence has the same content hash as the prior Stage32 reconstruction. No original sequence-file receipt was found. '
             'Its classification remains RECONSTRUCTED_NOT_ORIGINAL. Input order can be reconstructed from the frozen sync rows, while original sequence identity remains unverified.',
             '', '## Build environment and ABI','',
             'Current native OpenCV matches the recorded 4.5.4. Current compiler version, target triple, libstdc++ ABI macros, '
             'glibc/loader versions and shared-library inventory are recorded. The original build recipe uses C++17, O2, the fixed '
             'preprocessor flags and the ORB/DBoW2/Pangolin rpaths shown in the environment JSON. Original compiler exact version, '
             'actual ELF NEEDED/RUNPATH entries and original C++ ABI cannot be authenticated while the matching executable/library '
             'are missing. Present host compatibility alone does not establish the original runtime environment.',
             '',
             'Camera settings remain 640×400, RGBD.DepthMapFactor 1000, 1000 ORB features, scale factor 1.2, eight levels and FAST 20/7. '
             'No package installation, build-parameter change or tracking invocation was performed.',
             '', '## Unreproduced modified System behavior','',
             'The original TrackRGBD caller requires the System constructor, TrackRGBD gateway, tracking-state/MapPoint accessors '
             'and Shutdown. Without the original modified System implementation/header, their exact initialization, reset/map-control, '
             'entry/exit behavior and class ABI cannot be verified. The patch may concern export, tracking-relevant behavior, or both. '
             'The available hashes identify a difference but reveal no modification semantics. Inferring an export-only change from the '
             'public code would be unsupported.',
             '', '## Minimum files needed to unblock','']
    for item in missing:
        report.append(f"- Original {item['dependency']}, SHA256 {item['expected_sha256']}.")
    report+=['',
             'An authenticated original SDK backup or its original patch is preferable because it retains the surrounding build/dependency context. '
             'An original sequence.txt or independently recorded original checksum is additionally needed before authenticating the exact sequence file. '
             'An administrator can inspect the inaccessible backup/container/provider snapshot locations if such backups exist. '
             'No further camera dataset download is required.',
             '', '## Recovery gate and preservation','',
             'Exact reproduction and readiness for an instrumented build are false. The user stop condition is reached because the '
             'modified sources and original executable were not recovered. Stage24–Stage32 scientific artifacts were hash-checked and preserved. '
             'This checkpoint records restoration evidence and blockers, and does not claim completion of native tracking forensics.',
             '', '## Reproduce','',
             'Run stage32r_search.py on the server root and Windows drive roots with separate output/work directories, then run '
             'stage32r_environment.py on the server. Place the server/local search JSONs and server_environment.json under the Stage32R '
             'output directory. stage32r_finalize.py --work <separate scratch directory> merges the evidence and enforces the critical-original '
             'identity gate. Original files and archive members are read-only; matching recovered copies and Git bundle mirrors go into scratch.']
    (OUT/'STAGE32R_RUNTIME_RESTORATION_REPORT.md').write_text('\n'.join(report)+'\n',encoding='utf-8')
    print('STAGE32R CHECKPOINT',gate['status'],'missing',len(missing),flush=True)


if __name__=='__main__':
    main()
