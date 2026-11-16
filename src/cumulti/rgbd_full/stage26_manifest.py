"""Audit frozen Stage24E frame identities without modifying source data."""
import argparse
import csv
import json
from pathlib import Path


def read_csv(path):
    with path.open(newline='', encoding='utf-8') as handle:
        return list(csv.DictReader(handle))


def write_csv(path, rows):
    with path.open('w', newline='', encoding='utf-8') as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]), lineterminator='\n')
        writer.writeheader()
        writer.writerows(rows)


def run(root, cache_roots):
    outputs = root / 'outputs/cumulti_v1'
    source = outputs / '24_rgbd_full_v1'
    evidence = outputs / '24e_rgbd_feasibility'
    destination = outputs / '26_rgbd_reconstruction'
    destination.mkdir(parents=True, exist_ok=True)
    diagnostics = json.loads((evidence / 'dense_submap_diagnostics.json').read_text())
    boundaries = {row['boundary']: row for row in read_csv(evidence / 'epoch_boundary_audit.csv')}
    tables = {
        robot: tuple({int(row['index']): row for row in read_csv(source / robot / name)}
                     for name in ('camera_poses.csv', 'sync_pairs.csv'))
        for robot in ('robot1', 'robot3')
    }
    rows, missing = [], {}
    for entry in diagnostics:
        boundary = entry['boundary']
        meta = boundaries[boundary]
        robot = meta['robot']
        poses, sync = tables[robot]
        for side, epoch_key, anchor_key in (
            (0, 'previous_epoch', 'previous_last_valid'), (1, 'next_epoch', 'next_first_valid')
        ):
            fit = set(entry['fit_frames'][side])
            holdout = set(entry['holdout_frames'][side])
            anchor = int(meta[anchor_key])
            # The anchor defines a camera coordinate system and also has a frozen pose.
            for index in sorted(fit | holdout | {anchor}):
                pose, pair = poses[index], sync[index]
                assert pose['valid_pose'] == '1' and pose['map_epoch'] == meta[epoch_key]
                for field in ('rgb_bag_ns', 'rgb_header_ns', 'relative_ns'):
                    assert pose[field] == pair[field], (robot, index, field)
                assert int(pose['rgb_bag_ns']) - int(sync[0]['rgb_bag_ns']) == int(pose['relative_ns'])
                needs_assets = index in fit or index in holdout
                rgb, depth = Path(pair['rgb']), source.relative_to(root) / robot / 'depth' / f'{index:06d}.png'
                if needs_assets:
                    assert pair['paired'] == 'True'
                    assert abs(int(pair['rgb_bag_ns']) - int(pair['depth_bag_ns'])) <= 25_000_000
                row = dict(boundary=boundary, cohort='stage24e_pass' if entry['geometry_pass'] else 'failure_control',
                           robot=robot, epoch=meta[epoch_key], index=index, anchor=index == anchor,
                           fit=index in fit, holdout=index in holdout, reused=index in fit and index in holdout,
                           rgb_message_id=pair['rgb_message_id'], rgb_bag_ns=pair['rgb_bag_ns'],
                           rgb_header_ns=pair['rgb_header_ns'], depth_index=pair['depth_index'],
                           depth_message_id=pair['depth_message_id'], depth_bag_ns=pair['depth_bag_ns'],
                           depth_header_ns=pair['depth_header_ns'], frozen_pair=pair['paired'],
                           frozen_timestamps_verified=True, recovered_message_timestamps_verified=False)
                for kind, relative in (('rgb', rgb), ('depth', depth)):
                    found = needs_assets and any((cache / relative).is_file() for cache in cache_roots)
                    row[f'{kind}_path'] = relative.as_posix()
                    row[f'{kind}_availability'] = ('PRESENT_UNVERIFIED' if found else 'MISSING') if needs_assets else 'POSE_ONLY'
                    if needs_assets and not found:
                        key = (robot,index,kind)
                        item = missing.setdefault(key,dict(robot=robot,index=index,kind=kind,
                            expected_path=relative.as_posix(),message_id=pair[f'{kind}_message_id'],
                            bag_ns=pair[f'{kind}_bag_ns'],header_ns=pair[f'{kind}_header_ns'],
                            recovery_sources='set of matching raw ZIPs available' if robot=='robot3' else 'restore original RGB and depth ZIPs or verified cache',
                            cases=set()))
                        item['cases'].add(boundary)
                rows.append(row)
    for item in missing.values():
        item['cases'] = '|'.join(sorted(item['cases']))
    write_csv(destination / 'stage26_frame_manifest.csv', rows)
    write_csv(destination / 'stage26_missing_frames.csv', list(missing.values()))
    return rows, missing


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--cache-root',type=Path,action='append',default=[])
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[3]
    manifest, absent = run(root,[root,*args.cache_root])
    print(f'{len(manifest)} boundary memberships, {len(absent)} unique missing assets')
