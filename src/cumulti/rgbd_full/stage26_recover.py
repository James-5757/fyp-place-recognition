"""Recover only frozen Stage26 frame IDs from CRC-validated SQLite bag copies."""
import argparse
import csv
import json
import sqlite3
import zipfile
from contextlib import closing
from pathlib import Path

import cv2
import numpy as np
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

from common import OUT, RAW, align_depth, dump, sha
from stage26_manifest import read_csv, write_csv

DEST = OUT.parent / '26_rgbd_reconstruction'


def recover(robot, kind, scratch):
    cv2.setNumThreads(1)
    records = json.loads((OUT / 'dataset_manifest.json').read_text())['robots'][robot]
    archive = records[kind.upper()]['archive']
    path = RAW / robot / Path(archive['archive']).name
    with zipfile.ZipFile(path) as container:
        info = container.getinfo(archive['member'])
        assert (info.file_size, f'{info.CRC:08x}', path.stat().st_size) == (
            archive['db_bytes'], archive['CRC32'], archive['archive_bytes'])
    db = scratch / Path(info.filename).name
    # A final DB name is published by the runner only after unzip exits successfully.
    assert db.stat().st_size == info.file_size
    manifest = read_csv(DEST / 'stage26_required_assets.csv')
    selected = [row for row in manifest if row['robot'] == robot and row['kind'] == kind]
    rgb = records['RGB']['calibration']
    intrinsics = np.array(rgb['K']).reshape(3, 3)
    intrinsics[:2] *= .5
    distortion = np.array(rgb['D'])
    extrinsic = records['DEPTH']['extra']['T_color_from_depth']
    rotation, translation = np.array(extrinsic['R']), np.array(extrinsic['t_m'])
    store = get_typestore(Stores.ROS2_HUMBLE)
    folder = DEST / 'frames' / robot / kind
    folder.mkdir(parents=True, exist_ok=True)
    receipts = []
    with closing(sqlite3.connect(f'{db.resolve().as_uri()}?mode=ro&immutable=1', uri=True)) as connection:
        for typename, encoding, definition in connection.execute(
            'select topic_type,encoding,encoded_message_definition from message_definitions'
        ):
            if encoding == 'ros2msg':
                store.register(get_types_from_msg(definition, typename))
        for row in selected:
            ns, topic, typename, payload = connection.execute(
                'select m.timestamp,t.name,t.type,m.data from messages m join topics t on t.id=m.topic_id where m.id=?',
                (int(row['message_id']),),
            ).fetchone()
            assert ns == int(row['bag_ns']) and topic == records[kind.upper()]['topic'], row
            msg = store.deserialize_cdr(payload, typename)
            stamp = msg.header.stamp
            assert stamp.sec * 10**9 + stamp.nanosec == int(row['header_ns']), row
            assert msg.header.frame_id == records[kind.upper()]['calibration']['frame_id'], row
            target = folder / Path(row['expected_path']).name
            if kind == 'rgb':
                assert msg.encoding in ('rgb8', 'bgr8')
                pixels = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
                image = pixels[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
                if msg.encoding == 'rgb8':
                    image = image[:, :, ::-1]
                image = cv2.resize(image, (640, 400), interpolation=cv2.INTER_AREA)
                options = [cv2.IMWRITE_JPEG_QUALITY, 85]
            else:
                fields = {field.name: field for field in msg.fields}
                keys = ('x', 'y', 'z')
                assert all(fields[key].datatype == 7 and fields[key].count == 1 for key in keys)
                endian = '>' if msg.is_bigendian else '<'
                dtype = np.dtype(dict(names=keys, formats=[endian + 'f4'] * 3,
                                      offsets=[fields[key].offset for key in keys], itemsize=msg.point_step))
                points = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                                    strides=(msg.row_step, msg.point_step))
                xyz = np.stack([points[key].ravel() for key in keys], axis=1)
                image = np.rint(align_depth(xyz, rotation, translation, intrinsics, distortion) * 1000).astype(np.uint16)
                assert image.shape == (400, 640) and np.any(image)
                options = []
            temporary = target.with_suffix(f'.pending{target.suffix}')
            assert cv2.imwrite(str(temporary), image, options)
            temporary.replace(target)
            receipts.append(dict(robot=robot, kind=kind, index=row['index'], message_id=row['message_id'],
                                 bag_ns=str(ns), header_ns=row['header_ns'], topic=topic,
                                 archive_crc=archive['CRC32'], asset=target.relative_to(DEST).as_posix(),
                                 sha256=sha(target)))
    assert receipts
    write_csv(DEST / f'{robot}_{kind}_receipts.csv', receipts)


def gate():
    fields = ('robot', 'kind', 'index', 'message_id', 'bag_ns', 'header_ns')
    expected = {tuple(row[k] for k in fields) for row in read_csv(DEST / 'stage26_required_assets.csv')}
    records = [
        row for robot in ('robot1', 'robot3') for kind in ('rgb', 'depth')
        for row in read_csv(DEST / f'{robot}_{kind}_receipts.csv')
    ]
    assert len(records) == len(expected)
    assert {tuple(row[k] for k in fields) for row in records} == expected
    calibration = json.loads((OUT / 'dataset_manifest.json').read_text())['robots']
    frozen_coverage = {
        robot: {row['index']: float(row['valid_depth_fraction'])
                for row in read_csv(OUT / robot / 'depth_coverage.csv')}
        for robot in ('robot1','robot3')
    }
    failures = []
    for row in records:
        path = DEST / row['asset']
        rgb = row['kind']=='rgb'
        identity = calibration[row['robot']][row['kind'].upper()]
        reasons = []
        if row['topic']!=identity['topic'] or row['archive_crc']!=identity['archive']['CRC32']:
            reasons.append('topic or archive identity mismatch')
        if not path.is_file():
            reasons.append('file absent')
        else:
            if sha(path)!=row['sha256']:
                reasons.append('SHA256 mismatch')
            signature = path.read_bytes()[:8]
            if not (signature.startswith(b'\xff\xd8\xff') if rgb else signature==b'\x89PNG\r\n\x1a\n'):
                reasons.append('encoded format mismatch')
            image = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
            if image is None or (image.shape,image.dtype)!=(((400,640,3),np.uint8) if rgb else ((400,640),np.uint16)):
                reasons.append('decoded dimensions or dtype mismatch')
            elif not rgb and abs(float((image>0).mean())-frozen_coverage[row['robot']][row['index']])>1e-12:
                reasons.append('depth coverage differs from frozen Stage24 projection')
        if reasons:
            failures.append(dict(asset=row['asset'], reasons=reasons))
    dump(DEST / 'stage26_gate1.json', dict(status='PASS' if not failures else 'FAIL',
         verified_assets=len(records)-len(failures), expected_assets=len(expected),
         verified_rgbd_pairs=len(records)//2 if not failures else None,
         failures=failures, checks=['original message ID, bag timestamp, header timestamp, topic and optical frame at export',
                                'archive identity, encoded format, dimensionality, pixel dtype, SHA256',
                                'per-frame depth coverage equals frozen Stage24 projection'],
         scope='13 Stage24E cases, 11 failure controls and four non-adjacent pairs'))
    assert not failures, failures


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--scratch', type=Path, required=True)
    parser.add_argument('--robot', choices=['robot1', 'robot3'])
    parser.add_argument('--kind', choices=['rgb', 'depth'])
    args = parser.parse_args()
    if args.robot and args.kind:
        recover(args.robot, args.kind, args.scratch)
    elif not args.robot and not args.kind:
        gate()
    else:
        parser.error('provide both robot and kind, or neither for the gate')
