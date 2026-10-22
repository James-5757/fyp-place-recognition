"""Explicit Stage21 schema/catalog provenance; never substitute XYZ sizes for Stage22A XYZI."""
from common import *
import stage21_wire as wire


def main():
    verify_lock()
    schema=ROOT/'outputs/cumulti_v1/21_cost_baseline/message_schema.json'
    catalog=ROOT/'outputs/cumulti_v1/21_cost_baseline/cloud_wire_catalog.csv'
    assert read(schema)==wire.schema()
    inputs=read(OUT/'stage22b_input_manifest.json');provenance={}
    for path in [schema,catalog]:
        rel=str(path.relative_to(ROOT))
        digest=sha(path);initial=inputs['protected_worktree'].get(rel)
        assert initial==digest,'Stage21 audit input not frozen in protected baseline'
        blob=subprocess.check_output(['git','show',BASE+':'+rel],cwd=ROOT)
        assert hashlib.sha256(blob).hexdigest()==digest
        provenance[rel]=dict(sha256=digest,source_commit=subprocess.check_output(['git','log','-1','--format=%H',BASE,'--',rel],cwd=ROOT,text=True).strip())
    data=pd.read_csv(catalog)
    assert (data.framing_bytes==48).all() and (data.serialized_total_bytes==data.payload_bytes+48).all()
    raw=pd.read_csv(A/'lidar_input_manifest.csv')
    overlap=data.merge(raw,left_on=['robot_id','keyframe_id'],right_on=['robot','keyframe'])
    assert len(overlap)>=3946 and (overlap.source_sha256==overlap.sha256).all()
    messages=pd.read_csv(A/'online_protocol_message_trace.csv')
    assert set(messages[messages.message_type=='POINT_CLOUD_RESPONSE'].serialized_total_bytes)=={1048632}
    write_json(OUT/'stage21_schema_catalog_audit.json',dict(status='PASS',inputs=provenance,
        policy_freeze_provenance='Both schema and catalog hashes already in original protected_worktree snapshot; this is an explicit conformance explanation, not a new policy input',
        raw_overlap_hash_checks=len(overlap),Stage21_catalog_mode='XYZ (3 fields);786488 encoded bytes/cloud',
        Stage22A_and_22B_primary_mode='Unchanged lossless XYZI (4 fields);1048632 encoded bytes/cloud, plus49-byte request',
        format='same Stage21 variable-field-count binary container and48-byte framing',
        transfer_feature='All primary clouds use the same fixed65536x4 float32 format;1048681 bytes/miss is a constant workload model. No variable future per-frame cloud size is used to rank.',
        historical_XYZ_sizes_used_for_new_budget=False,transport_changed=False))

if __name__=='__main__':main()
