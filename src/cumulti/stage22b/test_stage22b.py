"""Synthetic causal/prefix/cache tests; no raw data or GT reads."""
import unittest
from replay import Cache,neighborhood,run
from common import *


class SyntheticData:
    def __init__(self,accept=True):
        self.events=[dict(event_index=0,robot_id='robot1',keyframe_id=150,normalized_ns=0),dict(event_index=1,robot_id='robot3',keyframe_id=234,normalized_ns=0)]
        self.groups={1:[dict(query_robot='robot3',candidate_robot='robot1',query_keyframe_id=234,candidate_keyframe_id=150,
            candidate_rank=1,SC_score=.9,current_R1_database_size=1,current_R3_database_size=0)]}
        self.fixed_events={0:5081,1:5133}
        self.fixed=pd.DataFrame([dict(event_index=0,message_type='FULL_SC_DESCRIPTOR_UPLOAD',serialized_total_bytes=4848),
            dict(event_index=1,message_type='FULL_SC_DESCRIPTOR_UPLOAD',serialized_total_bytes=4848),
            dict(event_index=0,message_type='ODOMETRY_UPDATE',serialized_total_bytes=233),
            dict(event_index=1,message_type='ODOMETRY_UPDATE',serialized_total_bytes=233)])
        self.cloud_size={('robot1',150):1048681,('robot3',234):1048681}
        self.policy=dict(normalization=dict(SC_q05=0,SC_q95=1,margin_scale=.1,cpu_scale_ms=1,transfer_scale_bytes=2097362),
            P5_weights=dict(confidence=1,margin=.1,novelty=.1,redundancy=.1,cpu=.05,transfer=.05),
            predicted_cpu_ms={'robot3:1':1},P3_top1_threshold=.5)
        self.budget=dict(reference_calls=15267,reference_application_bytes=6107977604)
        self.lookups=0;self.accept=accept
    def outcome(self,event,rank):
        self.lookups+=1
        assert (event,rank)==(1,1)
        return dict(event_index=1,candidate_rank=1,call_cpu_ms=1.,call_wall_ms=1.,geometrically_accepted=self.accept,
            canonical_transform=json.dumps(np.eye(4).tolist()))


class Tests(unittest.TestCase):
    def test_cache_lru(self):
        cache=Cache(2)
        self.assertFalse(cache.access('a'));self.assertFalse(cache.access('b'))
        self.assertTrue(cache.access('a'));self.assertFalse(cache.access('c'))
        self.assertEqual(cache.misses(['a','b','c']),['b','c'])

    def test_closed_neighborhood(self):
        self.assertTrue(neighborhood(3,5,[(0,0)]));self.assertFalse(neighborhood(4,5,[(0,0)]))

    def test_request_plan_counts_candidate_evicted_by_query(self):
        cache=Cache(2);cache.access('candidate');cache.access('other')
        self.assertEqual(cache.misses(['new_query','candidate']),['new_query','candidate'])
        self.assertEqual(list(cache.items),['candidate','other']) # planning does not mutate residency

    def test_decision_before_outcome(self):
        data=SyntheticData()
        result=run(data,'P5_TEST',threshold=10,end=2)[0]
        self.assertEqual(data.lookups,0);self.assertEqual(result['attempts'],0)

    def test_prefix_early_stop(self):
        data=SyntheticData()
        data.groups[1].append(dict(data.groups[1][0],candidate_rank=2,SC_score=.8))
        data.policy['predicted_cpu_ms']['robot3:2']=1
        result=run(data,'P0_FULL_TOP3',end=2)[0]
        self.assertEqual(data.lookups,1);self.assertEqual(result['admitted'],1)

    def test_zero_budget_no_receipt_lookup(self):
        data=SyntheticData()
        result=run(data,'P5_TEST',compute=0,end=2)[0]
        self.assertEqual(data.lookups,0);self.assertEqual(result['attempts'],0)

    def test_future_candidate_rejected(self):
        data=SyntheticData();data.groups[1][0]['candidate_keyframe_id']=151
        with self.assertRaisesRegex(AssertionError,'Future'):run(data,'P0_FULL_TOP3',end=2)

    def test_inverse_convention(self):
        from scipy.spatial.transform import Rotation
        T=np.eye(4);T[:3,:3]=Rotation.from_euler('z',.4).as_matrix();T[:3,3]=[1,2,3]
        np.testing.assert_allclose(inv(T)@T,np.eye(4),atol=1e-12)

    def test_candidate_only_actual_input(self):
        source=(Path(__file__).parent/'actual.py').read_text()
        self.assertIn('{key:row[key] for key in candidate_columns}',source)


if __name__=='__main__':
    suite=unittest.main(exit=False)
    write_json(OUT/'synthetic_unit_tests.json',dict(status='PASS' if suite.result.wasSuccessful() else 'FAIL',
        tests=suite.result.testsRun,source_sha256=sha(__file__),data_scope='synthetic, no GT'))
    raise SystemExit(0 if suite.result.wasSuccessful() else 1)
