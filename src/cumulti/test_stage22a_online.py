"""Synthetic causal, direction, descriptor and durable-ledger tests; no GT."""
import tempfile,unittest
from unittest.mock import patch
import gtsam
from scipy.spatial.transform import Rotation
import stage22a_common as c
import run_stage22a_verification as v
from stage22a_common import *

class Tests(unittest.TestCase):
    def setUp(self):self.d=np.random.default_rng(22).random((20,60),dtype=np.float32)
    def test_empty_before_and_reverse_after(self):
        r1=Database();r3=Database();self.assertEqual(r3.query(self.d)[0],[])
        r1.insert(150,self.d);self.assertEqual(r1.query(self.d)[0][0]['candidate_keyframe_id'],150)
    def test_small_and_large_databases(self):
        db=Database()
        for k in range(1002):
            db.insert(k,self.d+np.float32(k*.001))
            if k in [0,1,2,1001]:
                rows,_=db.query(self.d);self.assertEqual(len(rows),min(3,k+1));self.assertTrue(all(x['candidate_keyframe_id']<=k for x in rows))
    def test_schedule_ties(self):
        e,_=schedule({'robot3':[(234,9),(235,19)],'robot1':[(150,100),(151,110)]},[])
        self.assertEqual([(x['robot_id'],x['keyframe_id']) for x in e],[('robot1',150),('robot3',234),('robot1',151),('robot3',235)])
    def test_determinism(self):
        db=Database()
        for k in range(10):db.insert(k,self.d)
        self.assertEqual(db.query(self.d)[0],db.query(self.d)[0])
    def test_descriptor_equivalence(self):
        p=np.random.default_rng(7).normal(size=(5000,4)).astype(np.float32);a=s2.descriptor(p)
        self.assertTrue(np.array_equal(a,s2.descriptor(p.copy())))
        db=Database();db.insert(1,a);self.assertTrue(np.array_equal(db.keys[0],s8.ring_key(a[None])[0]))
    def test_reverse_inverse_factor(self):
        Z=np.eye(4);Z[:3,:3]=Rotation.from_euler('xyz',[.3,-.2,.8]).as_matrix();Z[:3,3]=[1,-2,3]
        q,k,T=canonical('robot3',252,283,inv(Z));self.assertEqual((q,k),(283,252));np.testing.assert_allclose(T,Z,atol=1e-14)
        X=np.eye(4);X[:3,3]=[7,2,-1];Y=X@Z
        factor=gtsam.BetweenFactorPose3(1,3,gtsam.Pose3(T),gtsam.noiseModel.Unit.Create(6));values=gtsam.Values();values.insert(1,gtsam.Pose3(X));values.insert(3,gtsam.Pose3(Y))
        self.assertLess(factor.error(values),1e-25)
        np.testing.assert_allclose(T@inv(T),np.eye(4),atol=1e-14)
    def test_causal_duplicate(self):
        accepted=[(100,200)];self.assertTrue(duplicate(103,205,accepted));self.assertFalse(duplicate(104,205,accepted));self.assertEqual(accepted,[(100,200)])
    def test_resume_exact_once_and_uncertain_block(self):
        row=dict(event_index=1,candidate_rank=1,query_robot='robot1',query_keyframe_id=150,candidate_robot='robot3',candidate_keyframe_id=234,best_shift=0)
        with tempfile.TemporaryDirectory() as t,patch.object(v,'OUT',Path(t)),patch.object(c,'OUT',Path(t)):
            p=Path(t)/'attempts';p.mkdir();raw=p/'e0001_r1.csv';raw.write_text('synthetic fixture\n')
            c.dump('attempts/e0001_r1.json',dict(candidate_identity=list(row.values()),measurement=dict(result_csv_sha256=sha(raw),call_wall_ms=3)))
            with patch.object(v,'actual_call',side_effect=AssertionError('must not rerun')):
                m,used=v.attempt(None,row,'full',1.,7200);self.assertEqual(used,1.);self.assertEqual(m['call_wall_ms'],3)
            row['event_index']=2;c.dump('attempts/e0002_r1.intent.json',dict(status='INTENT'))
            with self.assertRaises(AssertionError):v.attempt(None,row,'full',1.,7200)
    def test_exact_once_logical_factors(self):
        e,l=schedule({'robot1':[(150,0),(151,10)],'robot3':[(234,0),(235,10)]},[dict(frozen_loop_id=999,query_robot='robot3',query_keyframe_id=235,candidate_robot='robot1',candidate_keyframe_id=150)])
        from stage20_schedule import replay_registry
        rows=replay_registry(e,l);self.assertEqual(rows[-1]['loops'],1);self.assertEqual(rows[-1]['odometry'],2)
    def test_runtime_oracle_guard(self):
        # Hook is process-lifetime; install only in a child process.
        import subprocess
        for target in ['/tmp/gt_utm.csv','/tmp/ordered_rank1_clean_loops.csv','/tmp/09_sc_gicp_integration/candidate_interface.csv']:
            code="from stage22a_common import online_guard; online_guard(); open("+repr(target)+")"
            result=subprocess.run([sys.executable,'-c',code],cwd=Path(__file__).parent,capture_output=True,text=True)
            self.assertNotEqual(result.returncode,0);self.assertIn('oracle access forbidden',result.stderr)

if __name__=='__main__':
    result=unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Tests))
    dump('synthetic_unit_tests.json',dict(status='PASS' if result.wasSuccessful() else 'FAIL',tests=result.testsRun,failures=len(result.failures),errors=len(result.errors)))
    dump('transformation_direction_audit.json',dict(status='PASS' if result.wasSuccessful() else 'FAIL',forward='T_R1_from_R3',reverse='inverse(T_R3_from_R1), full matrix inverse',factor='X_R1.between(X_R3)=T_R1_from_R3',quaternions='xyzw at CSV boundary; matrix internal',synthetic_composition_and_factor_test=True))
    sys.exit(0 if result.wasSuccessful() else 1)
