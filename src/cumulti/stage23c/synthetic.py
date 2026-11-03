"""Reproducible anchored Gaussian and Pose3 tests, no new nonlinear optimize."""
from common import *
from scipy import sparse
from scipy.sparse.linalg import splu
def main():
    verify();A=np.array([[10.,0.],[-1.,1.]]);H=A.T@A;C=np.linalg.inv(H);a=np.array([[0.,10.]]);H1=H+a.T@a;C1=np.linalg.inv(H1)
    s0,l0=np.linalg.slogdet(H);s1,l1=np.linalg.slogdet(H1);assert s0>0 and s1>0 and l1>=l0 and np.linalg.eigvalsh(C-C1).min()>-1e-12
    truth=np.array([0.,1.]);mean0=np.linalg.solve(H,A.T@(A@truth));mean1=np.linalg.solve(H1,A.T@(A@truth)+a.T[:,0]*50.)
    assert np.linalg.norm(mean1-truth)>np.linalg.norm(mean0-truth)+1 and np.trace(C1)<np.trace(C)
    rng=np.random.default_rng(2303);errors=rng.normal(size=(4096,2))@A@C;MSE=float(np.mean((errors**2).sum(1)));assert abs(MSE/np.trace(C)-1)<.15
    unanchored=np.array([[1.,-1.],[-1.,1.]]);assert np.linalg.matrix_rank(unanchored)==1 and np.linalg.norm(unanchored@np.ones(2))==0
    t=gtsam.Pose3.Logmap(gtsam.Pose3(gtsam.Rot3.Rz(.2),np.zeros(3)));np.testing.assert_allclose(t,[0,0,.2,0,0,0],atol=1e-12)
    np.testing.assert_allclose(gtsam.Pose3.Logmap(gtsam.Pose3(gtsam.Rot3(),np.array([1.,2.,3.]))),[0,0,0,1,2,3],atol=1e-12)
    q=gtsam.Pose3(gtsam.Rot3.RzRyRx(.1,-.2,.3),np.array([1.,2.,3.]));c=gtsam.Pose3(gtsam.Rot3.RzRyRx(-.2,.1,.7),np.array([4.,-1.,2.]));z=gtsam.Pose3(gtsam.Rot3.Rz(.2),np.array([1.,1.,1.]));hq=np.zeros((6,6),order='F');hc=hq.copy(order='F');between=q.between(c,hq,hc);E=z.inverse().compose(between);J=gtsam.Pose3.LogmapDerivative(E)@np.hstack([hq,hc]);fd=np.empty((6,12));eps=1e-6
    for i in range(12):
        d=np.zeros(6);d[i%6]=eps;qp,cp=q,c;qm,cm=q,c
        if i<6:qp=q.retract(d);qm=q.retract(-d)
        else:cp=c.retract(d);cm=c.retract(-d)
        fd[:,i]=(gtsam.Pose3.Logmap(z.inverse().compose(qp.between(cp)))-gtsam.Pose3.Logmap(z.inverse().compose(qm.between(cm))))/(2*eps)
    jerr=float(abs(J-fd).max());assert jerr<1e-5
    g=gtsam.NonlinearFactorGraph();v=gtsam.Values();v.insert(1,q);v.insert(2,c);g.add(gtsam.PriorFactorPose3(1,q,gtsam.noiseModel.Isotropic.Sigma(6,.1)));g.add(gtsam.BetweenFactorPose3(1,2,q.between(c),gtsam.noiseModel.Isotropic.Sigma(6,1.)))
    marginal=gtsam.Marginals(g,v);C12=Cpair(marginal,[1,2]);assert np.linalg.norm(C12[:6,6:])>0 and np.isfinite(C12).all()
    hq=np.zeros((6,6),order='F');hc=hq.copy(order='F');q.between(c,hq,hc);jr=np.hstack([hq,hc]);diag=C12.copy();diag[:6,6:]=0;diag[6:,:6]=0
    rel=jr@C12@jr.T;incorrect=jr@diag@jr.T;assert np.linalg.norm(rel-incorrect)>1e-5
    S=sparse.diags([-np.ones(99),np.r_[11.,np.full(98,2.),1.],-np.ones(99)],[-1,0,1],format='csc');x=splu(S).solve(np.ones(100));sparse_err=float(np.linalg.norm(S@x-np.ones(100)));assert sparse_err<1e-8
    dump('synthetic_information_sanity.json',dict(status='PASS',fixed_logdet_before=float(l0),fixed_logdet_after=float(l1),fixed_PSD_information_gain_nats=float(.5*(l1-l0)),covariance_trace_before=float(np.trace(C)),covariance_trace_after=float(np.trace(C1)),unbiased_expected_squared_error_trace=float(np.trace(C)),unbiased_monte_carlo_MSE=MSE,unbiased_realized_error=float(np.linalg.norm(mean0-truth)),biased_realized_error=float(np.linalg.norm(mean1-truth)),gauge_nullity=1,unanchored_global_covariance='UNDEFINED_NO_FAKE_INVERSE',Pose3_tangent_order='rx ry rz tx ty tz',Pose3_exact_Logmap_Jacobian_max_error=jerr,joint_cross_block_norm=float(np.linalg.norm(C12[:6,6:])),relative_covariance_error_ignoring_cross=float(np.linalg.norm(rel-incorrect)),sparse_solve_residual=sparse_err,anchored_toy_condition_number=float(np.linalg.cond(H)),full_dense_global_graph_matrix=False,scope='fixed-linearization Gaussian identity, NOT nonlinear robust global monotonicity or real measurement bias diagnosis'))
if __name__=='__main__':main()
