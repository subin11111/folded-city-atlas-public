from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
from scipy.sparse import csr_matrix

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))
from audit_constraint_conflicts import graph_bridges,exclusion_step


def test_graph_bridge_screen_respects_parallel_measurements_and_disconnected_nodes():
    def e(key,a,b):return SimpleNamespace(id=key,source=a,target=b)
    edges=[e('ab1','a','b'),e('ab2','a','b'),e('bc','b','c')]
    assert graph_bridges(['a','b','c','isolated'],edges)=={'bc'}


def test_linear_exclusion_agrees_with_direct_remaining_factor_solution():
    rng=np.random.default_rng(42)
    j=rng.normal(size=(18,7));r=rng.normal(size=18)
    h=j.T@j+np.eye(7)*.01;g=j.T@r
    solve=lambda rhs:np.linalg.solve(h,rhs)
    base=solve(-g);a=csr_matrix(j[:6]);removed_r=r[:6]
    correction,eigen=exclusion_step(a,removed_r,solve,base)
    direct=np.linalg.solve(h-j[:6].T@j[:6],-g+j[:6].T@removed_r)
    assert eigen>0
    assert np.allclose(base+correction,direct,atol=1e-10)
