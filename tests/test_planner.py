import json
from pathlib import Path
import numpy as np
import pytest
from highway_planning.planner import SCENARIOS, N, DT, dynamics_map, plan, Program

DATA=json.loads((Path(__file__).parents[1]/'docs/data/experiment.json').read_text())


def test_exact_constant_acceleration_dynamics():
    times=np.array([0,.25,.5,1,6]);P,V=dynamics_map(times)
    np.testing.assert_allclose(P@np.ones(N),.5*times**2)
    np.testing.assert_allclose(V@np.ones(N),times)


def test_small_qp_known_solution_and_infeasibility():
    qp=Program(2);qp.square(np.array([1.,0]),-2,1);qp.square(np.array([0.,1]),-3,1)
    qp.constrain([1,1],high=2)
    z,_,residual=qp.solve(np.zeros(2),np.ones(2)*10)
    np.testing.assert_allclose(z,[.5,1.5],atol=1e-5)
    assert residual<1e-6
    qp.constrain([1,1],low=3)
    assert qp.solve(np.zeros(2),np.ones(2)*10) is None


def test_coupled_plan_beats_frozen_baseline_on_its_own_objective():
    coupled=plan(SCENARIOS[2],'coupled');short=plan(SCENARIOS[2],'short')
    assert coupled['metrics']['objective'] <= short['metrics']['objective']+1e-4
    assert coupled['metrics']['goal_transition_s'] < short['metrics']['goal_transition_s']-1
    assert coupled['metrics']['speed_rmse_mps'] > short['metrics']['speed_rmse_mps']


@pytest.mark.parametrize('scenario',DATA['scenarios'],ids=lambda s:s['id'])
def test_exported_dynamics_and_collision_checks(scenario):
    for run in scenario['runs']:
        t=run['trajectory'];time=np.array(t['time']);s,n,v,vn=map(np.array,(t['s'],t['n'],t['speed'],t['lateral_speed']))
        controls=np.array(run['controls']);P,V=dynamics_map(time)
        np.testing.assert_allclose(s,scenario['speed']*time+P@controls[:,0],atol=1e-7)
        np.testing.assert_allclose(n,P@controls[:,1],atol=1e-7)
        np.testing.assert_allclose(v,scenario['speed']+V@controls[:,0],atol=1e-7)
        np.testing.assert_allclose(vn,V@controls[:,1],atol=1e-7)
        assert abs(n[-1]-3.6)<1e-6 and abs(vn[-1])<1e-6
        assert np.min(controls[:,0])>=-3-1e-7 and np.max(controls[:,0])<=2+1e-7
        assert np.max(abs(controls[:,1]))<=1.2+1e-7
        for vehicle in scenario['vehicles']:
            dx=abs(s-vehicle['s']-vehicle['speed']*time)-4.6
            dy=abs(n-3.6*vehicle['lane'])-1.8
            assert np.min(np.maximum(dx,dy))>=-1e-6
        crossing=run['transitions'][0]
        assert abs(np.interp(crossing['time'],time,n)-1.8)<1e-6
        assert run['metrics']['max_constraint_violation']<2e-5


@pytest.mark.parametrize('scenario',DATA['scenarios'],ids=lambda s:s['id'])
def test_coarse_transition_has_required_gap_clearance(scenario):
    vehicles={v['name']:v for v in scenario['vehicles']}
    for run in scenario['runs']:
        transition=run['transitions'][1];tau,sigma=transition['time'],transition['s']
        last=run['trajectory']['s'][-1]
        assert 18*(tau-6)-1e-5 <= sigma-last <=34*(tau-6)+1e-5
        for rear,front in run['gaps']:
            for name,sign in [(rear,1),(front,-1)]:
                if name is None: continue
                vehicle=vehicles[name]
                for offset in [-1.5,1.5]:
                    separation=sign*((sigma+29*offset)-(vehicle['s']+vehicle['speed']*(tau+offset)))
                    assert separation>=11-1e-5
