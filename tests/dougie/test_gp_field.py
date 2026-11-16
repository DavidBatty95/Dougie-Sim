import json
from types import SimpleNamespace
import zipfile

import numpy as np
import pytest

from dougie_sim.gp_field import interpolate_field,observed_field_queries
from dougie_sim.spatial_gp import SpatialGP,load_gp
from dougie_sim.windowed_gp import WindowedGP


class ConstantExpert:
    def __init__(self,rate):
        self.rate = rate

    def predict_log(self,points):
        return np.full(len(points),np.log(self.rate)),np.full(len(points),.1)


def blended():
    gp = SpatialGP()
    gp.experts[(0,0,0)] = ConstantExpert(2.)
    gp.experts[(1,0,0)] = ConstantExpert(100.)
    return gp


@pytest.mark.parametrize('boundary',[2.4,3.2,4.,4.8,5.6])
def test_mean_uncertainty_and_upper_are_continuous_at_expert_support_edges(boundary):
    gp = blended()
    points = [[boundary+dx,2.,.75] for dx in (-1e-7,0.,1e-7)]
    for field in gp.predict(points):
        assert np.ptp(field) < 1e-3
        assert np.isfinite(field).all()


@pytest.mark.parametrize('boundary',[3.2,4.,4.8])
def test_mean_has_no_linear_blending_kink(boundary):
    gp = blended()
    h = 1e-4
    mean = gp.predict([[boundary+dx,2.,.75] for dx in (-h,0.,h)])[0]
    left,right = (mean[1]-mean[0])/h,(mean[2]-mean[1])/h
    assert abs(left-right) < 1e-3


def test_expanded_upper_never_discards_an_active_hot_expert():
    gp = blended()
    query = [[x,2.,.75] for x in np.linspace(3.2001,4.7999,101)]
    mean,std,upper = gp.predict(query)
    assert np.all(upper>=100.*np.exp(2*np.sqrt(.1))-1e-10)
    assert std[50]**2 > 3.
    assert np.all(upper>=mean)


@pytest.mark.parametrize('kind',['spatial','windowed'])
def test_new_blending_survives_model_export_and_lazy_reload(tmp_path,kind):
    gp = SpatialGP(capacity=8) if kind=='spatial' else WindowedGP(capacity=8,max_experts=1,max_planes=0)
    for x,count in ((2.,2),(3.8,10),(4.2,30),(6.,40)):
        gp.update_counts([x,2.,.75],count,.5)
    query = np.array([[x,2.,.75] for x in np.linspace(2.3,5.7,121)])
    before = gp.predict(query)
    gp.save(tmp_path/'field.npz')
    restored = load_gp(tmp_path/'field.npz')
    assert restored.blend == 'quintic'
    for actual,expected in zip(restored.predict(query),before):
        np.testing.assert_allclose(actual,expected,rtol=1e-10)


def test_legacy_spatial_checkpoint_keeps_its_recorded_linear_blending(tmp_path):
    gp = SpatialGP(capacity=8,blend='linear')
    gp.update_counts([3.9,2.,.75],20,.5)
    gp.save(tmp_path/'new.npz')
    with np.load(tmp_path/'new.npz') as archive:
        legacy = {k:archive[k] for k in archive.files if k!='blend'}
    legacy['schema_version'] = 1
    np.savez_compressed(tmp_path/'legacy.npz',**legacy)
    restored = load_gp(tmp_path/'legacy.npz')
    assert restored.blend == 'linear'
    np.testing.assert_allclose(restored.predict([[3.4,2.,.75]]),gp.predict([[3.4,2.,.75]]))


def test_legacy_windowed_checkpoint_keeps_recorded_blending(tmp_path):
    gp = WindowedGP(capacity=8,max_planes=0,blend='linear')
    gp.update_counts([3.9,2.,.75],20,.5)
    gp.save(tmp_path/'new.npz')
    with zipfile.ZipFile(tmp_path/'new.npz') as source:
        with np.load(tmp_path/'new.npz') as arrays:
            metadata = json.loads(str(arrays['metadata'].item()))
        metadata.pop('blend')
        with zipfile.ZipFile(tmp_path/'legacy.npz','w') as output:
            for name in source.namelist():
                if name in ('metadata.npy','schema_version.npy'):
                    with output.open(name,'w') as stream:
                        value = json.dumps(metadata) if name=='metadata.npy' else 1
                        np.lib.format.write_array(stream,np.asarray(value),allow_pickle=False)
                else:
                    output.writestr(name,source.read(name))
    restored = load_gp(tmp_path/'legacy.npz')
    assert restored.blend == 'linear'
    np.testing.assert_allclose(restored.predict([[3.4,2.,.75]]),gp.predict([[3.4,2.,.75]]))


def test_display_field_interpolates_instead_of_switching_nearest_samples():
    xyz = np.array([[0.,0.,.44],[.4,0.,.44],[.2,0.,2.84]])
    mean,std,upper,_,count = interpolate_field(
        [[.1999,0.,.44],[.2,0.,.44],[.2001,0.,.44],[.2,0.,1.64],[4.,0.,.44]],
        xyz,[2.,10.,1000.],[.4,.4,.4],[4.,20.,2000.])
    assert mean[0] < mean[1] < mean[2] and np.ptp(mean[:3]) < .1
    assert mean[1] == pytest.approx(6.) and upper[1] == pytest.approx(12.)
    assert std[1] > .4  # Disagreement adds uncertainty; it is not precision fusion.
    assert list(count) == [2,2,2,0,0]
    assert np.isnan(mean[3:]).all()


def test_display_reproduces_samples_and_does_not_multiply_correlated_confidence():
    xyz = np.repeat([[0.,0.,.44]],30,axis=0)
    mean,std,upper,_,_ = interpolate_field([[0.,0.,.44]],xyz,
        np.full(30,2.),np.full(30,.7),np.full(30,5.))
    np.testing.assert_allclose([mean[0],std[0],upper[0]],[2.,.7,5.])


def test_dense_field_queries_use_only_observed_heights_and_keep_measurement_priority():
    support = np.array([[x,y,z] for x in np.arange(0.,2.,.1) for y in np.arange(0.,2.,.1) for z in (0.,2.4)])
    terrain = SimpleNamespace(support_returns=dict(enumerate(support)),nodes={})
    queries = observed_field_queries(terrain,[[9.,9.,.44]],capacity=1000)
    assert len(queries) > 100
    assert {round(z,2) for z in queries[:,2]} == {.44,2.84}
    assert np.any(np.all(queries==[9.,9.,.44],axis=1))
    bounded = observed_field_queries(terrain,[[9.,9.,.44]],capacity=20)
    assert len(bounded)<=20 and np.any(np.all(bounded==[9.,9.,.44],axis=1))


def test_invalid_exported_predictions_remain_missing_instead_of_zero():
    values = interpolate_field([[0.,0.,.44]],[[0.,0.,.44]],[float('nan')],[.5],[10.])
    assert np.isnan(values[0]).all() and values[-1][0]==0


def test_legacy_quintic_archive_keeps_original_prior_envelope(tmp_path):
    gp=SpatialGP(capacity=8,upper_rule='expanded_prior')
    for _ in range(10):
        gp.update_counts([2.,2.,.44],1,.5)
    gp.save(tmp_path/'new.npz')
    with np.load(tmp_path/'new.npz') as archive:
        legacy={k:archive[k] for k in archive.files if k!='upper_rule'}
    legacy['schema_version']=2
    np.savez_compressed(tmp_path/'legacy.npz',**legacy)
    restored=load_gp(tmp_path/'legacy.npz')
    assert restored.upper_rule=='expanded_prior'
    np.testing.assert_allclose(restored.predict([[2.,2.,.44]]),gp.predict([[2.,2.,.44]]))
