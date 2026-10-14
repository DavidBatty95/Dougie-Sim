import torch

from dougie_sim.residual_policy import BoundedResidual


def test_residual_freezes_reference_and_bounds_corrections_through_export():
    torch.manual_seed(43)
    baseline = torch.jit.script(torch.nn.Linear(45, 12))
    correction = torch.nn.Linear(622, 12)
    policy = BoundedResidual(baseline, correction, .15)
    observations = torch.randn(32, 622)*10.
    reference = baseline(observations[:, :45]).detach()
    actions = policy(observations)
    assert torch.max(torch.abs(actions-reference)) <= .15001
    actions.square().mean().backward()
    assert all(p.grad is None and not p.requires_grad for p in baseline.parameters())
    assert correction.weight.grad.abs().sum() > 0
    scripted = torch.jit.script(policy)
    torch.testing.assert_close(scripted(observations), actions)
    with torch.no_grad():
        correction.weight.zero_()
        correction.bias.zero_()
    torch.testing.assert_close(policy(observations), reference)
