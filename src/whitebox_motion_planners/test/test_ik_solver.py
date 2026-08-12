import pytest
import numpy as np

from whitebox_motion_planners.kinematics import CommunityArmKinematics, CommunityArmIKSolver

@pytest.fixture
def arm_kinematics():
    link_lengths = {
        'base_height': 0.130,
        'lower_shank': 0.140,
        'upper_shank': 0.140,
        'gripper_dx': -0.05467,
        'gripper_dz': -0.0217,
        'gripper_k_elbow': 0.0
    }

    return CommunityArmKinematics(use_horizontal_constraint=False, link_lengths=link_lengths)

@pytest.fixture
def ik_solver(arm_kinematics):
    return CommunityArmIKSolver(arm_kinematics)

def test_reachability(ik_solver):
    # Base height = 0.130, L1 = 0.140, L2_eff = 0.140 - 0.05467 = 0.08533
    # Max reach = 0.140 + 0.08533 = 0.22533 m
    assert ik_solver.is_reachable(0.15, 0.0, 0.20) is True
    assert ik_solver.is_reachable(0.0, 0.20, 0.25) is True
    assert ik_solver.is_reachable(1.5, 0.0, 0.0) is False  # Too far

def test_ik_roundtrip(arm_kinematics, ik_solver):
    """
    Test that FK(IK(x, y, z)) returns (x, y, z) accurately for multiple reachable points.
    """
    test_points = [
        (0.15, 0.0, 0.20),
        (0.0, 0.18, 0.22),
        (-0.12, 0.12, 0.18),
        (0.10, -0.10, 0.25),
    ]

    for target in test_points:
        tx, ty, tz = target
        q_sol = ik_solver.compute_ik(tx, ty, tz, elbow_up=True)
        assert q_sol is not None, f"Target {target} should be reachable"

        # Compute FK using the resolved angles
        xyz_fk = arm_kinematics.compute_forward_kinematics_gripper(q_sol)
        
        np.testing.assert_allclose(
            xyz_fk, 
            np.array([tx, ty, tz]), 
            atol=1e-3, 
            err_msg=f"FK(IK({target})) produced {xyz_fk}, expected {target}"
        )

def test_ik_from_known_fk(arm_kinematics, ik_solver):
    """
    Test that for a given set of joint angles q_orig, FK(q_orig) -> (x,y,z) -> IK(x,y,z) -> q_sol -> FK(q_sol) matches (x,y,z).
    """
    sample_qs = [
        (0.0, np.pi/4, np.pi),
        (np.pi/3, np.pi/3, 5*np.pi/6),
        (-np.pi/4, np.pi/6, 7*np.pi/6),
    ]

    for q_orig in sample_qs:
        xyz_target = arm_kinematics.compute_forward_kinematics_gripper(q_orig)
        q_sol = ik_solver.compute_ik(xyz_target[0], xyz_target[1], xyz_target[2], elbow_up=True)
        assert q_sol is not None
        xyz_recomputed = arm_kinematics.compute_forward_kinematics_gripper(q_sol)
        np.testing.assert_allclose(xyz_recomputed, xyz_target, atol=1e-3)
