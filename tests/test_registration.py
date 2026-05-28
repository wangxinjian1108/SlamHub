import numpy as np
import pytest

from scripts.registration.base import RegistrationBase, RegistrationResult
from scripts.registration import get_registration_method
from scripts.registration.icp import ICPRegistration
from scripts.common.transform import euler_to_matrix, make_homogeneous


def test_registration_result_fields():
    """RegistrationResult has correct fields."""
    T = np.eye(4)
    result = RegistrationResult(
        transformation=T, fitness=0.95, inlier_rmse=0.01, num_inliers=100
    )
    assert result.transformation is T
    assert result.fitness == 0.95
    assert result.inlier_rmse == 0.01
    assert result.num_inliers == 100


def test_icp_is_registration_base():
    """ICPRegistration is an instance of RegistrationBase."""
    icp = ICPRegistration()
    assert isinstance(icp, RegistrationBase)


def test_get_registration_method():
    """get_registration_method('icp') returns a RegistrationBase instance."""
    method = get_registration_method("icp")
    assert isinstance(method, RegistrationBase)


def test_icp_identity_alignment():
    """Two identical clouds should yield near-identity transform and high fitness."""
    rng = np.random.default_rng(42)
    cloud = rng.random((200, 3))

    icp = ICPRegistration(max_correspondence_distance=1.0, max_iteration=50)
    result = icp.register(cloud, cloud)

    np.testing.assert_allclose(result.transformation, np.eye(4), atol=1e-6)
    assert result.fitness > 0.9


def test_icp_known_transform():
    """ICP should recover a small known transform within tolerance."""
    rng = np.random.default_rng(123)
    source = rng.random((300, 3))

    # Small rotation (0.05 rad yaw) and translation [0.1, 0.2, 0]
    R = euler_to_matrix(0.0, 0.0, 0.05)
    T_known = make_homogeneous(R, np.array([0.1, 0.2, 0.0]))

    # Transform source to get target: target = T_known @ source
    source_h = np.hstack([source, np.ones((len(source), 1))])
    target = (T_known @ source_h.T).T[:, :3]

    # ICP aligns source -> target, so it should recover T_known
    icp = ICPRegistration(max_correspondence_distance=1.0, max_iteration=100)
    result = icp.register(source, target)

    np.testing.assert_allclose(result.transformation, T_known, atol=0.1)
    assert result.fitness > 0.9
