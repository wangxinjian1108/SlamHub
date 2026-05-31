import numpy as np
import open3d as o3d

from . import register_method
from .base import RegistrationBase, RegistrationResult


@register_method("icp_pl")
class ICPPointToPlaneRegistration(RegistrationBase):
    """Point-to-plane ICP using Open3D.

    Adds target normal estimation (essential for the point-to-plane cost) and
    typically converges to a tighter fit on planar scenes (walls, ground,
    facades) than point-to-point ICP.

    Args:
        max_correspondence_distance: max distance to keep a correspondence (m).
        max_iteration: ICP iterations.
        normal_radius: radius for target normal estimation (m).
        normal_max_nn: max neighbors used for the local plane fit.
    """

    def __init__(self, max_correspondence_distance: float = 1.0,
                 max_iteration: int = 50,
                 normal_radius: float = 1.0,
                 normal_max_nn: int = 30):
        self.max_correspondence_distance = max_correspondence_distance
        self.max_iteration = max_iteration
        self.normal_radius = normal_radius
        self.normal_max_nn = normal_max_nn

    def register(self, source: np.ndarray, target: np.ndarray,
                 initial_guess: np.ndarray = None) -> RegistrationResult:
        if initial_guess is None:
            initial_guess = np.eye(4)

        src_pcd = o3d.geometry.PointCloud()
        src_pcd.points = o3d.utility.Vector3dVector(source.astype(np.float64))

        tgt_pcd = o3d.geometry.PointCloud()
        tgt_pcd.points = o3d.utility.Vector3dVector(target.astype(np.float64))
        tgt_pcd.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=self.normal_radius, max_nn=self.normal_max_nn
            )
        )

        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=self.max_iteration
        )
        estimation = o3d.pipelines.registration.TransformationEstimationPointToPlane()

        result = o3d.pipelines.registration.registration_icp(
            src_pcd, tgt_pcd,
            self.max_correspondence_distance,
            initial_guess,
            estimation,
            criteria,
        )

        info = _point_to_plane_information(
            source_pts=source,
            target_pts=target,
            target_normals=np.asarray(tgt_pcd.normals),
            transformation=np.asarray(result.transformation),
            correspondence_set=np.asarray(result.correspondence_set),
        )

        return RegistrationResult(
            transformation=np.asarray(result.transformation),
            fitness=result.fitness,
            inlier_rmse=result.inlier_rmse,
            num_inliers=len(result.correspondence_set),
            information_matrix=info,
        )


def _point_to_plane_information(source_pts, target_pts, target_normals,
                                 transformation, correspondence_set):
    """Compute the 6×6 information matrix of the point-to-plane cost at the
    converged transformation.

    Cost: r_i = n_i · (R p_i + t - q_i). For a left-perturbation
    ξ = (ω, t_pert), J_i = (a_i, n_i) ∈ R⁶ with a_i = (R p_i) × n_i.
    Hessian H = Σ_i J_i J_iᵀ. The information matrix is then H / σ² where
    σ² is the inlier residual variance.

    Returns a 6×6 numpy array (rotation-first, translation-last block order),
    or None if there are too few correspondences to be meaningful.
    """
    if correspondence_set is None or len(correspondence_set) < 6:
        return None
    src_idx = correspondence_set[:, 0]
    tgt_idx = correspondence_set[:, 1]
    p = source_pts[src_idx].astype(np.float64)
    q = target_pts[tgt_idx].astype(np.float64)
    n = target_normals[tgt_idx].astype(np.float64)

    R = transformation[:3, :3]
    t = transformation[:3, 3]
    src_w = p @ R.T + t  # (M, 3)
    a = np.cross(src_w, n)  # (M, 3)
    J = np.concatenate([a, n], axis=1)  # (M, 6)

    H = J.T @ J  # (6, 6)

    r = np.sum(n * (src_w - q), axis=1)  # (M,)
    sigma2 = float((r ** 2).mean()) if len(r) > 0 else 1.0
    if sigma2 < 1e-12:
        sigma2 = 1e-12
    return H / sigma2
