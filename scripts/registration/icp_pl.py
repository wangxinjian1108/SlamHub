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

        return RegistrationResult(
            transformation=np.asarray(result.transformation),
            fitness=result.fitness,
            inlier_rmse=result.inlier_rmse,
            num_inliers=len(result.correspondence_set),
        )
