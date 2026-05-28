import numpy as np
import open3d as o3d

from . import register_method
from .base import RegistrationBase, RegistrationResult


@register_method("icp")
class ICPRegistration(RegistrationBase):
    """ICP registration using Open3D's point-to-point ICP."""

    def __init__(self, max_correspondence_distance: float = 1.0,
                 max_iteration: int = 50):
        self.max_correspondence_distance = max_correspondence_distance
        self.max_iteration = max_iteration

    def register(self, source: np.ndarray, target: np.ndarray,
                 initial_guess: np.ndarray = None) -> RegistrationResult:
        """Register source to target using ICP.

        Args:
            source: (N, 3) array of source points.
            target: (M, 3) array of target points.
            initial_guess: Optional 4x4 initial transformation.

        Returns:
            RegistrationResult with estimated transformation and metrics.
        """
        if initial_guess is None:
            initial_guess = np.eye(4)

        src_pcd = o3d.geometry.PointCloud()
        src_pcd.points = o3d.utility.Vector3dVector(source.astype(np.float64))

        tgt_pcd = o3d.geometry.PointCloud()
        tgt_pcd.points = o3d.utility.Vector3dVector(target.astype(np.float64))

        criteria = o3d.pipelines.registration.ICPConvergenceCriteria(
            max_iteration=self.max_iteration
        )
        estimation = o3d.pipelines.registration.TransformationEstimationPointToPoint()

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
