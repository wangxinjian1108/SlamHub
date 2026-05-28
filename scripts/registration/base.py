from abc import ABC, abstractmethod
from dataclasses import dataclass

import numpy as np


@dataclass
class RegistrationResult:
    """Result of a point cloud registration."""

    transformation: np.ndarray  # 4x4 homogeneous transformation matrix
    fitness: float  # overlap ratio [0, 1]
    inlier_rmse: float  # RMSE of inlier correspondences
    num_inliers: int  # number of inlier correspondences


class RegistrationBase(ABC):
    """Abstract base class for point cloud registration methods."""

    @abstractmethod
    def register(self, source: np.ndarray, target: np.ndarray,
                 initial_guess: np.ndarray = None) -> RegistrationResult:
        """Register source point cloud to target.

        Args:
            source: (N, 3) array of source points.
            target: (M, 3) array of target points.
            initial_guess: Optional 4x4 initial transformation. Defaults to identity.

        Returns:
            RegistrationResult with the estimated transformation and metrics.
        """
        ...
