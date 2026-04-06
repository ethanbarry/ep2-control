from __future__ import annotations

from abc import ABC, abstractmethod

from src.domain.models import BucketMeasurement, FoxConfig, Pose


class DspTimeoutError(RuntimeError):
    """Raised when the DSP service does not respond in time."""


class DspPipelineError(RuntimeError):
    """Raised when the DSP service crashes or returns invalid data."""


class DspPipeline(ABC):
    """
    Stable seam for the RF pipeline.

    The current implementation is a deterministic mock, but this interface is
    intended to survive the future swap to a real RTL-SDR or IQ processing
    microservice. A real implementation would replace the synthetic signal math
    in `measure` with a transport call that forwards live IQ metadata or
    detection results from the SDR pipeline.
    """

    @abstractmethod
    def measure(self, pose: Pose, fox: FoxConfig, yaw_deg: int) -> BucketMeasurement:
        raise NotImplementedError
