from __future__ import annotations

import random

from src.domain.enums import DspFailureMode
from src.domain.models import BucketMeasurement, DspConfig, FoxConfig, Pose
from src.rf.dsp_interface import DspPipeline, DspPipelineError, DspTimeoutError
from src.rf.signal_model import inverse_distance_rssi_dbm, normalize_angle_deg


class MockDspPipeline(DspPipeline):
    def __init__(self, config: DspConfig, seed: int) -> None:
        self._config = config
        self._random = random.Random(seed)
        self._sample_count = 0

    def measure(self, pose: Pose, fox: FoxConfig, yaw_deg: int) -> BucketMeasurement:
        self._sample_count += 1
        self._check_failure_mode()

        # This is the single hand-off point to replace when we wire a real SDR
        # or IQ-derived detection service. The rest of the mission stack should
        # remain unchanged as long as the service returns the same shape.
        rssi_dbm, true_bearing = inverse_distance_rssi_dbm(
            pose=pose,
            fox=fox,
            yaw_deg=float(yaw_deg),
            bias_bearing_deg=self._config.forced_bearing_bias_deg,
        )
        noisy_rssi = rssi_dbm + self._random.gauss(0.0, self._config.noise_std_db)
        if self._random.random() < self._config.outlier_probability:
            noisy_rssi += self._random.choice([-1.0, 1.0]) * self._config.outlier_magnitude_db

        correlation_score = max(
            0.0,
            min(
                1.0,
                1.0 - abs(noisy_rssi + 90.0) / 70.0,
            ),
        )
        return BucketMeasurement(
            yaw_deg=yaw_deg,
            bucket_deg=yaw_deg,
            correlation_score=correlation_score,
            rssi_dbm=noisy_rssi,
            inferred_bearing_deg=normalize_angle_deg(true_bearing + self._config.forced_bearing_bias_deg),
        )

    def _check_failure_mode(self) -> None:
        if self._config.crash_after_samples is not None and self._sample_count >= self._config.crash_after_samples:
            raise DspPipelineError("mock DSP crashed after configured sample count")
        if self._config.timeout_after_samples is not None and self._sample_count >= self._config.timeout_after_samples:
            raise DspTimeoutError("mock DSP timed out after configured sample count")
        if self._config.failure_mode == DspFailureMode.CRASH:
            raise DspPipelineError("mock DSP crash mode enabled")
        if self._config.failure_mode == DspFailureMode.TIMEOUT:
            raise DspTimeoutError("mock DSP timeout mode enabled")
        if self._config.timeout_probability > 0.0 and self._random.random() < self._config.timeout_probability:
            raise DspTimeoutError("mock DSP probabilistic timeout")
