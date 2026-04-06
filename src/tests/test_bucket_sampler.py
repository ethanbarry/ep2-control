from __future__ import annotations

from src.domain.models import BucketMeasurement, Vector3
from src.rf.bucket_sampler import BucketSampler


def test_bucket_sampler_identifies_strongest_bearing() -> None:
    sampler = BucketSampler(interval_deg=10)

    def provider(yaw_deg: int) -> BucketMeasurement:
        rssi = -42.0 if yaw_deg == 90 else -78.0
        return BucketMeasurement(
            yaw_deg=yaw_deg,
            bucket_deg=yaw_deg,
            correlation_score=0.95 if yaw_deg == 90 else 0.3,
            rssi_dbm=rssi,
            inferred_bearing_deg=float(yaw_deg),
        )

    result, measurements = sampler.collect_spin("scan_1", Vector3(0.0, 0.0, 20.0), provider)

    assert len(measurements) == 36
    assert result.strongest_bucket_deg == 90
    assert result.strongest_rssi_dbm == -42.0
