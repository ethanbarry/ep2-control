from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass
import math
from typing import Callable

from src.domain.models import BucketMeasurement, ScanBucket, ScanResult, Vector3


MeasurementProvider = Callable[[int], BucketMeasurement]


@dataclass
class BucketSampler:
    interval_deg: int

    def __post_init__(self) -> None:
        assert self.interval_deg > 0, "bucket interval must be positive"
        assert 360 % self.interval_deg == 0, "bucket interval must evenly divide 360 degrees"

    def collect_spin(self, scan_label: str, origin: Vector3, provider: MeasurementProvider) -> tuple[ScanResult, list[BucketMeasurement]]:
        assert scan_label
        grouped_rssi: dict[int, list[float]] = defaultdict(list)
        grouped_correlation: dict[int, list[float]] = defaultdict(list)
        measurements: list[BucketMeasurement] = []
        for yaw_deg in range(0, 360, self.interval_deg):
            measurement = provider(yaw_deg)
            measurements.append(measurement)
            grouped_rssi[measurement.bucket_deg].append(measurement.rssi_dbm)
            grouped_correlation[measurement.bucket_deg].append(measurement.correlation_score)

        buckets: list[ScanBucket] = []
        for bucket_deg in sorted(grouped_rssi):
            rssi_values = grouped_rssi[bucket_deg]
            corr_values = grouped_correlation[bucket_deg]
            buckets.append(
                ScanBucket(
                    bucket_deg=bucket_deg,
                    sample_count=len(rssi_values),
                    average_rssi_dbm=sum(rssi_values) / len(rssi_values),
                    average_correlation=sum(corr_values) / len(corr_values),
                )
            )

        strongest = max(
            buckets,
            key=lambda bucket: (bucket.average_rssi_dbm, bucket.average_correlation),
        )
        estimated_bearing_deg = self._estimated_bearing_deg(buckets)
        return (
            ScanResult(
                scan_label=scan_label,
                origin=origin,
                bucket_interval_deg=self.interval_deg,
                buckets=buckets,
                strongest_bucket_deg=strongest.bucket_deg,
                estimated_bearing_deg=estimated_bearing_deg,
                strongest_rssi_dbm=strongest.average_rssi_dbm,
            ),
            measurements,
        )

    def _estimated_bearing_deg(self, buckets: list[ScanBucket]) -> float:
        assert buckets, "cannot estimate bearing from an empty bucket set"
        minimum_rssi = min(bucket.average_rssi_dbm for bucket in buckets)
        weighted_x = 0.0
        weighted_y = 0.0
        for bucket in buckets:
            weight = 10 ** ((bucket.average_rssi_dbm - minimum_rssi) / 10.0)
            radians = math.radians(bucket.bucket_deg)
            weighted_x += weight * math.cos(radians)
            weighted_y += weight * math.sin(radians)
        return math.degrees(math.atan2(weighted_y, weighted_x)) % 360.0
