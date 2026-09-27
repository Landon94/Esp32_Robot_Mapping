from dataclasses import dataclass


@dataclass
class RPMData:
    sequence: int
    timestamp_us: int
    left_rpm: float
    right_rpm: float


@dataclass
class LidarSampleData:
    angle_deg: float
    distance_mm: float
    quality: int
    start_flag: bool


@dataclass
class LidarBatchData:
    sequence: int
    timestamp_us: int
    samples: list[LidarSampleData]

@dataclass
class FullScan:
    timestamp_us: int
    samples: list[LidarSampleData]

@dataclass
class FullScanCartesian:
    timestamp_us: int
    samples: list[(int,int)]