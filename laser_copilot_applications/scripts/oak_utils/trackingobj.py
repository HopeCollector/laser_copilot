import math
from depthai_sdk.classes import (
    TrackerPacket,
    TrackingDetection,
)
from typing import Tuple, Dict
from depthai import Tracklet

TrackingStatus = Tracklet.TrackingStatus

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi
HFOV = 108.0 * DEG2RAD
VFOV = 93.0 * DEG2RAD
HALF_HFOV = HFOV / 2.0
HALF_VFOV = VFOV / 2.0


def map_range(
    x: float, in_min: float, in_max: float, out_min: float, out_max: float
) -> float:
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class TrackedObj:
    def __init__(self, pkg: TrackingDetection, frame_shape: Tuple[int, ...]) -> None:
        assert len(frame_shape) == 2
        self.__pkg = pkg
        self.__frame_shape = frame_shape
        p = pkg.filtered_3d
        self.__raw_p = [p.z / 1000, -p.x / 1000, p.y / 1000]

    @property
    def distance(self) -> float:
        return math.sqrt(
            self.__raw_p[0] ** 2 + self.__raw_p[1] ** 2 + self.__raw_p[2] ** 2
        )

    @property
    def bbox(self) -> Tuple[int, int, int, int]:
        return self.__pkg.filtered_2d.to_tuple(self.__frame_shape)

    @property
    def confidence(self) -> float:
        return self.__pkg.confidence

    @property
    def pos_ball(self) -> Tuple[float, float, float]:
        """
        return (r, theta, phi) in spherical coordinate
        theta: polar angle
        phi: azimuthal angle
        """
        bbox = self.bbox
        x = (bbox[0] + bbox[2]) / 2
        y = (bbox[1] + bbox[3]) / 2
        return (
            self.distance,
            map_range(
                y,
                0,
                self.__frame_shape[0],
                math.pi / 2.0 - HALF_VFOV,
                math.pi / 2.0 + HALF_VFOV,
            ),
            -map_range(x, 0, self.__frame_shape[1], -HALF_HFOV, HALF_HFOV),
        )

    @property
    def pos_xyz(self) -> Tuple[float, float, float]:
        r, theta, phi = self.pos_ball
        return (
            r * math.sin(theta) * math.cos(phi),
            r * math.sin(theta) * math.sin(phi),
            r * math.cos(theta),
        )

    def __str__(self) -> str:
        x, y, z = self.pos_xyz
        return f"{self.confidence * 100.0 : .1f}% {self.distance:.2f}m\n{x:.1f},{y:.1f},{z:.1f}"


def is_tracklet_ok(tracklet: TrackingDetection) -> bool:
    return (
        tracklet.tracklet.status == TrackingStatus.TRACKED
        and tracklet.filtered_2d is not None
        and tracklet.filtered_3d is not None
    )


def parase_tracking_pkg(pkg: TrackerPacket) -> Dict[int, TrackedObj]:
    img = pkg.decode()
    assert img is not None
    ret = {}
    frame_shape = img.shape[:2]
    for id, tracklets in pkg.tracklets.items():
        tracklet = tracklets[-1]
        if is_tracklet_ok(tracklet):
            ret[id] = TrackedObj(tracklet, frame_shape)
    return ret
