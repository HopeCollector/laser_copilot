from depthai_sdk import OakCamera, RecordType
import depthai as dai
from .trackingobj import parase_tracking_pkg, TrackedObj
from typing import Callable, Optional, Dict
import numpy as np

class oak_camera:
    def __init__(
        self, cb: Callable[[Dict[int, TrackedObj], Optional[np.ndarray]], None]
    ):
        self.__device = OakCamera()
        color = self.__device.camera("color")
        left = self.__device.camera("left")
        right = self.__device.camera("right")
        stereo = self.__device.stereo(left=left, right=right)
        stereo.set_auto_ir(True, True)
        stereo.config_stereo(align=color, lr_check=True, subpixel=True)
        nn = self.__device.create_nn(
            # "/home/lawskiy/.local/lib/python3.10/site-packages/depthai_sdk/nn_models/yolov6nr3_coco_640x352/config.json",
            "yolov6nr3_coco_640x352",
            color,
            tracker=True,
            spatial=stereo,
        )
        nn.config_nn(resize_mode="stretch")
        nn.config_tracker(
            tracker_type=dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM,
            track_labels=[
                0
            ],  # Track only 1st object from the object map. If unspecified, track all object types
            # track_labels=['person'] # Track only people (for coco datasets, person is 1st object in the map)
            assignment_policy=dai.TrackerIdAssignmentPolicy.SMALLEST_ID,
            max_obj=10,  # Max objects to track, which can improve performance
            threshold=0.5,  # Tracker threshold
        )
        self.__device.callback(
            nn.out.tracker, callback=lambda x: cb(parase_tracking_pkg(x), x.decode())
        )

    def start(self):
        self.__device.start(blocking=False)
    
    def close(self):
        self.__device.close()

    def is_ok(self) -> bool:
        return self.__device.running()

    def poll(self):
        self.__device.poll()

    def __del__(self):
        self.close()

def track(cb: Callable[[Dict[int, TrackedObj], Optional[np.ndarray]], None]):
    with OakCamera() as oak:
        color = oak.camera("color")
        left = oak.camera("left")
        right = oak.camera("right")
        stereo = oak.stereo(left=left, right=right)
        stereo.set_auto_ir(True, True)
        stereo.config_stereo(align=color, lr_check=True, subpixel=True)
        nn = oak.create_nn(
            # "/home/lawskiy/.local/lib/python3.10/site-packages/depthai_sdk/nn_models/yolov6nr3_coco_640x352/config.json",
            "yolov6nr3_coco_640x352",
            color,
            tracker=True,
            spatial=stereo,
        )
        nn.config_nn(resize_mode="stretch")
        nn.config_tracker(
            tracker_type=dai.TrackerType.ZERO_TERM_COLOR_HISTOGRAM,
            track_labels=[
                0
            ],  # Track only 1st object from the object map. If unspecified, track all object types
            # track_labels=['person'] # Track only people (for coco datasets, person is 1st object in the map)
            assignment_policy=dai.TrackerIdAssignmentPolicy.SMALLEST_ID,
            max_obj=10,  # Max objects to track, which can improve performance
            threshold=0.5,  # Tracker threshold
        )
        oak.callback(nn.out.tracker, callback=lambda x : cb(parase_tracking_pkg(x), x.decode()))
        oak.start(blocking=True)

def record(path: str, cb: Callable[[Optional[np.ndarray]], None]):
    with OakCamera() as oak:
        # color = oak.create_camera('color', resolution='1080P', fps=30)
        color = oak.create_camera("color", resolution="1920x1080", fps=30, encode="h265")
        left = oak.create_camera("left", resolution="800p", fps=30, encode="h265")
        right = oak.create_camera("right", resolution="800p", fps=30, encode="h265")
        oak.create_stereo(
            "800p", fps=30, left=left, right=right, encode="h265"
        ).set_auto_ir(auto_mode=True, continuous_mode=True)
        # Synchronize & save all (encoded) streams
        oak.record(
            [color.out.encoded, left.out.encoded, right.out.encoded],
            path,
            RecordType.VIDEO,
        )
        # Show color stream
        oak.callback(color.out.main, callback=lambda x : cb(x.decode()))
        # oak.start(blocking=False)
        oak.start(blocking=True)
