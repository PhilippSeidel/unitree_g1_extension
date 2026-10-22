import base64
import json
import logging
from dataclasses import dataclass

import cv2
import zmq
import time

import numpy as np
from incar.extensions.cameras import Camera, CameraConfig
 
log = logging.getLogger(__name__)


@CameraConfig.register_subclass("g1_zmq")
@dataclass
class G1ZmqConfig(CameraConfig):
    host: str = "192.168.123.164"
    port: int = 5555
    camera_name: str = "head_camera"
    def build_camera(self): return G1ZmqCamera(self)

class G1ZmqCamera(Camera):
    def __init__(self, conf):
        self.name = conf.camera_name
        self.sock = zmq.Context.instance().socket(zmq.SUB)
        self.sock.setsockopt(zmq.CONFLATE, 1)          
        self.sock.setsockopt(zmq.RCVTIMEO, 2000)
        self.sock.setsockopt_string(zmq.SUBSCRIBE, "")
        self.addr = f"tcp://{conf.host}:{conf.port}"
        self.sock.connect(self.addr)

    def next_frame(self):
           names, t_warn = None, time.monotonic()
           while True:
               try:
                   images = json.loads(self.sock.recv())["images"]
                   names = list(images)
                   img = images.get(self.name)
                   if img is not None:
                       jpeg = np.frombuffer(base64.b64decode(img), np.uint8)
                       frame = cv2.imdecode(jpeg, cv2.IMREAD_COLOR)  # same channel order as encoded: RGB
                       if frame is not None:
                           return frame
               except zmq.Again:
                   pass
               except (ValueError, KeyError, TypeError):
                   pass
               if time.monotonic() - t_warn > 5:
                   log.warning("no '%s' frame from %s yet; cameras in stream: %s",
                               self.name, self.addr, names)
                   t_warn = time.monotonic()



 
 
@CameraConfig.register_subclass("g1_sim")
@dataclass
class G1SimCameraConfig(CameraConfig):
    server_address: str = "127.0.0.1" 
    port: int = 5555                    
    camera_name: str = "head_camera"    
    width: int = 640                   
    height: int = 480
    timeout_s: float = 2.0              
 
    def build_camera(self) -> "G1SimCamera":
        return G1SimCamera(self)
 
 
class G1SimCamera(Camera):
    def __init__(self, conf: G1SimCameraConfig):
 
        self._cv2 = cv2
        self._zmq = zmq
        self.conf = conf
 
        frame_type = getattr(conf.frame_type, "name", conf.frame_type)
        if str(frame_type).upper() != "RGB":
            log.warning("g1_sim cameras only provide RGB; frame_type=%s is ignored", frame_type)
 
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.SUB)
        self._sock.setsockopt(zmq.CONFLATE, 1) 
        self._sock.setsockopt_string(zmq.SUBSCRIBE, "")
        self._sock.setsockopt(zmq.RCVTIMEO, int(conf.timeout_s * 1000))
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.connect(f"tcp://{conf.server_address}:{conf.port}")
 
    def next_frame(self) -> np.ndarray:
        while True:
            try:
                msg = self._sock.recv()
            except self._zmq.Again:
                log.warning(
                    "g1_sim %s: no frame for %.1f s from tcp://%s:%d. Is the sim running with "
                    "sim_publish_images=True?",
                    self.conf.camera_name, self.conf.timeout_s, self.conf.server_address, self.conf.port,
                )
                continue
            frame = self._decode(msg)
            if frame is not None:
                return frame
 
    def _decode(self, msg: bytes) -> np.ndarray | None:
        try:
            images = json.loads(msg)["images"]
        except (ValueError, KeyError, TypeError):
            log.warning("g1_sim: dropped a message that isn't the sim's image format")
            return None
        if self.conf.camera_name not in images:
            raise ValueError(
                f"g1_sim: camera {self.conf.camera_name!r} is not in the stream; "
                f"available: {sorted(images)}"
            )
 
        cv2 = self._cv2
        jpeg = np.frombuffer(base64.b64decode(images[self.conf.camera_name]), np.uint8)
        frame = cv2.imdecode(jpeg, cv2.IMREAD_COLOR)  # same channel order as encoded: RGB
        if frame is None:
            log.warning("g1_sim %s: could not decode a JPEG", self.conf.camera_name)
            return None
 
        w, h = self.conf.width, self.conf.height
        if w > 0 and h > 0 and (frame.shape[1], frame.shape[0]) != (w, h):
            frame = cv2.resize(frame, (w, h), interpolation=cv2.INTER_AREA)
        return frame
 
