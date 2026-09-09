import time
import numpy as np
import cv2
from typing import Tuple, Optional, Dict, Any

# Try importing pyrealsense2
try:
    import pyrealsense2 as rs
    PYREALSENSE2_AVAILABLE = True
except ImportError:
    PYREALSENSE2_AVAILABLE = False


class RealSenseCamera:
    """
    Intel RealSense RGB-D / depth-only camera manager.
    Uses color when available, otherwise uses the left infrared stream (IR1)
    together with depth. The returned image is always BGR so existing ArUco
    and tracking code can be reused unchanged.
    """
    def __init__(
        self, 
        width: int = 640, 
        height: int = 480, 
        fps: int = 30,
        enable_depth: bool = True,
        infrared_preprocess: bool = True,
        disable_ir_emitter: bool = False
    ):
        self.width = width
        self.height = height
        self.fps = fps
        self.enable_depth = enable_depth
        self.infrared_preprocess = infrared_preprocess
        self.disable_ir_emitter = disable_ir_emitter
        
        self.pipeline = None
        self.config = None
        self.align = None
        self.profile = None
        
        self.camera_matrix: Optional[np.ndarray] = None
        self.dist_coeffs: Optional[np.ndarray] = None
        self.is_rs_active = False
        self.device_name: Optional[str] = None
        self.device_serial: Optional[str] = None
        self.frame_source = "none"  # "color", "infrared", "opencv", or "synthetic"
        self.depth_scale_m = 0.001
        
        # Fallback VideoCapture if pyrealsense2 is unavailable or device missing
        self.cap = None

    def start(self) -> bool:
        """
        Start camera stream. Attempts pyrealsense2 hardware connection first.
        Falls back to cv2.VideoCapture(0) if hardware pipeline fails.
        """
        if PYREALSENSE2_AVAILABLE:
            try:
                # Prefer RGB, but allow a depth-only module to use its IR image.
                device = rs.context().query_devices()[0]
                self.device_name = device.get_info(rs.camera_info.name)
                self.device_serial = device.get_info(rs.camera_info.serial_number)
                color_profiles = []
                infrared_profiles = []
                for sensor in device.query_sensors():
                    for profile in sensor.get_stream_profiles():
                        try:
                            video = profile.as_video_stream_profile()
                            if video.stream_type() == rs.stream.color:
                                color_profiles.append(profile)
                            elif video.stream_type() == rs.stream.infrared:
                                infrared_profiles.append(profile)
                        except RuntimeError:
                            continue
                if color_profiles:
                    self.frame_source = "color"
                    image_stream = rs.stream.color
                    image_format = rs.format.bgr8
                elif infrared_profiles:
                    self.frame_source = "infrared"
                    image_stream = rs.stream.infrared
                    image_format = rs.format.y8
                    print(
                        f"[RealSenseCamera] {self.device_name} has no color sensor; "
                        "using infrared stream 1 for ArUco/PnP."
                    )
                else:
                    raise RuntimeError(
                        f"SDK device '{self.device_name}' (S/N {self.device_serial}) "
                        "does not expose color or infrared video streams."
                    )

                self.pipeline = rs.pipeline()
                self.config = rs.config()
                
                # Enable color or IR image stream. IR1 is the left stereo image
                # and is the correct image plane for depth alignment/PnP.
                if self.frame_source == "infrared":
                    self.config.enable_stream(
                        rs.stream.infrared, 1,
                        self.width, self.height, image_format, self.fps
                    )
                else:
                    self.config.enable_stream(
                        image_stream, self.width, self.height,
                        image_format, self.fps
                    )
                
                # Enable Depth stream if requested
                if self.enable_depth:
                    self.config.enable_stream(
                        rs.stream.depth, 
                        self.width, 
                        self.height, 
                        rs.format.z16, 
                        self.fps
                    )
                    # Align depth to the actual image plane (RGB or IR).
                    self.align = rs.align(image_stream)
                
                # Start pipeline
                self.profile = self.pipeline.start(self.config)

                if self.enable_depth:
                    try:
                        depth_sensor = self.profile.get_device().first_depth_sensor()
                        self.depth_scale_m = float(depth_sensor.get_depth_scale())
                        if (
                            self.frame_source == "infrared"
                            and self.disable_ir_emitter
                            and depth_sensor.supports(rs.option.emitter_enabled)
                        ):
                            depth_sensor.set_option(rs.option.emitter_enabled, 0.0)
                            print("[RealSenseCamera] IR emitter disabled for cleaner ArUco images; depth may be noisier.")
                    except Exception:
                        self.depth_scale_m = 0.001
                
                # Extract camera intrinsics directly from SDK
                image_stream_profile = self.profile.get_stream(
                    rs.stream.infrared, 1
                ).as_video_stream_profile() if self.frame_source == "infrared" else self.profile.get_stream(
                    rs.stream.color
                ).as_video_stream_profile()
                intrinsics = image_stream_profile.get_intrinsics()
                
                self.camera_matrix = np.array([
                    [intrinsics.fx, 0, intrinsics.ppx],
                    [0, intrinsics.fy, intrinsics.ppy],
                    [0, 0, 1]
                ], dtype=np.float64)
                
                self.dist_coeffs = np.array(intrinsics.coeffs, dtype=np.float64)
                self.is_rs_active = True
                print(f"[RealSenseCamera] Hardware RealSense {self.device_name} started successfully using {self.frame_source} ({self.width}x{self.height}@{self.fps}FPS).")
                print(f"[RealSenseCamera] SDK Intrinsics ({self.frame_source}): fx={intrinsics.fx:.2f}, fy={intrinsics.fy:.2f}, cx={intrinsics.ppx:.2f}, cy={intrinsics.ppy:.2f}")
                return True
            except Exception as e:
                print(f"[RealSenseCamera] RealSense SDK start failed ({e}). Falling back to OpenCV VideoCapture...")
                self.is_rs_active = False
                if self.pipeline:
                    try:
                        self.pipeline.stop()
                    except Exception:
                        pass
                    self.pipeline = None

        # Fallback mode
        return self._start_fallback()

    def _start_fallback(self) -> bool:
        """Initialize OpenCV VideoCapture fallback and set default camera matrix."""
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        
        # Default estimated intrinsic matrix K for standard 640x480 webcam
        fx = 615.0  # Approx D435 color focal length
        fy = 615.0
        cx = self.width / 2.0
        cy = self.height / 2.0
        
        self.camera_matrix = np.array([
            [fx, 0, cx],
            [0, fy, cy],
            [0, 0, 1]
        ], dtype=np.float64)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)
        
        if self.cap.isOpened():
            self.frame_source = "opencv"
            print(f"[RealSenseCamera] Standard OpenCV camera fallback opened on /dev/video0.")
            return True
        else:
            self.frame_source = "synthetic"
            print(f"[RealSenseCamera] Warning: No camera device opened. Synthetic frame generator mode active.")
            return True

    def get_frame(self) -> Tuple[bool, np.ndarray, Optional[Any], Optional[np.ndarray]]:
        """
        Fetch next color image and depth frame.
        
        Returns:
            Tuple of (success, color_image, depth_frame, depth_image_vis)
            - color_image: BGR np.ndarray (H, W, 3); for depth-only devices
              this is the IR1 grayscale image replicated to three channels
            - depth_frame: rs.depth_frame object (if pyrealsense2 active) else None
            - depth_image_vis: Colorized depth map np.ndarray (H, W, 3) for visualization
        """
        if self.is_rs_active and self.pipeline:
            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=5000)
                if self.align:
                    frames = self.align.process(frames)
                    
                if self.frame_source == "infrared":
                    image_frame = frames.get_infrared_frame(1)
                else:
                    image_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame() if self.enable_depth else None
                
                if not image_frame:
                    return False, np.array([]), None, None
                
                image = np.asanyarray(image_frame.get_data())
                if self.frame_source == "infrared" and self.infrared_preprocess:
                    # Suppress isolated projector speckles while preserving
                    # the square black/white cells of an ArUco marker.
                    image = cv2.medianBlur(image, 3)
                color_image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if self.frame_source == "infrared" else image
                
                depth_image_vis = None
                if depth_frame:
                    depth_data = np.asanyarray(depth_frame.get_data())
                    # Convert depth image to 8-bit colormap for display
                    depth_image_vis = cv2.applyColorMap(
                        cv2.convertScaleAbs(depth_data, alpha=0.03), 
                        cv2.COLORMAP_JET
                    )
                    
                return True, color_image, depth_frame, depth_image_vis
            except Exception as e:
                print(f"[RealSenseCamera] Frame capture error: {e}")
                return False, np.array([]), None, None
            
        elif self.cap and self.cap.isOpened():
            ret, color_image = self.cap.read()
            if not ret:
                return False, np.array([]), None, None
            return True, color_image, None, None
        else:
            # Generate synthetic test frame with ArUco marker pattern for offline testing
            synthetic_bg = np.zeros((self.height, self.width, 3), dtype=np.uint8)
            synthetic_bg[:] = (40, 40, 40)
            cv2.putText(synthetic_bg, "NO CAMERA CONNECTED - OFFLINE SIMULATION", (30, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            return True, synthetic_bg, None, None

    def get_depth_at_pixel(self, depth_frame: Any, x: int, y: int) -> float:
        """
        Query physical distance (in meters) from depth sensor at image coordinate (x, y).
        
        Args:
            depth_frame: rs.depth_frame object
            x: Pixel column index
            y: Pixel row index
            
        Returns:
            Distance in meters (0.0 if invalid)
        """
        if depth_frame and self.is_rs_active:
            try:
                # Clamp coordinates within frame dimensions
                x_clamped = max(0, min(self.width - 1, int(x)))
                y_clamped = max(0, min(self.height - 1, int(y)))
                distance = depth_frame.get_distance(x_clamped, y_clamped)
                return float(distance)
            except Exception:
                return 0.0
        return 0.0

    def depth_to_meters(self, depth_frame: Any) -> Optional[np.ndarray]:
        """Convert a RealSense depth frame/raw depth image to float meters."""
        if depth_frame is None:
            return None
        try:
            data = np.asanyarray(depth_frame.get_data()) if hasattr(depth_frame, "get_data") else np.asarray(depth_frame)
            if data.size == 0:
                return None
            if np.issubdtype(data.dtype, np.integer):
                return data.astype(np.float32) * float(self.depth_scale_m)
            return data.astype(np.float32, copy=False)
        except Exception:
            return None

    def stop(self):
        """Release camera resources."""
        if self.is_rs_active and self.pipeline:
            try:
                self.pipeline.stop()
                print("[RealSenseCamera] RealSense pipeline stopped.")
            except Exception:
                pass
        if self.cap and self.cap.isOpened():
            self.cap.release()
            print("[RealSenseCamera] OpenCV VideoCapture released.")
