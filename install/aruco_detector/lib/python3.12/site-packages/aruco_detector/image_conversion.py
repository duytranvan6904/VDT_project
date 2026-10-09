from typing import Optional

import cv2
import numpy as np

CHANNELS_BY_ENCODING = {
    "mono8": 1,
    "8uc1": 1,
    "rgb8": 3,
    "bgr8": 3,
    "rgba8": 4,
    "bgra8": 4,
}

COLOR_CONVERSION_BY_ENCODING = {
    "rgb8": cv2.COLOR_RGB2BGR,
    "rgba8": cv2.COLOR_RGBA2BGR,
    "bgra8": cv2.COLOR_BGRA2BGR,
}

ENCODINGS_16BIT = {"mono16", "16uc1"}

def extract_pixel_rows(msg, channels: int) -> Optional[np.ndarray]:
    buffer = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.step <= 0 or buffer.size < msg.step * msg.height:
        return None
    rows = buffer[: msg.step * msg.height].reshape((msg.height, msg.step))
    return rows[:, : msg.width * channels]


def shape_pixels(rows: np.ndarray, msg, channels: int) -> np.ndarray:
    if channels == 1:
        return rows
    return rows.reshape((msg.height, msg.width, channels))


def convert_to_bgr_if_needed(pixels: np.ndarray, encoding: str) -> np.ndarray:
    conversion = COLOR_CONVERSION_BY_ENCODING.get(encoding)
    if conversion is None:
        return pixels
    return cv2.cvtColor(pixels, conversion)

def mono16_to_mono8(msg) -> Optional[np.ndarray]:
    buffer = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.step <= 0 or buffer.size < msg.step * msg.height:
        return None
    rows = buffer[: msg.step * msg.height].reshape((msg.height, msg.step))
    rows = np.ascontiguousarray(rows[:, : msg.width * 2])
    dtype = np.dtype(">u2") if msg.is_bigendian else np.dtype("<u2")
    pixels = rows.view(dtype).reshape((msg.height, msg.width))
    return (pixels >> 8).astype(np.uint8)

def ros_image_to_array(msg) -> Optional[np.ndarray]:
    encoding = msg.encoding.lower()
    if encoding in ENCODINGS_16BIT:
        return mono16_to_mono8(msg)
    channels = CHANNELS_BY_ENCODING.get(encoding)
    channels = CHANNELS_BY_ENCODING.get(encoding)
    if channels is None:
        return None
    rows = extract_pixel_rows(msg, channels)
    if rows is None:
        return None
    pixels = shape_pixels(rows, msg, channels)
    return convert_to_bgr_if_needed(pixels, encoding).copy()
