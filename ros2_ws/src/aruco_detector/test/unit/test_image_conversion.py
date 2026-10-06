from types import SimpleNamespace

import numpy as np
import pytest

from aruco_detector.image_conversion import ros_image_to_array


def msg(data, width, height, encoding, step=None, big=False):
    raw = bytes(data) if not isinstance(data, bytes) else data
    return SimpleNamespace(
        data=raw, width=width, height=height, encoding=encoding,
        step=len(raw) // height if step is None else step, is_bigendian=big)


def test_mono8_roundtrip():
    arr = np.arange(12, dtype=np.uint8).reshape(3, 4)
    out = ros_image_to_array(msg(arr.tobytes(), 4, 3, 'mono8'))
    assert np.array_equal(out, arr)
    assert out.flags.writeable


def test_8uc1_alias_and_case_insensitive():
    arr = np.arange(6, dtype=np.uint8).reshape(2, 3)
    assert np.array_equal(ros_image_to_array(msg(arr.tobytes(), 3, 2, '8UC1')), arr)
    assert np.array_equal(ros_image_to_array(msg(arr.tobytes(), 3, 2, 'MONO8')), arr)


def test_row_padding_is_trimmed():
    rows = np.arange(18, dtype=np.uint8).reshape(3, 6)
    out = ros_image_to_array(msg(rows.tobytes(), 4, 3, 'mono8', step=6))
    assert out.shape == (3, 4)
    assert np.array_equal(out, rows[:, :4])


def test_rgb_to_bgr_swap():
    pixel = np.array([[[255, 10, 0]]], dtype=np.uint8)
    out = ros_image_to_array(msg(pixel.tobytes(), 1, 1, 'rgb8'))
    assert out.shape == (1, 1, 3)
    assert out[0, 0].tolist() == [0, 10, 255]


def test_bgr8_unchanged():
    pixel = np.array([[[1, 2, 3]]], dtype=np.uint8)
    out = ros_image_to_array(msg(pixel.tobytes(), 1, 1, 'bgr8'))
    assert out[0, 0].tolist() == [1, 2, 3]


def test_rgba_and_bgra_drop_alpha():
    rgba = np.array([[[255, 10, 0, 128]]], dtype=np.uint8)
    bgra = np.array([[[0, 10, 255, 128]]], dtype=np.uint8)
    out1 = ros_image_to_array(msg(rgba.tobytes(), 1, 1, 'rgba8'))
    out2 = ros_image_to_array(msg(bgra.tobytes(), 1, 1, 'bgra8'))
    assert out1.shape == (1, 1, 3) and out2.shape == (1, 1, 3)
    assert out1[0, 0].tolist() == [0, 10, 255]
    assert out2[0, 0].tolist() == [0, 10, 255]


def test_mono16_little_endian_high_byte():
    arr = np.array([[0x1234, 0xFF00], [0x00FF, 0x8000]], dtype='<u2')
    out = ros_image_to_array(msg(arr.tobytes(), 2, 2, 'mono16'))
    assert out.dtype == np.uint8
    assert out.tolist() == [[0x12, 0xFF], [0x00, 0x80]]


def test_mono16_big_endian():
    arr = np.array([[0x1234, 0xFF00]], dtype='>u2')
    out = ros_image_to_array(msg(arr.tobytes(), 2, 1, 'mono16', big=True))
    assert out.tolist() == [[0x12, 0xFF]]


def test_16uc1_alias():
    arr = np.array([[0x4000]], dtype='<u2')
    assert ros_image_to_array(msg(arr.tobytes(), 1, 1, '16UC1')).tolist() == [[0x40]]


def test_mono16_row_padding():
    rows = np.zeros((2, 3), dtype='<u2')
    rows[:, :2] = [[0x0100, 0x0200], [0x0300, 0x0400]]
    out = ros_image_to_array(msg(rows.tobytes(), 2, 2, 'mono16', step=6))
    assert out.tolist() == [[1, 2], [3, 4]]


@pytest.mark.parametrize('encoding', ['32FC1', 'bayer_rggb8', 'yuv422', ''])
def test_unsupported_encoding_returns_none(encoding):
    assert ros_image_to_array(msg(bytes(16), 4, 4, encoding, step=4)) is None


def test_short_buffer_returns_none():
    assert ros_image_to_array(msg(bytes(5), 4, 3, 'mono8', step=4)) is None
    assert ros_image_to_array(msg(bytes(5), 4, 3, 'mono16', step=8)) is None


def test_non_positive_step_returns_none():
    assert ros_image_to_array(msg(bytes(12), 4, 3, 'mono8', step=0)) is None
    assert ros_image_to_array(msg(bytes(12), 4, 3, 'mono16', step=0)) is None


def test_output_independent_of_source_buffer():
    raw = bytearray(12)
    m = msg(bytes(raw), 4, 3, 'mono8')
    out = ros_image_to_array(m)
    out[0, 0] = 99
    assert m.data[0] == 0