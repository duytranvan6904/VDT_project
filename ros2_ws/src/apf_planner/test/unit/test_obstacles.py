import numpy as np
import pytest

from apf_planner.obstacles import (
    cloud_to_xyz,
    cylinder_nearest_point,
    load_cylinder_obstacles_from_sdf,
    select_obstacle_points,
    voxel_downsample,
)
from ros_utils import make_cloud
from sensor_msgs.msg import PointCloud2, PointField


def write_sdf(path, models):
    body = ''
    for name, pose, cyl in models:
        pose_xml = f'<pose>{pose}</pose>' if pose is not None else ''
        body += f'<model name="{name}">{pose_xml}<link><collision><geometry>{cyl}</geometry></collision></link></model>'
    path.write_text(f'<sdf><world name="w">{body}</world></sdf>')
    return str(path)


def cyl(r, h):
    return f'<cylinder><radius>{r}</radius><length>{h}</length></cylinder>'


def test_sdf_missing_file(tmp_path):
    assert load_cylinder_obstacles_from_sdf(str(tmp_path / 'none.sdf')) == []


def test_sdf_parses_prefixed_cylinders(tmp_path):
    path = write_sdf(tmp_path / 'w.sdf', [
        ('cyl_1', '1 2 1.5 0 0 0', cyl(0.4, 3.0)),
        ('cylinder_obs_2', '-3 4 2.0 0 0 0', cyl(0.6, 4.0)),
        ('box_1', '0 0 0 0 0 0', '<box><size>1 1 1</size></box>'),
        ('tree', '5 5 1 0 0 0', cyl(0.3, 2.0)),
    ])
    out = load_cylinder_obstacles_from_sdf(path)
    assert out == [(1.0, 2.0, 0.4, 3.0, 1.5), (-3.0, 4.0, 0.6, 4.0, 2.0)]


def test_sdf_default_radius_and_length(tmp_path):
    path = write_sdf(tmp_path / 'w.sdf', [('cyl_a', '0 0 0.5 0 0 0', '<cylinder/>')])
    assert load_cylinder_obstacles_from_sdf(path) == [(0.0, 0.0, 0.5, 1.0, 0.5)]


def test_sdf_skips_models_without_pose_or_cylinder(tmp_path):
    path = write_sdf(tmp_path / 'w.sdf', [
        ('cyl_nopose', None, cyl(0.4, 3.0)),
        ('cyl_nogeom', '1 1 1 0 0 0', '<box><size>1 1 1</size></box>'),
    ])
    assert load_cylinder_obstacles_from_sdf(path) == []


def test_nearest_point_horizontal():
    p = cylinder_nearest_point(np.array([5.0, 0.0, 1.0]), 0.0, 0.0, 1.0, 4.0, 2.0)
    assert p == pytest.approx([1.0, 0.0, 1.0])


def test_nearest_point_clips_z_above_and_below():
    top = cylinder_nearest_point(np.array([5.0, 0.0, 10.0]), 0.0, 0.0, 1.0, 4.0, 2.0)
    bottom = cylinder_nearest_point(np.array([5.0, 0.0, -3.0]), 0.0, 0.0, 1.0, 4.0, 2.0)
    assert top[2] == pytest.approx(4.0)
    assert bottom[2] == pytest.approx(0.0)


def test_nearest_point_on_axis_is_defined():
    p = cylinder_nearest_point(np.array([0.0, 0.0, 1.0]), 0.0, 0.0, 1.0, 4.0, 2.0)
    assert p == pytest.approx([1.0, 0.0, 1.0])


def test_nearest_point_offset_axis_diagonal():
    p = cylinder_nearest_point(np.array([4.0, 4.0, 2.0]), 1.0, 1.0, 1.0, 4.0, 2.0)
    assert p[:2] == pytest.approx([1.0 + np.sqrt(0.5), 1.0 + np.sqrt(0.5)])


def test_cloud_roundtrip():
    pts = np.array([[1, 2, 3], [4, 5, 6]], dtype=np.float32)
    assert cloud_to_xyz(make_cloud(pts)) == pytest.approx(pts)


def test_cloud_filters_non_finite():
    pts = np.array([[1, 2, 3], [np.nan, 0, 0], [0, np.inf, 0], [7, 8, 9]], dtype=np.float32)
    assert cloud_to_xyz(make_cloud(pts)) == pytest.approx(np.array([[1, 2, 3], [7, 8, 9]]))


def test_cloud_empty():
    assert cloud_to_xyz(make_cloud(np.empty((0, 3)))).shape == (0, 3)


def test_cloud_missing_field_returns_empty():
    m = make_cloud([[1, 2, 3]])
    m.fields = m.fields[:2]
    assert cloud_to_xyz(m).shape == (0, 3)


def test_cloud_short_buffer_returns_empty():
    m = make_cloud([[1, 2, 3], [4, 5, 6]])
    m.data = m.data[:-4]
    assert cloud_to_xyz(m).shape == (0, 3)


def test_cloud_with_extra_field_and_padding():
    arr = np.zeros(2, dtype=[('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('i', '<f4')])
    arr['x'], arr['y'], arr['z'], arr['i'] = [1, 4], [2, 5], [3, 6], [9, 9]
    m = PointCloud2()
    m.height, m.width = 1, 2
    m.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                for i, n in enumerate('xyzi')]
    m.point_step, m.row_step = 16, 32
    m.data = arr.tobytes()
    assert cloud_to_xyz(m) == pytest.approx(np.array([[1, 2, 3], [4, 5, 6]]))


def test_cloud_big_endian():
    pts = np.array([[1, 2, 3]], dtype='>f4')
    m = make_cloud(np.zeros((1, 3)))
    m.is_bigendian = True
    m.data = pts.tobytes()
    assert cloud_to_xyz(m) == pytest.approx(np.array([[1, 2, 3]]))


def test_voxel_passthrough_cases():
    xyz = np.array([[0.0, 0, 0], [0.01, 0, 0]])
    assert voxel_downsample(xyz, 0.0) is xyz
    assert len(voxel_downsample(np.empty((0, 3)), 0.3)) == 0


def test_voxel_merges_same_cell_keeps_distinct():
    xyz = np.array([[0.05, 0.05, 0.05], [0.1, 0.1, 0.1], [1.0, 1.0, 1.0]])
    out = voxel_downsample(xyz, 0.3)
    assert len(out) == 2


def test_voxel_handles_negative_coordinates():
    xyz = np.array([[-0.05, 0, 0], [0.05, 0, 0]])
    assert len(voxel_downsample(xyz, 0.3)) == 2


def test_select_empty_cloud():
    assert select_obstacle_points(np.empty((0, 3)), np.zeros(3), 5.0, 0.5, 8) == []


def test_select_radius_cluster_and_order():
    cloud = np.array([[1.0, 0, 0], [1.1, 0, 0], [0, 3.0, 0], [10.0, 0, 0]])
    out = select_obstacle_points(cloud, np.zeros(3), 5.0, 0.5, 8)
    assert out == [(1.0, 0.0, 0.0), (0.0, 3.0, 0.0)]


def test_select_max_points():
    cloud = np.array([[i * 2.0, 0, 0] for i in range(1, 6)])
    out = select_obstacle_points(cloud, np.zeros(3), 20.0, 0.5, 3)
    assert out == [(2.0, 0.0, 0.0), (4.0, 0.0, 0.0), (6.0, 0.0, 0.0)]


def test_select_nothing_in_radius():
    cloud = np.array([[10.0, 0, 0]])
    assert select_obstacle_points(cloud, np.zeros(3), 5.0, 0.5, 8) == []


def test_select_uses_3d_distance():
    cloud = np.array([[0.0, 0.0, 4.0], [3.0, 0.0, 0.0]])
    out = select_obstacle_points(cloud, np.zeros(3), 3.5, 0.5, 8)
    assert out == [(3.0, 0.0, 0.0)]