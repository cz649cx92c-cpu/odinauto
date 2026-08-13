import math

from lidar_preview import preview_points, retained_preview_points


def test_standard_transform_and_offsets():
    points = preview_points([1.0], math.pi, 0.1, 0.02, 12.0)
    expected_x = math.cos(math.radians(357.0))
    expected_y = math.sin(math.radians(357.0)) + 0.035
    assert points == [(round(expected_x, 4), round(expected_y, 4))]


def test_standard_range_and_view_crop():
    values = [0.04, 0.50, float('inf'), float('nan'), 7.0]
    points = preview_points(values, math.pi, 0.0, 0.01, 20.0)
    assert len(points) == 1


def test_standard_downsample_limit():
    points = preview_points([1.0] * 1441, math.pi, 0.0, 0.05, 6.0)
    assert 0 < len(points) <= 720


def test_stale_preview_retention_is_bounded():
    points = [{'x': 1.0, 'y': 0.0}]
    assert retained_preview_points(points, 0.25) is points
    assert retained_preview_points(points, 1.99) is points
    assert retained_preview_points(points, 2.0) == []
    assert retained_preview_points(points, None) == []


if __name__ == '__main__':
    test_standard_transform_and_offsets()
    test_standard_range_and_view_crop()
    test_standard_downsample_limit()
    test_stale_preview_retention_is_bounded()
    print('lidar preview tests: PASS')
