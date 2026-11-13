import pytest

from mortar_pestle.sim.app import intersect_plane_z, mouse_ray

# A camera 1 m above the origin that looks straight down, in the layout that
# pybullet.getDebugVisualizerCamera returns. Screen right is world +x, screen up is world +y.
FAR = 10_000.0
CAMERA = (
    800,  # width in pixels
    600,  # height in pixels
    None,  # view matrix (unused)
    None,  # projection matrix (unused)
    (0.0, 1.0, 0.0),  # camera up
    (0.0, 0.0, -1.0),  # camera forward
    (0.8 * FAR, 0.0, 0.0),  # horizontal extent of the view at the far plane
    (0.0, 0.6 * FAR, 0.0),  # vertical extent of the view at the far plane
    0.0,
    -90.0,
    1.0,  # distance from camera to target
    (0.0, 0.0, 0.0),  # target
)


def test_centre_pixel_hits_the_camera_target() -> None:
    origin, direction = mouse_ray(CAMERA, 400, 300)

    assert origin == pytest.approx((0.0, 0.0, 1.0))
    assert intersect_plane_z(origin, direction, 0.0) == pytest.approx((0.0, 0.0))


def test_pixels_right_of_and_below_centre_hit_right_and_below_on_the_floor() -> None:
    origin, direction = mouse_ray(CAMERA, 800, 600)

    # The view is 0.8 m wide and 0.6 m tall at 1 m distance.
    assert intersect_plane_z(origin, direction, 0.0) == pytest.approx((0.4, -0.3))


def test_higher_plane_is_hit_closer_to_the_camera_axis() -> None:
    origin, direction = mouse_ray(CAMERA, 800, 300)

    assert intersect_plane_z(origin, direction, 0.5) == pytest.approx((0.2, 0.0))


def test_ray_parallel_to_the_plane_never_hits() -> None:
    assert intersect_plane_z((0.0, 0.0, 1.0), (1.0, 0.0, 0.0), 0.0) is None


def test_plane_behind_the_camera_never_hits() -> None:
    assert intersect_plane_z((0.0, 0.0, 1.0), (0.0, 0.0, -1.0), 2.0) is None
