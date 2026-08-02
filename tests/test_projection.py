import unittest

from hub.serve.projection import FixedView, project_point


class ProjectionTests(unittest.TestCase):
    def setUp(self) -> None:
        self.view = FixedView(
            position_m=(1.8, -2.0, 1.65),
            yaw_deg=0.0,
            pitch_deg=0.0,
            horizontal_fov_deg=50.0,
        )

    def test_point_straight_ahead_projects_to_center(self) -> None:
        projected = project_point((1.8, 2.0, 1.65), self.view, (1920, 1080))
        self.assertIsNotNone(projected)
        self.assertAlmostEqual(projected[0], 960.0)
        self.assertAlmostEqual(projected[1], 540.0)

    def test_positive_wall_x_projects_right(self) -> None:
        center = project_point((1.8, 2.0, 1.65), self.view, (1920, 1080))
        right = project_point((2.8, 2.0, 1.65), self.view, (1920, 1080))
        self.assertGreater(right[0], center[0])

    def test_more_depth_reduces_lateral_displacement(self) -> None:
        near = project_point((2.8, 0.0, 1.65), self.view, (1920, 1080))
        far = project_point((2.8, 4.0, 1.65), self.view, (1920, 1080))
        self.assertGreater(abs(near[0] - 960), abs(far[0] - 960))

    def test_point_behind_viewer_is_not_projected(self) -> None:
        self.assertIsNone(
            project_point((1.8, -3.0, 1.65), self.view, (1920, 1080))
        )


if __name__ == "__main__":
    unittest.main()
