import unittest
from droneops.q_ground_control import QGroundControlUtils

class TestQGroundControlUtils(unittest.TestCase):

    def setUp(self):
        self.qgc_utils = QGroundControlUtils()

    def test_create_qgc_mission(self):
        waypoints = [(34.0, -118.0, 100), (34.1, -118.1, 100)]
        mission_file = self.qgc_utils.create_qgc_mission(waypoints, "test_mission.plan")
        self.assertTrue(mission_file.endswith(".plan"))

    def test_convert_qgc_to_droneops_mission(self):
        # Assuming a valid QGC plan file exists for testing
        qgc_plan_path = "test_mission.plan"
        waypoints = self.qgc_utils.convert_qgc_to_droneops_mission(qgc_plan_path)
        self.assertIsInstance(waypoints, list)
        self.assertGreater(len(waypoints), 0)

    def test_create_survey_grid(self):
        boundary_points = [(34.0, -118.0), (34.0, -118.1), (34.1, -118.1), (34.1, -118.0)]
        altitude = 100
        waypoints = self.qgc_utils.create_survey_grid(boundary_points, altitude, spacing=10)
        self.assertIsInstance(waypoints, list)
        self.assertGreater(len(waypoints), 0)

    def test_create_circular_pattern(self):
        center_lat = 34.0
        center_lon = -118.0
        radius = 50
        altitude = 100
        waypoints = self.qgc_utils.create_circular_pattern(center_lat, center_lon, radius, altitude)
        self.assertIsInstance(waypoints, list)
        self.assertGreater(len(waypoints), 0)

    def test_calculate_mission_stats(self):
        waypoints = [(34.0, -118.0, 100), (34.1, -118.1, 100)]
        stats = self.qgc_utils.calculate_mission_stats(waypoints)
        self.assertIn('total_distance', stats)
        self.assertIn('waypoint_count', stats)
        self.assertIn('estimated_flight_time', stats)

if __name__ == '__main__':
    unittest.main()