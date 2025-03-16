from droneops.mission_planner import MissionPlanner
import unittest
import json
import os

class TestMissionPlanner(unittest.TestCase):

    def setUp(self):
        self.waypoints = [
            (34.0001, -117.0001, 10),
            (34.0002, -117.0002, 10),
            (34.0003, -117.0003, 10)
        ]
        self.filename = "test_mission.json"

    def tearDown(self):
        if os.path.exists(self.filename):
            os.remove(self.filename)

    def test_create_mission_file(self):
        result = MissionPlanner.create_mission_file(self.waypoints, self.filename)
        self.assertEqual(result, self.filename)

        with open(self.filename, 'r') as f:
            mission_data = json.load(f)

        self.assertEqual(mission_data["fileType"], "Plan")
        self.assertEqual(len(mission_data["mission"]["items"]), 4)  # Takeoff + 2 waypoints + RTL

    def test_create_survey_grid(self):
        boundary_points = [(34.0001, -117.0001), (34.0001, -117.0002), (34.0002, -117.0002), (34.0002, -117.0001)]
        altitude = 10
        waypoints = MissionPlanner.create_survey_grid(boundary_points, altitude)

        self.assertGreater(len(waypoints), 0)
        for wp in waypoints:
            self.assertEqual(wp[2], altitude)

    def test_create_perimeter_scan(self):
        boundary_points = [(34.0001, -117.0001), (34.0001, -117.0002), (34.0002, -117.0002), (34.0002, -117.0001)]
        altitude = 10
        waypoints = MissionPlanner.create_perimeter_scan(boundary_points, altitude)

        self.assertEqual(len(waypoints), len(boundary_points) + 1)  # Plus one to close the loop
        for wp in waypoints:
            self.assertEqual(wp[2], altitude)

    def test_calculate_mission_stats(self):
        waypoints = [
            (34.0001, -117.0001, 10),
            (34.0002, -117.0002, 10),
            (34.0003, -117.0003, 20)
        ]
        stats = MissionPlanner.calculate_mission_stats(waypoints)

        self.assertIn('total_distance', stats)
        self.assertIn('waypoint_count', stats)
        self.assertIn('estimated_flight_time', stats)
        self.assertIn('max_altitude', stats)
        self.assertIn('min_altitude', stats)

if __name__ == '__main__':
    unittest.main()