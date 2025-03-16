from droneops.droneops.droneops import DroneOps
import unittest

class TestDroneOps(unittest.TestCase):

    def setUp(self):
        self.drone = DroneOps(connection_string='udp:127.0.0.1:14550')

    def test_initialization(self):
        self.assertIsNotNone(self.drone.vehicle)
        self.assertEqual(self.drone.takeoff_altitude, 10)
        self.assertEqual(self.drone.std_speed, 15)

    def test_set_mode(self):
        self.drone.set_mode("GUIDED")
        self.assertEqual(self.drone.vehicle.mode.name, "GUIDED")

    def test_arm_and_takeoff(self):
        self.drone.arm_and_takeoff(10)
        self.assertTrue(self.drone.vehicle.armed)

    def test_goto(self):
        self.drone.goto(34.0, -117.0, altitude=10)
        # Assuming the vehicle has a method to check if it reached the destination
        self.assertTrue(self.drone.vehicle.location.global_relative_frame.lat, 34.0)
        self.assertTrue(self.drone.vehicle.location.global_relative_frame.lon, -117.0)

    def test_land(self):
        self.drone.land()
        self.assertEqual(self.drone.vehicle.mode.name, "LAND")

    def test_return_to_home(self):
        self.drone.return_to_home()
        # Assuming the vehicle has a method to check if it returned home
        self.assertEqual(self.drone.vehicle.location.global_relative_frame.lat, self.drone.vehicle.home_location.lat)
        self.assertEqual(self.drone.vehicle.location.global_relative_frame.lon, self.drone.vehicle.home_location.lon)

    def tearDown(self):
        self.drone.__del__()

if __name__ == '__main__':
    unittest.main()