"""
DroneOps - Advanced Drone Control and Mission Planning Library

A comprehensive library for autonomous drone operations, mission planning,
and intelligent landing capabilities.
"""

import os
import json
import time
import threading
import numpy as np
from shapely.geometry import Polygon, Point
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command
from pymavlink import mavutil

class DroneOps:
	"""Main drone operations class that encapsulates drone control functionality."""
	
	def __init__(self, connection_string, baud_rate=57600):
		"""
		Initialize drone connection and setup basic parameters.
		
		Args:
			connection_string (str): Connection string for the drone (e.g., 'udp:127.0.0.1:14550')
			baud_rate (int): Baud rate for serial connections
		"""
		self.vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
		self.takeoff_altitude = 10  # meters
		self.std_speed = 15  # m/s
		self.close_enough_dist = 1  # meters
		self.base_coords_file = "home_coords.json"
		self.lidar_file = "lidar_data.log"
		self.mission_in_progress = False
		self.return_mission_event = threading.Event()
		
	def __del__(self):
		"""Safely disconnect when object is destroyed."""
		if hasattr(self, 'vehicle') and self.vehicle:
			self.vehicle.close()
			
	def set_mode(self, mode_name, timeout=10):
		"""
		Set the drone flight mode.
		
		Args:
			mode_name (str): Mode to set ('GUIDED', 'AUTO', 'RTL', 'LAND', etc.)
			timeout (int): Maximum time to wait for mode change
			
		Raises:
			TimeoutError: If mode cannot be set within the timeout period
		"""
		self.vehicle.mode = VehicleMode(mode_name)
		start_time = time.time()
		while not self.vehicle.mode.name == mode_name and time.time() - start_time < timeout:
			time.sleep(1)
		if not self.vehicle.mode.name == mode_name:
			raise TimeoutError(f"Failed to set vehicle mode to {mode_name}")
	
	def arm_and_takeoff(self, altitude=None):
		"""
		Arm the drone and take off to the specified altitude.
		
		Args:
			altitude (float, optional): Target altitude in meters. If None, uses default takeoff altitude.
		"""
		if altitude is None:
			altitude = self.takeoff_altitude
			
		# Set mode to GUIDED
		self.set_mode("GUIDED")
		
		# Arm the drone
		self.vehicle.armed = True
		while not self.vehicle.armed:
			time.sleep(1)
			
		# Take off
		self.vehicle.simple_takeoff(altitude)
		
		# Wait for drone to reach target altitude
		while True:
			current_alt = self.vehicle.location.global_relative_frame.alt
			if current_alt >= altitude * 0.95:  # 95% of target altitude
				break
			time.sleep(1)
			
	def goto(self, latitude, longitude, altitude=None, groundspeed=None):
		"""
		Navigate to a specified location.
		
		Args:
			latitude (float): Target latitude
			longitude (float): Target longitude
			altitude (float, optional): Target altitude in meters. If None, uses default altitude.
			groundspeed (float, optional): Speed in m/s. If None, uses default speed.
			
		Returns:
			bool: True if destination reached, False otherwise
		"""
		if altitude is None:
			altitude = self.takeoff_altitude
			
		if groundspeed is None:
			groundspeed = self.std_speed
			
		# If drone is not armed, arm and takeoff first
		if not self.vehicle.armed:
			self.arm_and_takeoff(altitude)
			
		# Navigate to the target location
		target_location = LocationGlobalRelative(latitude, longitude, altitude)
		self.vehicle.simple_goto(target_location, groundspeed=groundspeed)
		
		# Monitor progress
		while self.vehicle.mode.name == "GUIDED":
			current_location = self.vehicle.location.global_frame
			distance = self.get_distance_metres(current_location, target_location)
			
			if distance < self.close_enough_dist:
				return True
				
			time.sleep(1)
			
		return False
	
	def land(self):
		"""Land the drone at the current location."""
		self.set_mode("LAND")
		while self.vehicle.armed:
			time.sleep(1)
	
	def return_to_home(self):
		"""Return to the home location and land."""
		home_coords = self.load_base_coordinates()
		if home_coords:
			self.goto(home_coords['latitude'], home_coords['longitude'], self.takeoff_altitude, self.std_speed)
			self.land()
		else:
			print("Error: Home coordinates not available.")
	
	def upload_mission(self, waypoints):
		"""
		Upload a mission to the drone.
		
		Args:
			waypoints (list): List of (lat, lon, alt) tuples defining waypoints
		"""
		cmds = self.vehicle.commands
		cmds.clear()
		
		# Add takeoff command
		cmds.add(Command(
			0, 0, 0,
			mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
			mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
			0, 0, 0, 0, 0, 0,
			0, 0, self.takeoff_altitude))
		
		# Add waypoints
		for lat, lon, alt in waypoints:
			cmds.add(Command(
				0, 0, 0,
				mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
				mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
				0, 0, 0, 0, 0, 0,
				lat, lon, alt))
		
		# Upload mission
		cmds.upload()
	
	def run_route(self, waypoints):
		"""
		Execute a mission consisting of multiple waypoints.
		
		Args:
			waypoints (list): List of (lat, lon, alt) tuples defining the route
		"""
		self.arm_and_takeoff(self.takeoff_altitude)
		self.upload_mission(waypoints)
		
		# Switch to AUTO mode to execute the mission
		self.set_mode("AUTO", 15)
		
		# Wait for mission to complete
		while self.vehicle.commands.next < len(waypoints):
			time.sleep(1)
		
		# Pause briefly before landing
		time.sleep(0.5)
		self.land()
	
	def delivery_mission(self, latitude, longitude):
		"""
		Execute a simple delivery mission.
		
		Args:
			latitude (float): Delivery destination latitude
			longitude (float): Delivery destination longitude
		"""
		self.goto(latitude, longitude, self.takeoff_altitude, self.std_speed)
		self.land()
		time.sleep(5)  # Wait for delivery/pickup
		self.return_to_home()
	
	def execute_mission(self, plan_file, return_plan=None):
		"""
		Execute a complete mission with optional return route.
		
		Args:
			plan_file (str): Path to mission plan file
			return_plan (str, optional): Path to return plan file. If None, reverses the outbound path.
		"""
		# Load waypoints from plan file
		waypoints = self.load_plan_file(plan_file)
		
		# Add final waypoint again to ensure drone reaches the destination
		waypoints.append(waypoints[-1])
		
		# Get return mission
		if return_plan:
			return_waypoints = self.load_plan_file(return_plan)
			return_waypoints.append(return_waypoints[-1])
		else:
			# Reverse the outbound path for return, skipping the first waypoint
			return_waypoints = list(reversed(waypoints[1:]))
			
			# Add home location
			home_coords = self.load_base_coordinates()
			if home_coords:
				home_coords_tuple = (home_coords['latitude'], home_coords['longitude'], waypoints[-1][2])
				return_waypoints.append(home_coords_tuple)
				return_waypoints.append(home_coords_tuple)  # Add twice to ensure drone reaches home
		
		# Start mission logging
		self.mission_in_progress = True
		log_thread = threading.Thread(target=self._log_vehicle_status)
		log_thread.daemon = True
		log_thread.start()
		
		# Execute outbound mission
		self.run_route(waypoints)
		
		# Wait for return confirmation
		self.return_mission_event.clear()
		print("Waiting for confirmation to start return route...")
		self.return_mission_event.wait()
		
		# Execute return mission
		self.run_route(return_waypoints)
		
		# Stop mission logging
		self.mission_in_progress = False
	
	def confirm_return(self):
		"""Signal confirmation to start the return mission."""
		self.return_mission_event.set()
	
	def find_landing_spot(self, radius=50, grid_resolution=5):
		"""
		Use LiDAR data to find a suitable landing spot near the current location.
		
		Args:
			radius (float): Search radius in meters
			grid_resolution (float): Grid resolution in meters
			
		Returns:
			tuple: (latitude, longitude, altitude) of best landing spot, or None if not found
		"""
		# Get current location
		current_location = self.vehicle.location.global_relative_frame
		
		# Define search area
		lat_meters = 111320  # Approximate meters per degree of latitude
		lon_meters = 111320 * np.cos(np.radians(current_location.lat))  # Meters per degree of longitude
		
		lat_delta = radius / lat_meters
		lon_delta = radius / lon_meters
		
		boundary_coords = [
			(current_location.lat - lat_delta, current_location.lon - lon_delta),
			(current_location.lat + lat_delta, current_location.lon - lon_delta),
			(current_location.lat + lat_delta, current_location.lon + lon_delta),
			(current_location.lat - lat_delta, current_location.lon + lon_delta),
		]
		
		# Create grid of waypoints
		waypoints = self.create_grid_within_polygon(boundary_coords, grid_resolution/lat_meters, current_location.alt)
		
		# Collect LiDAR data
		lidar_file = self.collect_lidar_data(waypoints)
		
		# Find best landing spot
		return self.find_best_landing_spot(lidar_file)
	
	def emergency_land(self):
		"""
		Find a safe spot and land when battery is low or in emergency situations.
		
		Returns:
			bool: True if landing successful, False otherwise
		"""
		# Check if battery level is critically low
		if self.vehicle.battery.level < 15:  # 15% battery remaining
			print("Emergency: Battery critically low. Finding safe landing zone...")
			
			# Find a suitable landing spot
			best_spot = self.find_landing_spot(radius=30, grid_resolution=2)
			
			if best_spot:
				lat, lon, alt = best_spot
				print(f"Landing at optimal location: {lat}, {lon}")
				self.goto(lat, lon, alt, 5)  # Slowly approach landing site
				self.land()
				return True
			else:
				print("No suitable landing spot found. Attempting to land at current location.")
				self.land()
				return True
		return False
	
	def collect_lidar_data(self, waypoints):
		"""
		Collect LiDAR data at specified waypoints.
		
		Args:
			waypoints (list): List of dict with keys 'latitude', 'longitude', 'altitude'
			
		Returns:
			str: Path to LiDAR data file
		"""
		with open(self.lidar_file, 'w') as log_file:
			log_file.write("Latitude,Longitude,Altitude,LiDAR Distance (m)\n")
			
			for waypoint in waypoints:
				latitude = waypoint['latitude']
				longitude = waypoint['longitude']
				altitude = waypoint['altitude']
				
				# Fly to the waypoint
				self.goto(latitude, longitude, altitude, 5)  # Lower speed for accurate sensing
				
				# Get LiDAR reading
				lidar_data = self._simulate_lidar()  # Replace with actual LiDAR reading implementation
				
				if lidar_data is not None:
					log_file.write(f"{latitude},{longitude},{altitude},{lidar_data}\n")
				else:
					log_file.write(f"{latitude},{longitude},{altitude},No data\n")
					
		return self.lidar_file
	
	def _simulate_lidar(self):
		"""
		Simulate LiDAR readings. Replace this with actual LiDAR implementation.
		
		Returns:
			float: Simulated LiDAR distance in meters
		"""
		# This is a placeholder - in real implementation, connect to actual LiDAR sensor
		import random
		return random.uniform(0.5, 5.0)
	
	def _log_vehicle_status(self):
		"""Log vehicle status during missions."""
		while self.mission_in_progress:
			status = {
				'timestamp': time.time(),
				'lat': self.vehicle.location.global_frame.lat,
				'lon': self.vehicle.location.global_frame.lon,
				'alt': self.vehicle.location.global_frame.alt,
				'battery': self.vehicle.battery.level,
				'airspeed': self.vehicle.airspeed,
				'heading': self.vehicle.heading,
				'mode': self.vehicle.mode.name
			}
			
			# In a real implementation, this could insert into a database or write to a log file
			# For now, just print to console
			print(f"Status: {status}")
			time.sleep(5)
	
	# Utility methods
	def get_distance_metres(self, loc1, loc2):
		"""
		Calculate distance between two locations.
		
		Args:
			loc1: Location object with lat and lon attributes
			loc2: Location object with lat and lon attributes
			
		Returns:
			float: Distance in meters
		"""
		dlat = loc2.lat - loc1.lat
		dlong = loc2.lon - loc1.lon
		return ((dlat ** 2) + (dlong ** 2)) ** 0.5 * 111320
	
	def has_sufficient_battery(self, waypoints, margin=30):
		"""
		Check if battery is sufficient for planned route.
		
		Args:
			waypoints (list): List of locations
			margin (int): Safety margin percentage
			
		Returns:
			bool: True if sufficient battery, False otherwise
		"""
		# Calculate total distance
		distance = 0
		for i in range(len(waypoints) - 1):
			distance += self.get_distance_metres(waypoints[i], waypoints[i + 1])
			
		# Estimate battery needed (simplified model)
		battery_needed = (distance / 1000) * 10  # Assume 10% battery per km
		battery_needed += margin  # Add safety margin
		
		return self.vehicle.battery.level > battery_needed
	
	def create_grid_within_polygon(self, boundary_coords, grid_resolution, altitude):
		"""
		Create a grid of waypoints within a polygon boundary.
		
		Args:
			boundary_coords (list): List of (lat, lon) tuples defining the boundary
			grid_resolution (float): Grid spacing in degrees
			altitude (float): Altitude for all waypoints
			
		Returns:
			list: List of waypoint dictionaries with 'latitude', 'longitude', 'altitude' keys
		"""
		polygon = Polygon(boundary_coords)

		# Determine the bounding box for the polygon
		min_lat = min(coord[0] for coord in boundary_coords)
		max_lat = max(coord[0] for coord in boundary_coords)
		min_lon = min(coord[1] for coord in boundary_coords)
		max_lon = max(coord[1] for coord in boundary_coords)

		# Create a grid of points covering the bounding box
		lat_points = np.arange(min_lat, max_lat, grid_resolution)
		lon_points = np.arange(min_lon, max_lon, grid_resolution)

		# Check each grid point to see if it's inside the polygon
		waypoints = []
		for lat in lat_points:
			for lon in lon_points:
				point = Point(lat, lon)
				if polygon.contains(point):
					waypoints.append({'latitude': lat, 'longitude': lon, 'altitude': altitude})

		return waypoints
	
	def find_best_landing_spot(self, lidar_log_file):
		"""
		Analyze LiDAR data to find the flattest landing spot with minimal inclination.
		
		Args:
			lidar_log_file (str): Path to LiDAR log file
			
		Returns:
			tuple: (latitude, longitude, altitude) of best spot, or None if not found
		"""
		# First, load all valid readings into a dictionary for quick lookup
		readings = {}
		try:
			with open(lidar_log_file, 'r') as file:
				next(file)  # Skip header
				for line in file:
					try:
						lat, lon, alt, lidar_distance = line.strip().split(',')
						if lidar_distance != "No data":
							lat, lon = float(lat), float(lon)
							readings[(lat, lon)] = (float(alt), float(lidar_distance))
					except ValueError:
						continue

			# Find the spot with minimum inclination
			best_spot = None
			min_inclination = float('inf')
			
			for (lat, lon), (alt, distance) in readings.items():
				# Get readings from neighboring points (using small delta)
				delta = 0.00001  # Approximately 1m at equator
				neighbors = [
					readings.get((lat + delta, lon)),
					readings.get((lat - delta, lon)),
					readings.get((lat, lon + delta)),
					readings.get((lat, lon - delta))
				]
				
				# Calculate inclination as max height difference with neighbors
				max_inclination = 0
				valid_neighbors = 0
				
				for neighbor in neighbors:
					if neighbor is not None:
						neighbor_alt = neighbor[0] - neighbor[1]  # True ground height
						current_alt = alt - distance  # True ground height
						inclination = abs(neighbor_alt - current_alt)
						max_inclination = max(max_inclination, inclination)
						valid_neighbors += 1
				
				# Only consider points with at least 3 valid neighbors
				if valid_neighbors >= 3:
					if max_inclination < min_inclination:
						min_inclination = max_inclination
						best_spot = (lat, lon, alt)

			if best_spot:
				print(f"Best landing spot found at: {best_spot} with inclination: {min_inclination:.2f}m")
			else:
				print("No suitable landing spot found from LiDAR data")
				
			return best_spot
				
		except Exception as e:
			print(f"Error analyzing LiDAR data: {e}")
			return None
	
	def load_base_coordinates(self):
		"""
		Load home/base coordinates from file.
		
		Returns:
			dict: Home coordinates with 'latitude' and 'longitude' keys, or None if not found
		"""
		if os.path.exists(self.base_coords_file):
			try:
				with open(self.base_coords_file, 'r') as file:
					return json.load(file)
			except Exception as e:
				print(f"Error loading base coordinates: {e}")
				return None
		else:
			return None
	
	def save_base_coordinates(self):
		"""
		Save current location as home/base coordinates.
		
		Returns:
			bool: True if successful, False otherwise
		"""
		try:
			location = self.vehicle.location.global_frame
			home_coords = {
				"latitude": location.lat,
				"longitude": location.lon,
				"altitude": location.alt
			}
			
			with open(self.base_coords_file, 'w') as file:
				json.dump(home_coords, file)
			return True
		except Exception as e:
			print(f"Error saving base coordinates: {e}")
			return False
	
	def parse_waypoints_from_text(self, file_path):
		"""
		Parse waypoints from a text file.
		
		Args:
			file_path (str): Path to waypoint file
			
		Returns:
			list: List of waypoint dictionaries
		"""
		waypoints = []
		try:
			with open(file_path, 'r') as file:
				for line in file:
					# Skip header lines and empty lines
					if line.startswith("QGC") or line.strip() == "":
						continue
					
					parts = line.split()
					if len(parts) >= 11:  # Ensure enough components
						try:
							latitude = float(parts[8])
							longitude = float(parts[9])
							altitude = float(parts[10])
							waypoints.append({
								'latitude': latitude,
								'longitude': longitude,
								'altitude': altitude
							})
						except (ValueError, IndexError):
							continue
		except Exception as e:
			print(f"Error parsing waypoints from text: {e}")
			
		return waypoints
	
	def load_plan_file(self, plan_file_path):
		"""
		Load mission plan from a JSON file.
		
		Args:
			plan_file_path (str): Path to plan file
			
		Returns:
			list: List of waypoint tuples (lat, lon, alt)
		"""
		waypoints = []
		try:
			with open(plan_file_path, 'r') as file:
				plan_data = json.load(file)

			for item in plan_data.get('mission', {}).get('items', []):
				if item.get('command') == 16:  # MAV_CMD_NAV_WAYPOINT
					params = item.get('params', [])
					if len(params) >= 7:
						lat = params[4]
						lon = params[5]
						alt = params[6]
						waypoints.append((lat, lon, alt))
		except Exception as e:
			print(f"Error loading plan file: {e}")
			
		return waypoints
	
	def generate_survey_grid(self, boundary_points, altitude, spacing=10):
		"""
		Generate a survey grid pattern for mapping or scanning an area.
		
		Args:
			boundary_points (list): List of (lat, lon) tuples defining the area boundary
			altitude (float): Flight altitude for the survey
			spacing (float): Distance between parallel flight paths in meters
			
		Returns:
			list: List of waypoint tuples (lat, lon, alt)
		"""
		# Create polygon from boundary points
		boundary = Polygon(boundary_points)
		
		# Get bounding box
		min_lat = min(p[0] for p in boundary_points)
		max_lat = max(p[0] for p in boundary_points)
		min_lon = min(p[1] for p in boundary_points)
		max_lon = max(p[1] for p in boundary_points)
		
		# Convert spacing from meters to degrees
		lat_meters = 111320  # Approximate meters per degree of latitude
		lon_meters = 111320 * np.cos(np.radians((min_lat + max_lat) / 2))  # Meters per degree of longitude
		
		lat_spacing = spacing / lat_meters
		lon_spacing = spacing / lon_meters
		
		# Generate survey lines
		waypoints = []
		current_lon = min_lon
		direction = 1  # Start going north
		
		while current_lon <= max_lon:
			if direction == 1:  # Going north
				lat_range = np.arange(min_lat, max_lat, lat_spacing)
			else:  # Going south
				lat_range = np.arange(max_lat, min_lat, -lat_spacing)
				
			for lat in lat_range:
				point = Point(lat, current_lon)
				if boundary.contains(point):
					waypoints.append((lat, current_lon, altitude))
			
			# Switch direction and move to next line
			direction *= -1
			current_lon += lon_spacing
			
		return waypoints
	
	def create_circular_pattern(self, center_lat, center_lon, radius, altitude, points=8):
		"""
		Create a circular flight pattern around a center point.
		
		Args:
			center_lat (float): Center latitude
			center_lon (float): Center longitude
			radius (float): Radius in meters
			altitude (float): Flight altitude
			points (int): Number of points on the circle
			
		Returns:
			list: List of waypoint tuples (lat, lon, alt)
		"""
		# Convert radius from meters to degrees
		lat_meters = 111320  # Approximate meters per degree of latitude
		lon_meters = 111320 * np.cos(np.radians(center_lat))  # Meters per degree of longitude
		
		lat_radius = radius / lat_meters
		lon_radius = radius / lon_meters
		
		# Generate points around the circle
		waypoints = []
		for i in range(points):
			angle = 2 * np.pi * i / points
			lat = center_lat + lat_radius * np.sin(angle)
			lon = center_lon + lon_radius * np.cos(angle)
			waypoints.append((lat, lon, altitude))
			
		# Add the first point again to complete the circle
		waypoints.append(waypoints[0])
		
		return waypoints
	
	def get_battery_status(self):
		"""
		Get detailed battery status.
		
		Returns:
			dict: Battery status information
		"""
		battery = self.vehicle.battery
		return {
			'level': battery.level,
			'voltage': battery.voltage,
			'current': battery.current,
			'estimated_time_remaining': self._estimate_flight_time_remaining()
		}
	
	def _estimate_flight_time_remaining(self):
		"""
		Estimate remaining flight time based on battery level.
		
		Returns:
			float: Estimated flight time in minutes
		"""
		# This is a simplified model and should be calibrated for each drone
		battery_level = self.vehicle.battery.level
		
		# Assuming a linear relationship between battery and flight time
		# with 20 minutes of flight time at 100% battery
		max_flight_time = 20  # minutes
		
		# Conservative estimate
		return (battery_level / 100) * max_flight_time * 0.8