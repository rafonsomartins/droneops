"""
Mission planning utilities for DroneOps library.
"""

import json
import numpy as np
from shapely.geometry import Polygon

class MissionPlanner:
	"""
	Tools for creating and managing drone mission plans.
	"""
	
	@staticmethod
	def create_mission_file(waypoints, filename="mission.json", mission_type="survey"):
		"""
		Create a mission plan file compatible with DroneOps and common ground control stations.
		
		Args:
			waypoints (list): List of waypoint tuples (lat, lon, alt)
			filename (str): Output file name
			mission_type (str): Type of mission ('survey', 'delivery', 'inspection')
			
		Returns:
			str: Path to the created mission file
		"""
		mission = {
			"fileType": "Plan",
			"geoFence": {
				"circles": [],
				"polygons": [],
				"version": 2
			},
			"groundStation": "DroneOps",
			"mission": {
				"cruiseSpeed": 15,
				"firmwareType": 3,
				"hoverSpeed": 5,
				"items": [],
				"plannedHomePosition": [
					waypoints[0][0],
					waypoints[0][1],
					waypoints[0][2]
				],
				"vehicleType": 2,
				"version": 2
			},
			"rallyPoints": {
				"points": [],
				"version": 2
			},
			"version": 1
		}
		
		# Add home position
		mission["mission"]["items"].append({
			"autoContinue": True,
			"command": 22,  # MAV_CMD_NAV_TAKEOFF
			"doJumpId": 1,
			"frame": 3,  # MAV_FRAME_GLOBAL_RELATIVE_ALT
			"params": [
				0,  # Min pitch
				0,  # Empty
				0,  # Empty
				0,  # Yaw angle
				waypoints[0][0],  # Latitude
				waypoints[0][1],  # Longitude
				waypoints[0][2]   # Altitude
			],
			"type": "SimpleItem"
		})
		
		# Add waypoints
		for i, waypoint in enumerate(waypoints):
			if i == 0:  # Skip the first waypoint as it's already added as takeoff
				continue
				
			mission["mission"]["items"].append({
				"autoContinue": True,
				"command": 16,  # MAV_CMD_NAV_WAYPOINT
				"doJumpId": i + 1,
				"frame": 3,  # MAV_FRAME_GLOBAL_RELATIVE_ALT
				"params": [
					0,  # Hold time
					1,  # Accept radius
					0,  # Pass radius
					0,  # Yaw
					waypoint[0],  # Latitude
					waypoint[1],  # Longitude
					waypoint[2]   # Altitude
				],
				"type": "SimpleItem"
			})
		
		# Add RTL at the end
		mission["mission"]["items"].append({
			"autoContinue": True,
			"command": 20,  # MAV_CMD_NAV_RETURN_TO_LAUNCH
			"doJumpId": len(waypoints) + 1,
			"frame": 3,
			"params": [0, 0, 0, 0, 0, 0, 0],
			"type": "SimpleItem"
		})
		
		# Write mission to file
		with open(filename, 'w') as f:
			json.dump(mission, f, indent=4)
			
		return filename
	
	@staticmethod
	def create_survey_grid(boundary_points, altitude, spacing=10, crosshatch=False):
		"""
		Generate a survey grid pattern for mapping or scanning an area.
		
		Args:
			boundary_points (list): List of (lat, lon) tuples defining the area boundary
			altitude (float): Flight altitude for the survey
			spacing (float): Distance between parallel flight paths in meters
			crosshatch (bool): If True, add perpendicular lines for a crosshatch pattern
			
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

		lat_meters = 111320  # Approximate meters per degree of latitude
		lon_meters = 111320 * np.cos(np.radians((min_lat + max_lat) / 2))  # Meters per degree of longitude
		
		lat_spacing = spacing / lat_meters
		lon_spacing = spacing / lon_meters
		
		# Generate primary survey lines (east-west)
		waypoints = []
		current_lon = min_lon
		direction = 1  # Start going north
		
		while current_lon <= max_lon:
			if direction == 1:  # Going north
				lat_range = np.arange(min_lat, max_lat, lat_spacing/10)
			else:  # Going south
				lat_range = np.arange(max_lat, min_lat, -lat_spacing/10)
				
			line_points = []
			for lat in lat_range:
				point = Point(lat, current_lon)
				if boundary.contains(point):
					line_points.append((lat, current_lon, altitude))
			
			# Only add points if we have a valid line
			if line_points:
				# Add the points in the correct order
				waypoints.extend(line_points)
			
			# Switch direction and move to next line
			direction *= -1
			current_lon += lon_spacing
		
		# Add crosshatch pattern if requested
		if crosshatch:
			# Generate crosshatch survey lines (north-south)
			crosshatch_waypoints = []
			current_lat = min_lat
			direction = 1  # Start going east
			
			while current_lat <= max_lat:
				if direction == 1:  # Going east
					lon_range = np.arange(min_lon, max_lon, lon_spacing/10)
				else:  # Going west
					lon_range = np.arange(max_lon, min_lon, -lon_spacing/10)
					
				line_points = []
				for lon in lon_range:
					point = Point(current_lat, lon)
					if boundary.contains(point):
						line_points.append((current_lat, lon, altitude))
				
				# Only add points if we have a valid line
				if line_points:
					# Add the points in the correct order
					crosshatch_waypoints.extend(line_points)
				
				# Switch direction and move to next line
				direction *= -1
				current_lat += lat_spacing
			
			# Combine both patterns
			waypoints.extend(crosshatch_waypoints)
			
		return waypoints
	
	@staticmethod
	def create_perimeter_scan(boundary_points, altitude, offset=0):
		"""
		Create a perimeter scan mission following the boundary of an area.
		
		Args:
			boundary_points (list): List of (lat, lon) tuples defining the area boundary
			altitude (float): Flight altitude for the scan
			offset (float): Distance in meters to offset from the boundary (negative for inside, positive for outside)
			
		Returns:
			list: List of waypoint tuples (lat, lon, alt)
		"""
		if offset != 0:
			# Creating an offset polygon is complex and requires specialized geometry operations
			# For simplicity, we'll skip the offset implementation in this example
			pass
		
		# Use the boundary points directly as waypoints
		waypoints = [(lat, lon, altitude) for lat, lon in boundary_points]
		
		# Add the first point again to close the loop
		waypoints.append(waypoints[0])
		
		return waypoints
	
	@staticmethod
	def create_poi_inspection(poi_lat, poi_lon, altitude, radius, points=8):
		"""
		Create a mission to inspect a point of interest from multiple angles.
		
		Args:
			poi_lat (float): Point of interest latitude
			poi_lon (float): Point of interest longitude
			altitude (float): Flight altitude
			radius (float): Radius around the POI in meters
			points (int): Number of inspection points
			
		Returns:
			list: List of waypoint tuples (lat, lon, alt)
		"""
		# Convert radius from meters to degrees
		lat_meters = 111320  # Approximate meters per degree of latitude
		lon_meters = 111320 * np.cos(np.radians(poi_lat))  # Meters per degree of longitude
		
		lat_radius = radius / lat_meters
		lon_radius = radius / lon_meters
		
		# Generate points around the POI
		waypoints = []
		for i in range(points):
			angle = 2 * np.pi * i / points
			lat = poi_lat + lat_radius * np.sin(angle)
			lon = poi_lon + lon_radius * np.cos(angle)
			waypoints.append((lat, lon, altitude))
			
		# Add the first point again to complete the circle
		waypoints.append(waypoints[0])
		
		return waypoints
	
	@staticmethod
	def calculate_mission_stats(waypoints):
		"""
		Calculate statistics for a mission.
		
		Args:
			waypoints (list): List of waypoint tuples (lat, lon, alt)
			
		Returns:
			dict: Mission statistics
		"""
		# Calculate total distance
		total_distance = 0
		for i in range(len(waypoints) - 1):
			# Calculate distance between waypoints
			lat1, lon1, _ = waypoints[i]
			lat2, lon2, _ = waypoints[i + 1]
			
			# Simple Haversine formula for distance
			R = 6371000  # Earth radius in meters
			lat1_rad = np.radians(lat1)
			lat2_rad = np.radians(lat2)
			dlat = np.radians(lat2 - lat1)
			dlon = np.radians(lon2 - lon1)
			
			a = np.sin(dlat/2)**2 + np.cos(lat1_rad) * np.cos(lat2_rad) * np.sin(dlon/2)**2
			c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
			distance = R * c
			
			total_distance += distance
		
		# Calculate altitude changes
		altitude_changes = []
		for i in range(len(waypoints) - 1):
			alt_change = waypoints[i+1][2] - waypoints[i][2]
			altitude_changes.append(alt_change)
		
		# Calculate estimated flight time (assuming 15 m/s cruise speed)
		cruise_speed = 15  # m/s
		estimated_time = total_distance / cruise_speed
		
		# Add time for climbs and descents (simplified)
		for alt_change in altitude_changes:
			if alt_change > 0:
				# Climbing at 5 m/s
				estimated_time += abs(alt_change) / 5
			else:
				# Descending at 3 m/s
				estimated_time += abs(alt_change) / 3
		
		return {
			'total_distance': total_distance,
			'waypoint_count': len(waypoints),
			'estimated_flight_time': estimated_time,
			'max_altitude': max(wp[2] for wp in waypoints),
			'min_altitude': min(wp[2] for wp in waypoints),
		}