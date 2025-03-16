import json
import os
import numpy as np
from shapely.geometry import Point, Polygon


class QGroundControlUtils:
    """
    Utilities for working with QGroundControl mission plans and integrating with DroneOps.
    """
    
    def __init__(self):
        """Initialize QGroundControl utilities"""
        self.default_cruise_speed = 15  # m/s
        self.default_hover_speed = 5    # m/s
        
    def convert_qgc_to_droneops_mission(self, qgc_plan_path, output_path=None):
        """
        Convert a QGroundControl mission plan file to DroneOps waypoints format.
        
        Args:
            qgc_plan_path (str): Path to QGC plan file (.plan)
            output_path (str, optional): Path to save the converted waypoints. If None, returns waypoints list.
            
        Returns:
            list: List of (lat, lon, alt) tuples if output_path is None, otherwise path to the output file
        """
        waypoints = []
        
        try:
            with open(qgc_plan_path, 'r') as f:
                plan_data = json.load(f)
                
            # Extract mission items
            items = plan_data.get("mission", {}).get("items", [])
            
            for item in items:
                # Only process waypoints (command 16) and takeoff (command 22)
                if item.get("command") in [16, 22]:  # MAV_CMD_NAV_WAYPOINT or MAV_CMD_NAV_TAKEOFF
                    params = item.get("params", [])
                    if len(params) >= 7:
                        lat = params[4]
                        lon = params[5]
                        alt = params[6]
                        waypoints.append((lat, lon, alt))
                        
            if output_path:
                with open(output_path, 'w') as f:
                    json.dump(waypoints, f, indent=4)
                return output_path
            else:
                return waypoints
                
        except Exception as e:
            print(f"Error converting QGC plan: {e}")
            return None
            
    def create_qgc_mission(self, waypoints, filename="mission.plan", mission_type="survey"):
        """
        Create a QGroundControl mission plan file from waypoints.
        
        Args:
            waypoints (list): List of waypoint tuples (lat, lon, alt)
            filename (str): Output file name
            mission_type (str): Type of mission ('survey', 'delivery', 'inspection')
            
        Returns:
            str: Path to the created mission file
        """
        if not waypoints:
            print("Error: No waypoints provided")
            return None
            
        mission = {
            "fileType": "Plan",
            "geoFence": {
                "circles": [],
                "polygons": [],
                "version": 2
            },
            "groundStation": "QGroundControl",
            "mission": {
                "cruiseSpeed": self.default_cruise_speed,
                "firmwareType": 3,  # PX4
                "hoverSpeed": self.default_hover_speed,
                "items": [],
                "plannedHomePosition": [
                    waypoints[0][0],
                    waypoints[0][1],
                    waypoints[0][2]
                ],
                "vehicleType": 2,  # Multirotor
                "version": 2
            },
            "rallyPoints": {
                "points": [],
                "version": 2
            },
            "version": 1
        }
        
        # Add home position and takeoff
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
        
    def create_survey_grid(self, boundary_points, altitude, spacing=10, crosshatch=False):
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
        
        # Convert spacing from meters to degrees
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
                    waypoints.extend(line_points)
                    
                # Switch direction and move to next line
                direction *= -1
                current_lat += lat_spacing
                
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
        
    def calculate_mission_stats(self, waypoints):
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
        cruise_speed = self.default_cruise_speed  # m/s
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
        
    def import_kml_boundary(self, kml_file):
        """
        Import a KML file and extract boundary points.
        
        Args:
            kml_file (str): Path to KML file
            
        Returns:
            list: List of (lat, lon) tuples defining the boundary
        """
        try:
            from fastkml import kml
            
            with open(kml_file, 'rb') as f:
                doc = f.read()
                
            k = kml.KML()
            k.from_string(doc)
            
            boundary_points = []
            
            for feature in list(list(k.features())[0].features()):
                # Extract the first polygon found
                if hasattr(feature, 'geometry') and feature.geometry is not None:
                    # Get coordinates
                    coords = list(feature.geometry.exterior.coords)
                    # KML format is (lon, lat, [alt]) - we need to swap to (lat, lon)
                    boundary_points = [(c[1], c[0]) for c in coords]
                    break
                    
            return boundary_points
            
        except ImportError:
            print("Error: fastkml package is required for KML import.")
            print("Install it using: pip install fastkml")
            return None
        except Exception as e:
            print(f"Error importing KML file: {e}")
            return None
            
    def export_as_kml(self, waypoints, filename="mission.kml", name="DroneOps Mission"):
        """
        Export waypoints as a KML file.
        
        Args:
            waypoints (list): List of waypoint tuples (lat, lon, alt)
            filename (str): Output filename
            name (str): Name of the mission
            
        Returns:
            str: Path to the created KML file
        """
        try:
            # Create KML file content
            kml_content = f"""<?xml version="1.0" encoding="UTF-8"?>
<kml xmlns="http://www.opengis.net/kml/2.2">
  <Document>
    <name>{name}</name>
    <Style id="yellowLineGreenPoly">
      <LineStyle>
        <color>7f00ffff</color>
        <width>4</width>
      </LineStyle>
      <PolyStyle>
        <color>7f00ff00</color>
      </PolyStyle>
    </Style>
    <Placemark>
      <name>Mission Waypoints</name>
      <styleUrl>#yellowLineGreenPoly</styleUrl>
      <LineString>
        <extrude>1</extrude>
        <tessellate>1</tessellate>
        <altitudeMode>relativeToGround</altitudeMode>
        <coordinates>
"""
            
            # Add waypoint coordinates
            for waypoint in waypoints:
                lat, lon, alt = waypoint
                # KML format is lon,lat,alt
                kml_content += f"{lon},{lat},{alt}\n"
                
            kml_content += """
        </coordinates>
      </LineString>
    </Placemark>
"""
            
            # Add waypoint markers
            for i, waypoint in enumerate(waypoints):
                lat, lon, alt = waypoint
                kml_content += f"""
    <Placemark>
      <name>WP{i}</name>
      <Point>
        <coordinates>{lon},{lat},{alt}</coordinates>
      </Point>
    </Placemark>
"""
                
            kml_content += """
  </Document>
</kml>"""
            
            # Write to file
            with open(filename, 'w') as f:
                f.write(kml_content)
                
            return filename
            
        except Exception as e:
            print(f"Error exporting as KML: {e}")
            return None
            
    def split_mission(self, waypoints, max_waypoints=100):
        """
        Split a large mission into smaller segments to work within QGC limitations.
        
        Args:
            waypoints (list): List of waypoint tuples (lat, lon, alt)
            max_waypoints (int): Maximum number of waypoints per segment
            
        Returns:
            list: List of waypoint lists, each representing a mission segment
        """
        segments = []
        
        # If the mission is already small enough
        if len(waypoints) <= max_waypoints:
            return [waypoints]
            
        # Split into segments, ensuring first waypoint is included in each segment
        for i in range(0, len(waypoints), max_waypoints - 1):
            segment = [waypoints[0]] if i > 0 else []
            segment.extend(waypoints[i:i + max_waypoints - (1 if i > 0 else 0)])
            segments.append(segment)
            
        return segments
        
    def create_mission_files(self, segments, base_filename="mission_segment"):
        """
        Create mission files for each segment.
        
        Args:
            segments (list): List of waypoint lists
            base_filename (str): Base name for output files
            
        Returns:
            list: Paths to created mission files
        """
        files = []
        
        for i, segment in enumerate(segments):
            filename = f"{base_filename}_{i+1}.plan"
            path = self.create_qgc_mission(segment, filename)
            files.append(path)
            
        return files