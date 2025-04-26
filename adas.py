import carla
import cv2
import numpy as np
import time
import threading
import math
import random

# ==============================
# 1. Connect to CARLA
# ==============================
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()

# ==============================
# 2. Set Foggy Weather
# ==============================
weather = world.get_weather()
weather.fog_density = 80.0  # Adjust fog intensity (0-100)
weather.fog_distance = 10.0  # Visibility range in meters
weather.fog_falloff = 1.0
world.set_weather(weather)
print("🌫 Foggy Weather Applied!")

# ==============================
# 3. Spawn the Ego Vehicle
# ==============================
blueprint_library = world.get_blueprint_library()
vehicle_bp = blueprint_library.filter('model3')[0]  # Tesla Model 3
spawn_point = random.choice(world.get_map().get_spawn_points())  # Random spawn point
vehicle = world.spawn_actor(vehicle_bp, spawn_point)
vehicle.set_autopilot(True)
print("🚗 Vehicle Spawned and Autopilot Enabled!")

# ==============================
# 4. Attach a Spectator Camera
# ==============================
spectator = world.get_spectator()

def update_spectator():
    while True:
        transform = vehicle.get_transform()
        location = transform.location
        rotation = transform.rotation

        distance_behind = 6.0
        height_above = 2.5

        yaw_rad = math.radians(rotation.yaw)
        cam_x = location.x - distance_behind * math.cos(yaw_rad)
        cam_y = location.y - distance_behind * math.sin(yaw_rad)
        cam_z = location.z + height_above

        spectator.set_transform(carla.Transform(
            carla.Location(x=cam_x, y=cam_y, z=cam_z),
            carla.Rotation(pitch=-10, yaw=rotation.yaw)
        ))
        time.sleep(0.05)

spectator_thread = threading.Thread(target=update_spectator)
spectator_thread.daemon = True
spectator_thread.start()
print("📷 Spectator Camera Activated!")

# ==============================
# 5. Attach Front Camera for Lane Detection
# ==============================
camera_bp = blueprint_library.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '800')
camera_bp.set_attribute('image_size_y', '600')
camera_bp.set_attribute('fov', '110')

camera_transform = carla.Transform(carla.Location(x=0.5, z=1.5))
camera_sensor = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)

def lane_detection(image):
    img = np.array(image.raw_data, dtype=np.uint8)
    img = img.reshape((image.height, image.width, 4))[:, :, :3]

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)

    height, width = edges.shape
    mask = np.zeros_like(edges)
    polygon = np.array([
        [(width * 0.1, height), (width * 0.9, height),
         (width * 6/10, height * 5/10), (width * 4/10, height * 5/10)]
    ], np.int32)
    cv2.fillPoly(mask, polygon, 255)

    masked_edges = cv2.bitwise_and(edges, mask)
    lines = cv2.HoughLinesP(masked_edges, 2, np.pi/180, 100, np.array([]), minLineLength=40, maxLineGap=5)

    line_image = np.zeros_like(img)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_image, (x1, y1), (x2, y2), (0, 255, 0), 5)

    lane_overlay = cv2.addWeighted(img, 0.8, line_image, 1, 1)
    cv2.imshow("Lane Detection", lane_overlay)
    cv2.waitKey(1)

camera_sensor.listen(lambda image: lane_detection(image))
print("🚦 Lane Detection Activated!")

# ==============================
# 6. Attach Radar Sensor for Object Detection
# ==============================
radar_bp = blueprint_library.find('sensor.other.radar')
radar_bp.set_attribute('horizontal_fov', '30.0')
radar_bp.set_attribute('vertical_fov', '20.0')
radar_bp.set_attribute('range', '50.0')

radar_transform = carla.Transform(carla.Location(x=2.0, z=1.0))
radar_sensor = world.spawn_actor(radar_bp, radar_transform, attach_to=vehicle)

def radar_callback(data):
    for detection in data:
        distance = detection.depth  # Distance of object in meters
        velocity = detection.velocity
        print(f"📡 Radar: Object detected at {distance:.2f}m, velocity={velocity:.2f} m/s")

radar_sensor.listen(lambda data: radar_callback(data))
print("📡 Radar Sensor Activated!")

# ==============================
# 7. Spawn Traffic Vehicles
# ==============================
traffic_manager = client.get_trafficmanager()
traffic_manager.set_global_distance_to_leading_vehicle(2.5)

num_vehicles = 10  # Number of AI vehicles
spawn_points = world.get_map().get_spawn_points()
vehicles_list = []

for _ in range(num_vehicles):
    vehicle_bp = random.choice(blueprint_library.filter('vehicle.*'))
    spawn_point = random.choice(spawn_points)
    ai_vehicle = world.try_spawn_actor(vehicle_bp, spawn_point)
    if ai_vehicle:
        ai_vehicle.set_autopilot(True, traffic_manager.get_port())
        vehicles_list.append(ai_vehicle)

print(f"🚗 {len(vehicles_list)} AI Traffic Vehicles Spawned!")

# ==============================
# 8. Spawn Pedestrians
# ==============================
walker_bp = random.choice(blueprint_library.filter('walker.pedestrian.*'))
walker_controller_bp = blueprint_library.find('controller.ai.walker')

num_pedestrians = 5  # Number of NPCs
pedestrians_list = []

for _ in range(num_pedestrians):
    spawn_point = carla.Transform()
    spawn_point.location = world.get_random_location_from_navigation()
    pedestrian = world.try_spawn_actor(walker_bp, spawn_point)
    if pedestrian:
        pedestrians_list.append(pedestrian)
        walker_controller = world.spawn_actor(walker_controller_bp, carla.Transform(), attach_to=pedestrian)
        walker_controller.start()
        walker_controller.go_to_location(world.get_random_location_from_navigation())
        walker_controller.set_max_speed(1.5)

print(f"🚶 {len(pedestrians_list)} Pedestrians Spawned!")

# ==============================
# 9. Keep the Simulation Running
# ==============================
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("\n🛑 Stopping Simulation...")

    # Destroy all spawned actors
    for actor in vehicles_list + pedestrians_list + [vehicle, camera_sensor, radar_sensor]:
        if actor is not None:
            actor.destroy()

    cv2.destroyAllWindows()
    print("✅ Cleanup Complete!")