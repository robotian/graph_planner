import pyzed.sl as sl
import numpy as np
import struct
import time

def save_pcd(filename, points, colors):
    """Saves XYZ points and RGB colors with float32-packed RGB for PCL viewer."""
    header = f"""# .PCD v0.7 - Point Cloud Data file format
VERSION 0.7
FIELDS x y z rgb
SIZE 4 4 4 4
TYPE F F F F
COUNT 1 1 1 1
WIDTH {len(points)}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {len(points)}
DATA ascii
"""
    with open(filename, 'w') as f:
        f.write(header)
        for p, c in zip(points, colors):
            r, g, b = int(c[0]*255), int(c[1]*255), int(c[2]*255)
            # Pack RGB into float for PCL compatibility
            rgb_packed = struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]
            f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f} {rgb_packed:.8e}\n")
    print(f"\n[SUCCESS] Saved PCD file: {filename}")


def extract_obstacles(points, mount_height=0.30, ground_clearance=0.10, wall_angle_threshold=45.0):
    """
    Extracts obstacles by combining height thresholding and surface tilt detection.
    
    :param points: Nx3 numpy array (X, Y, Z) in Right-Handed Y-Up frame.
    :param mount_height: Distance (meters) from camera to ground (e.g., 0.30m = 30cm).
    :param ground_clearance: Distance (meters) above ground considered safe floor.
    :param wall_angle_threshold: Minimum angle (degrees) relative to ground to be considered a wall/obstacle.
    :return: obstacle_mask (Boolean array where True = Obstacle, False = Ground)
    """
    # Ground height cutoff in Y-up frame (Camera is at Y=0, Ground is at Y = -mount_height)
    ground_cutoff_y = -mount_height + ground_clearance

    # 1. Height-based initial mask
    # Anything above (Floor Level + Clearance) MUST be an obstacle (including walls, boxes, chairs)
    height_obstacle_mask = points[:, 1] > ground_cutoff_y

    # 2. For points close to the ground, check local verticality to prevent misclassifying low walls
    # Calculate radial distances and elevation angles relative to camera
    x, y, z = points[:, 0], points[:, 1], points[:, 2]
    r = np.sqrt(x**2 + z**2)
    
    # Avoid division by zero
    r_safe = np.maximum(r, 0.001)
    
    # Calculate local slope / angle relative to horizontal plane
    elevation_angles = np.degrees(np.arctan2(np.abs(y + mount_height), r_safe))

    # Points are marked as obstacles if they are above ground cutoff OR have high slope
    obstacle_mask = height_obstacle_mask | (elevation_angles > wall_angle_threshold)

    return obstacle_mask


def main():
    # 1. Initialize ZED Camera
    zed = sl.Camera()
    init_params = sl.InitParameters()
    init_params.depth_mode = sl.DEPTH_MODE.NEURAL
    init_params.coordinate_units = sl.UNIT.METER
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    init_params.camera_image_flip = sl.FLIP_MODE.OFF  # Keeps IMU active

    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("Failed to open ZED camera.")
        return

    # Enable Positional Tracking
    tracking_params = sl.PositionalTrackingParameters()
    zed.enable_positional_tracking(tracking_params)

    sensors_data = sl.SensorsData()
    point_cloud = sl.Mat()
    runtime_params = sl.RuntimeParameters()

    # 180° rotation matrix around Z-axis (corrects physical upside-down camera mount)
    R_upside_down = np.array([
        [-1.0,  0.0,  0.0],
        [ 0.0, -1.0,  0.0],
        [ 0.0,  0.0,  1.0]
    ])

    print("Capturing frame for Wall-Safe Ground Extraction...")

    # Grab 1 valid frame
    while True:
        if zed.grab(runtime_params) == sl.ERROR_CODE.SUCCESS:
            # Get IMU Orientation Matrix for Gravity Alignment
            zed.get_sensors_data(sensors_data, sl.TIME_REFERENCE.IMAGE)
            imu_data = sensors_data.get_imu_data()
            sl_orientation = imu_data.get_pose().get_orientation()
            R_imu = sl_orientation.get_rotation_matrix().r

            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZRGBA)
            pc_data = point_cloud.get_data()
            points = pc_data[:, :, :3].reshape(-1, 3)

            valid_mask = np.isfinite(points).all(axis=1)
            valid_points = points[valid_mask]
            if len(valid_points) > 0:
                break

    zed.close()

    # 2. Correct physical upside-down mount & align with gravity
    inverted_points = np.dot(valid_points, R_upside_down)
    gravity_aligned_points = np.dot(inverted_points, R_imu)

    print("Extracting obstacles and filtering ground...")
    start_time = time.time()

    # 3. Filter point cloud
    # MOUNT_HEIGHT = 0.30m (30cm), GROUND_CLEARANCE = 0.08m (8cm)
    obstacle_mask = extract_obstacles(
        gravity_aligned_points, 
        mount_height=0.30, 
        ground_clearance=0.08, 
        wall_angle_threshold=25.0
    )

    elapsed_time = time.time() - start_time
    print(f"Filter completed in {elapsed_time:.3f} seconds.")

    # 4. Extract Obstacles and Ground (using inverted_points for visualization)
    obstacles = inverted_points[obstacle_mask]
    obs_colors = np.tile([0.0, 1.0, 0.0], (len(obstacles), 1))  # GREEN

    ground = inverted_points[~obstacle_mask]
    ground_colors = np.tile([1.0, 0.0, 0.0], (len(ground), 1))  # RED

    print(f"\nResults:")
    print(f"Total Valid Points: {len(valid_points)}")
    print(f"  └─ Obstacles Kept (GREEN): {len(obstacles)}")
    print(f"  └─ Ground Filtered (RED)  : {len(ground)}")

    # 5. Export PCD file
    all_points = np.vstack([obstacles, ground])
    all_colors = np.vstack([obs_colors, ground_colors])
    save_pcd("wall_safe_obstacles.pcd", all_points, all_colors)


if __name__ == "__main__":
    main()