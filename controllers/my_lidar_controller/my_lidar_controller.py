from controller import Robot

# Create the robot instance
robot = Robot()

# Get the simulation timestep
timestep = int(robot.getBasicTimeStep())

# Initialize the LiDAR sensor
lidar = robot.getDevice('lidar')  # Replace 'lidar' with the exact name of your LiDAR in Webots
lidar.enable(timestep)

# Enable point cloud
lidar.enablePointCloud()

# Main simulation loop
while robot.step(timestep) != -1:
    # Get the range image from the LiDAR
    lidar_data = lidar.getRangeImage()
    print("Lidar data (first 10 values):", lidar_data[:10])

    # Find and print the minimum distance detected
    min_distance = min(lidar_data)
    print("Minimum distance detected by Lidar:", min_distance)

    # Get and process the point cloud
    point_cloud = lidar.getPointCloud()
    if point_cloud:
        print("Point Cloud Data (first 5 points):")
        for i, point in enumerate(point_cloud[:5]):  # Display only the first 5 points
            print(f"Point {i}: x={point.x:.2f}, y={point.y:.2f}, z={point.z:.2f}")
