#!/usr/bin/env python3

import rclpy
from offboard_control.drone_controller import DroneController
from offboard_control.path_planner import PathPlanner
from offboard_control.map_loader import MapLoader

def main(args=None):
    rclpy.init(args=args)

    # Load bản đồ
    map_loader = MapLoader("/home/hai/DevPX4/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/my_world.world")
    obstacles = map_loader.load_obstacles()
    
    print(f"\nLoaded obstacles: {obstacles}")

    # Điểm bắt đầu và điểm kết thúc
    start = (0, 0, 2)
    goal = (7, 7, 2)

    # Tìm đường với Bi-RRT
    planner = PathPlanner(obstacles)
    rrt_path = planner.bi_RRT(start, goal)
    print("\nRRT path:", rrt_path)
    
    # Rut gọn đường đi
    simplified_path = planner.simplify_path(rrt_path)
    print("\nSimplify path:", simplified_path)
    
    # Tang mat do diem
    densed_path = planner.densify_path(simplified_path, 1)
    print("\nDensify path:", densed_path)
        
    # Khởi tạo drone controller với đường đi đã tìm
    node = DroneController(densed_path)
    print("\nDroneController started successfully!")
    rclpy.spin(node)

if __name__ == "__main__":
    main()
