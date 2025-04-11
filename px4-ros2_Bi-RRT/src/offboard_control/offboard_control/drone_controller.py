import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from math import fabs

class DroneController(Node):
    def __init__(self, path):
        super().__init__('offboard_rrt_control')

        # QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publisher
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos_profile)

        # Subscriber
        self.vehicle_local_position_sub = self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.vehicle_local_position_callback, qos_profile)
        self.vehicle_status_sub = self.create_subscription(VehicleStatus, "/fmu/out/vehicle_status", self.vehicle_status_callback, qos_profile)

        # Biến trạng thái
        self.vehicle_local_position = VehicleLocalPosition()
        self.vehicle_status = VehicleStatus()
        self.path = path
        self.current_waypoint_index = 0
        self.offset_XYsetpoint = 0.1
        self.offset_Zsetpoint = 0.5
        self.state = "START"

        self.timer = self.create_timer(0.1, self.timer_callback)

    def vehicle_local_position_callback(self, vehicle_local_position):
        self.vehicle_local_position = vehicle_local_position
        
    def vehicle_status_callback(self, vehicle_status):
        self.vehicle_status = vehicle_status

    def arm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
        
    def disarm(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)

    def offboard_mode(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)

    def land(self):
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        
    def publish_position_setpoint_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)
    
    def publish_velocity_setpoint_heartbeat(self):
        msg = OffboardControlMode()
        msg.position = False
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.offboard_control_mode_pub.publish(msg)

    def publish_position_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)
        
    def publish_velocity_setpoint(self):
        msg = TrajectorySetpoint()
        msg.velocity = [0.2, 0.2, 0.2]
        msg.yaw = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command, **params):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.from_external = True
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self.vehicle_command_pub.publish(msg)
    
    def timer_callback(self):
        
        if self.state == "START":
            self.offboard_mode()
            self.arm()
            self.state = "TAKEOFF"
            print("Drone is taking off...")
            print("Path lenght:", len(self.path))
            
        elif self.state == "TAKEOFF":
            self.publish_position_setpoint(0.0, 0.0, -2.0)
            if fabs(self.vehicle_local_position.z - (-2.0)) < self.offset_Zsetpoint:
                self.state = "FOLLOW_PATH"

        elif self.state == "FOLLOW_PATH" and self.current_waypoint_index < len(self.path):
            x, y, z = self.path[self.current_waypoint_index]
            self.publish_position_setpoint(x, y, -z)

            if fabs(self.vehicle_local_position.x - x) < self.offset_XYsetpoint and \
            fabs(self.vehicle_local_position.y - y) < self.offset_XYsetpoint and \
            fabs(self.vehicle_local_position.z - (-z)) < self.offset_Zsetpoint:
                self.current_waypoint_index += 1
                print("current_waypoint_index:", self.current_waypoint_index)
                if self.current_waypoint_index >= len(self.path):
                    self.state = "LAND"
                    self.land()
                    print("Drone is landing...")

        elif self.state == "LAND":
            if self.vehicle_status.arming_state == VehicleStatus.ARMING_STATE_DISARMED:
                print("Drone disarmed!")
                self.state = "FINISHED"
            
        elif self.state == "FINISHED":
            self.get_logger().info("Mission complete")
            self.destroy_node()
            rclpy.shutdown()
            return
          
        self.publish_position_setpoint_heartbeat()
