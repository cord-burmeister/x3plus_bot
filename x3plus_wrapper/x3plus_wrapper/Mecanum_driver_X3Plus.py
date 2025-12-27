"""
Mecanum_driver_X3Plus.py
========================

This script defines a ROS2 node for controlling a Yahboom Mecanum-wheeled robot (X3Plus). 
It provides functionality for subscribing to control commands, publishing sensor data, and managing the robot's motion.

Classes:
--------
- yahboomcar_driver: A ROS2 node that interfaces with the Yahboom Mecanum-wheeled robot.

Functions:
----------
- main(): Initializes the ROS2 node and starts the event loop.

Dependencies:
-------------
- ROS2 libraries: rclpy, std_msgs, geometry_msgs, sensor_msgs
- Custom library: Rosmaster_Lib (provides hardware-specific functionality for the robot)

"""

# Import standard libraries
import numpy as np
import sys
import math
import random
import threading
from math import pi
from time import sleep

# Import custom library for robot control
from Rosmaster_Lib import Rosmaster

# Import ROS2 libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32, Int32, Bool
from geometry_msgs.msg import Twist, Quaternion
from sensor_msgs.msg import Imu, MagneticField, JointState
from rclpy.clock import Clock
from nav_msgs.msg import Odometry
from tf_transformations import quaternion_from_euler

from rcl_interfaces.msg import SetParametersResult

# Dictionary mapping car types to their respective IDs
car_type_dic = {
    'R2': 5,
    'X3': 1,
    'X3Plus': 2,
    'NONE': -1
}

class yahboomcar_driver(Node):
    """
    A ROS2 node for controlling the Yahboom Mecanum-wheeled robot.

    Attributes:
    -----------
    - car: Instance of the Rosmaster class for hardware control.
    - Parameters: Various ROS2 parameters for configuration (e.g., car type, imu link, motion limits).
    - Publishers: ROS2 publishers for sensor data (e.g., IMU, magnetic field, voltage).
    - Subscribers: ROS2 subscribers for receiving control commands (e.g., cmd_vel, Buzzer).
    - Timer: A periodic timer for publishing sensor data.

    Methods:
    --------
    - __init__(name): Initializes the node and sets up parameters, publishers, subscribers, and timers.
    - cmd_vel_callback(msg): Callback for processing velocity commands (Twist messages).
    - RGBLightcallback(msg): Callback for controlling RGB lights (Int32 messages).
    - Buzzercallback(msg): Callback for controlling the buzzer (Bool messages).
    - pub_data(): Periodically publishes sensor data (IMU, magnetic field, velocity, etc.).
    - cleanup(): Cleans up resources and stops the robot when shutting down the node.
    """

    def __init__(self, name):
        """
        Initializes the yahboomcar_driver node.

        Parameters:
        -----------
        - name (str): Name of the ROS2 node.
        """
        super().__init__(name)

		# Initialize the node 
        self.front_right_encoder_old = 0.0	# Last processed encoder value right 
        self.front_left_encoder_old = 0.0;	# Last processed encoder value left
        self.rear_right_encoder_old = 0.0;	# Last processed encoder value right 
        self.rear_left_encoder_old = 0.0;	
        self.last_time_stamp = Clock().now()


		# Radius wheels (in meters)
        self.wheel_radius= 0.040          # meters
		# Distance between front and rear wheels (in meters)
        # self.wheel_separation_length = 0.22  # Based on spec
        self.wheel_separation_length = 0.22 # Measured 
		# Distance between left and right wheels (in meters)
        # self.wheel_separation_width = 0.208  # Based on spec
        self.wheel_separation_width = 0.215 # Measured  

        # conversion factor from meter to ticks 
        # MD520  1:56 has Encoder counts per output shaft turn 360 
        # Wheel is 80 mm diameter.
        # circumference is then 2 * PI * Radius ==> 251,28 mm per turn
        # 1000 mm / 251,28 mm * 2464 ticks per round  ==> 9805.794333 ticks per meter
        # The speed of the motor is 205 +-10 RPM (revolutions per minute)
        # So the maximum speed of the car is 205 * 0.25128 m / 60 s = 0.85854 m/s
        # self.ticks_per_revolution = 2464.0	# Encoder ticks per round based on spec
        self.ticks_per_revolution = 2444.0	# Measured Encoder ticks per round 

        # --- Per‑wheel calibration multipliers ---
        self.front_left_scale = 1.0
        self.front_right_scale = 1.0
        self.rear_left_scale = 1.0
        self.rear_right_scale = 1.0
        
        # remember the position for the odometry calculation
        self.x = 0.0
        self.y = 0.0    
        self.theta = 0.0

        # Initialize car control
        self.RA2DE = 180 / pi  # Radians to degrees conversion factor
        self.car = Rosmaster()
        self.car.set_car_type(2)  # Set car type to X3Plus

        # React to dynamic parameter changes (parameter callback)
        self.add_on_set_parameters_callback(self.on_param_change)

        # Declare and retrieve ROS2 parameters
        self.declare_parameter('car_type', 'X3Plus')
        self.car_type = self.get_parameter('car_type').get_parameter_value().string_value
        print(self.car_type)

        self.declare_parameter('imu_link', 'imu_link')
        self.imu_link = self.get_parameter('imu_link').get_parameter_value().string_value
        print(self.imu_link)

        self.declare_parameter('Prefix', "")
        self.Prefix = self.get_parameter('Prefix').get_parameter_value().string_value
        print(self.Prefix)

        self.declare_parameter('xlinear_limit', 1.0)
        self.xlinear_limit = self.get_parameter('xlinear_limit').get_parameter_value().double_value
        print(self.xlinear_limit)

        self.declare_parameter('ylinear_limit', 1.0)
        self.ylinear_limit = self.get_parameter('ylinear_limit').get_parameter_value().double_value
        print(self.ylinear_limit)

        self.declare_parameter('angular_limit', 5.0)
        self.angular_limit = self.get_parameter('angular_limit').get_parameter_value().double_value
        print(self.angular_limit)

        self.declare_parameter('wheel_radius', self.wheel_radius)
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        print(self.wheel_radius)    

        self.declare_parameter('wheel_separation_length', self.wheel_separation_length)
        self.wheel_separation_length = self.get_parameter('wheel_separation_length').get_parameter_value().double_value
        print(self.wheel_separation_length)    
    
        self.declare_parameter('wheel_separation_width', self.wheel_separation_width)
        self.wheel_separation_width = self.get_parameter('wheel_separation_width').get_parameter_value().double_value
        print(self.wheel_separation_width)    

        self.declare_parameter('ticks_per_revolution', self.ticks_per_revolution)
        self.ticks_per_revolution = self.get_parameter('ticks_per_revolution').get_parameter_value().double_value
        print(self.ticks_per_revolution)    

        self.declare_parameter('front_left_scale', self.front_left_scale)
        self.front_left_scale = self.get_parameter('front_left_scale').get_parameter_value().double_value
        print(self.front_left_scale)    

        self.declare_parameter('front_right_scale', self.front_right_scale)
        self.front_right_scale = self.get_parameter('front_right_scale').get_parameter_value().double_value
        print(self.front_right_scale)    

        self.declare_parameter('rear_left_scale', self.rear_left_scale)
        self.rear_left_scale = self.get_parameter('rear_left_scale').get_parameter_value().double_value
        print(self.rear_left_scale)    

        self.declare_parameter('rear_right_scale', self.rear_right_scale)
        self.rear_right_scale = self.get_parameter('rear_right_scale').get_parameter_value().double_value
        print(self.rear_right_scale)    

    
        # Create subscribers
        self.sub_cmd_vel = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 1)
        self.sub_Buzzer = self.create_subscription(Bool, "Buzzer", self.Buzzercallback, 100) 

        # Create publishers
        self.EdiPublisher = self.create_publisher(Float32, "edition", 100)
        self.volPublisher = self.create_publisher(Float32, "voltage", 100)
        self.velPublisher = self.create_publisher(Twist, "vel_raw", 50)
        self.imuPublisher = self.create_publisher(Imu, "/imu/data_raw", 100)
        self.magPublisher = self.create_publisher(MagneticField, "/imu/mag", 100)
        self.odomPublisher = self.create_publisher(Odometry, "/wheel/odometry", 100)
        self.jointStatePublisher = self.create_publisher(JointState, 'joint_states', 10)

        # Create a timer for periodic data publishing
        self.timer = self.create_timer(0.1, self.pub_data)

        # Initialize variables
        self.edition = Float32()
        self.edition.data = 1.0
        self.car.create_receive_threading()

    def compute_ticks_per_meter(self) -> float:
        # conversion factor from meter to ticks 
        # MD520  1:56 has Encoder counts per output shaft turn 360 
        # Wheel is 80 mm diameter.
        # circumference is then 2 * PI * Radius ==> 251,28 mm per turn
        # 1000 mm / 251,28 mm * 2464 ticks per round  ==> 9805.794333 ticks per meter
        #print ("compute_ticks_per_meter")
        #print (self.wheel_radius)
        #print (self.ticks_per_revolution)
        return  1.0 / (2.0 * pi * self.wheel_radius) * self.ticks_per_revolution

    def cmd_vel_callback(self, msg):
        """
        Callback for processing velocity commands.

        Parameters:
        -----------
        - msg (Twist): Velocity command message.
        """
        if not isinstance(msg, Twist):
            return
        vx = msg.linear.x * 1.0
        vy = msg.linear.y * 1.0
        angular = msg.angular.z * 1.0
        self.car.set_car_motion(vx, vy, angular)

    def RGBLightcallback(self, msg):
        """
        Callback for controlling RGB lights.

        Parameters:
        -----------
        - msg (Int32): RGB light control message.
        """
        if not isinstance(msg, Int32):
            return
        for i in range(3):
            self.car.set_colorful_effect(msg.data, 6, parm=1)

    def Buzzercallback(self, msg):
        """
        Callback for controlling the buzzer.

        Parameters:
        -----------
        - msg (Bool): Buzzer control message.
        """
        if not isinstance(msg, Bool):
            return
        if msg.data:
            for i in range(3):
                self.car.set_beep(1)
        else:
            for i in range(3):
                self.car.set_beep(0)

    @staticmethod
    def update_position(x, y, theta, Vx, Vy, omega, dt):
        x_new = x + (Vx * np.cos(theta) - Vy * np.sin(theta)) * dt
        y_new = y + (Vx * np.sin(theta) + Vy * np.cos(theta)) * dt
        theta_new = theta + omega * dt
        return x_new, y_new, theta_new

    def pub_data(self):
        """
        Periodically publishes sensor data (IMU, magnetic field, velocity, etc.).
        """
        time_stamp = Clock().now()
        imu = Imu()
        twist = Twist()
        battery = Float32()
        edition = Float32()
        mag = MagneticField()
        state = JointState()
        odom = Odometry()

        # Populate sensor data
      

        edition.data = self.car.get_version() * 1.0
        battery.data = self.car.get_battery_voltage() * 1.0
        ax, ay, az = self.car.get_accelerometer_data()
        gx, gy, gz = self.car.get_gyroscope_data()
        mx, my, mz = self.car.get_magnetometer_data()
        vx, vy, angular = self.car.get_motion_data()
        # m1 = front_left_encoder
        # m2 = rear_left_encoder
        # m3 = front_right_encoder
		# m4 = rear_right_encoder
        front_left_encoder, rear_left_encoder, front_right_encoder, rear_right_encoder = self.car.get_motor_encoder()

        # Populate IMU data
        imu.header.stamp = time_stamp.to_msg()
        imu.header.frame_id = self.imu_link
        imu.linear_acceleration.x = ax * 1.0
        imu.linear_acceleration.y = ay * 1.0
        imu.linear_acceleration.z = az * 1.0
        imu.angular_velocity.x = gx * 1.0
        imu.angular_velocity.y = gy * 1.0
        imu.angular_velocity.z = gz * 1.0

        # orientation quaternion already set here… 
        imu.orientation_covariance = [ 
            99999.0, 0.0, 0.0, 
            0.0, 99999.0, 0.0, 
            0.0, 0.0, 0.05 # yaw 
        ] 
        imu.angular_velocity_covariance = [ 
            99999.0, 0.0, 0.0, 
            0.0, 99999.0, 0.0, 
            0.0, 0.0, 0.02 # yaw rate 
        ] 
        imu.linear_acceleration_covariance = [ 
               0.5, 0.0, 0.0, 
               0.0, 0.5, 0.0, 
               0.0, 0.0, 0.5 
        ]

        # Populate magnetic field data
        mag.header.stamp = time_stamp.to_msg()
        mag.header.frame_id = self.imu_link
        mag.magnetic_field.x = mx * 1.0
        mag.magnetic_field.y = my * 1.0
        mag.magnetic_field.z = mz * 1.0



        # Populate velocity data
        twist.linear.x = vx * 1.0
        twist.linear.y = vy * 1.0
        twist.angular.z = angular * 1.0


		#   //  next, we'll publish the odometry message over ROS 2
		#   nav_msgs::msg::Odometry odom;
		#   odom.header.stamp = current_time;
		#   odom.header.frame_id = "odom";

		#   //  set the position
		#   odom.pose.pose.position.x = x;
		#   odom.pose.pose.position.y = y;
		#   odom.pose.pose.position.z = 0.0;
		#   odom.pose.pose.orientation = odom_quat;

		#   //  set the velocity
		#   odom.child_frame_id = "base_link";
		#   odom.twist.twist.linear.x = vel_xy;
		#   odom.twist.twist.linear.y = 0;
		#   odom.twist.twist.angular.z = vel_th;

        # See https://research.ijcaonline.org/volume113/number3/pxc3901586.pdf for the odometry calculation
        # calculate the delta time
        dt = (time_stamp - self.last_time_stamp).nanoseconds / 1e9  # Convert to seconds
        if dt <= 0.0:
            # nothing to integrate; update timestamp and encoder baselines and skip
            self.get_logger().warning(f"Non-positive dt in odometry: {dt}. Skipping update.")
            self.last_time_stamp = time_stamp
            self.front_left_encoder_old = front_left_encoder
            self.front_right_encoder_old = front_right_encoder
            self.rear_left_encoder_old = rear_left_encoder
            self.rear_right_encoder_old = rear_right_encoder
            return
        self.last_time_stamp = time_stamp

        # calculate the distance traveled by each wheel (meters)
        dfl = (front_left_encoder - self.front_left_encoder_old) / self.compute_ticks_per_meter() * self.front_left_scale
        dfr = (front_right_encoder - self.front_right_encoder_old) / self.compute_ticks_per_meter() * self.front_right_scale
        drl = (rear_left_encoder - self.rear_left_encoder_old) / self.compute_ticks_per_meter() * self.rear_left_scale
        drr = (rear_right_encoder - self.rear_right_encoder_old) / self.compute_ticks_per_meter() * self.rear_right_scale

        # calculate the average distance traveled by the robot (meters)
        dx = (dfl + dfr + drl + drr) / 4.0
        dy = (-dfl + dfr + drl - drr) / 4.0
        dtheta = (-dfl + dfr - drl + drr) / (2.0 * (self.wheel_separation_width + self.wheel_separation_length) )

        # log for debugging
        self.get_logger().debug(f"odom dt={dt:.6f} dfl={dfl:.6f} dfr={dfr:.6f} drl={drl:.6f} drr={drr:.6f} dx={dx:.6f} dy={dy:.6f} dtheta={dtheta:.6f}")

        # update pose (dx/dy are distances for the interval; do NOT multiply by dt)
        self.x += (dx * np.cos(self.theta) - dy * np.sin(self.theta))
        self.y += (dx * np.sin(self.theta) + dy * np.cos(self.theta))
        self.theta += dtheta
        self.theta = (self.theta + np.pi) % (2 * np.pi) - np.pi  # Normalize theta to [-pi, pi]

        # compute velocities for odometry (m/s, rad/s)
        vx_odom = dx / dt
        vy_odom = dy / dt
        vth_odom = dtheta / dt
        self.get_logger().debug(f"odom vx={vx_odom:.6f} vy={vy_odom:.6f} vth={vth_odom:.6f}")

        # remember the old encoder values
        self.front_left_encoder_old = front_left_encoder
        self.front_right_encoder_old = front_right_encoder
        self.rear_left_encoder_old = rear_left_encoder
        self.rear_right_encoder_old = rear_right_encoder

        # calculate the average distance traveled by the robot

  		#  calculate odometry
        roll = 0.0
        pitch = 0.0
        yaw = self.theta  # degrees in radians
        quat = quaternion_from_euler(roll, pitch, yaw)

  		# Populate odometry message
        odom.header.stamp = time_stamp.to_msg()
        odom.header.frame_id = "odom"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])

        #  set the velocity
        odom.child_frame_id = "base_footprint"
        odom.twist.twist.linear.x =  vx_odom
        odom.twist.twist.linear.y = vy_odom
        odom.twist.twist.angular.z = vth_odom

        # When you publish an nav_msgs/Odometry message without a covariance, ROS 2 fills the covariance with all zeros. 
        # And for robot_localization, a zero covariance means:
        # “This measurement is perfectly certain.”
        # The EKF treats that as invalid and rejects the message.
        # If all your inputs have zero covariances, the filter never initializes and therefore never publishes.
        
        # For an omnidirectional base, you can usually assume similar uncertainty in x and y, and treat z/roll/pitch as “don’t care” with huge covariances.
        odom.pose.covariance = [
            0.02, 0.0,    0.0,     0.0,     0.0,     0.0,
            0.0,    0.02, 0.0,     0.0,     0.0,     0.0,
            0.0,    0.0,    99999.0, 0.0,     0.0,     0.0,
            0.0,    0.0,    0.0,     99999.0, 0.0,     0.0,
            0.0,    0.0,    0.0,     0.0,     99999.0, 0.0,
            0.0,    0.0,    0.0,     0.0,     0.0,     0.05
        ]

        # Twist covariance for odom
        # For an omni robot, both vx and vy velocities more than integrated pose:
        
        odom.twist.covariance = [
            0.01, 0.0,    0.0,     0.0,     0.0,     0.0,
            0.0,    0.01, 0.0,     0.0,     0.0,     0.0,
            0.0,    0.0,    99999.0, 0.0,     0.0,     0.0,
            0.0,    0.0,    0.0,     99999.0, 0.0,     0.0,
            0.0,    0.0,    0.0,     0.0,     99999.0, 0.0,
            0.0,    0.0,    0.0,     0.0,     0.0,     0.02
        ]


        # Calulate joint states
        state.header.stamp = time_stamp.to_msg()
        state.header.frame_id = "joint_states"
        state.name = [
            self.Prefix + "front_right_joint",
            self.Prefix + "front_left_joint",
            self.Prefix + "back_right_joint",
            self.Prefix + "back_left_joint",
        ]
        state.position = [
            self.front_right_encoder_old / self.ticks_per_revolution * 2.0 * pi,
            self.front_left_encoder_old / self.ticks_per_revolution * 2.0 * pi,
            self.rear_right_encoder_old / self.ticks_per_revolution * 2.0 * pi,
            self.rear_left_encoder_old / self.ticks_per_revolution * 2.0 * pi,
        ]

        # Publish data
        self.velPublisher.publish(twist)
        self.imuPublisher.publish(imu)
        self.magPublisher.publish(mag)
        self.volPublisher.publish(battery)
        self.EdiPublisher.publish(edition)
        self.odomPublisher.publish(odom)
        self.jointStatePublisher.publish(state)

    def cleanup(self):
        """
        Cleans up resources and stops the robot when shutting down the node.
        """
        self.car.set_car_motion(0, 0, 0)
        self.velPublisher.unregister()
        self.imuPublisher.unregister()
        self.EdiPublisher.unregister()
        self.volPublisher.unregister()
        self.magPublisher.unregister()
        self.odomPublisher.unregister()
        self.sub_cmd_vel.unregister()
        self.sub_Buzzer.unregister()
        rclpy.loginfo("Close the robot...")
        rclpy.sleep(1)

    def on_param_change(self, params):
        for p in params:
            if p.name == "front_left_scale":
                self.front_left_scale = p.value
                self.get_logger().info(f"Updated front_left_scale → {p.value}")

            elif p.name == "front_right_scale":
                self.front_right_scale = p.value
                self.get_logger().info(f"Updated front_right_scale → {p.value}")

            elif p.name == "rear_left_scale":
                self.rear_left_scale = p.value
                self.get_logger().info(f"Updated rear_left_scale → {p.value}")

            elif p.name == "rear_right_scale":
                self.rear_right_scale = p.value
                self.get_logger().info(f"Updated rear_right_scale → {p.value}")

            elif p.name == "wheel_radius":
                self.wheel_radius = p.value
                self.get_logger().info(f"Updated wheel_radius → {p.value}")
                
            elif p.name == "wheel_separation_width":
                self.wheel_separation_width = p.value
                self.get_logger().info(f"Updated wheel_separation_width → {p.value}")
                
            elif p.name == "wheel_separation_length":
                self.wheel_separation_length = p.value
                self.get_logger().info(f"Updated wheel_separation_length → {p.value}")
                
            elif p.name == "ticks_per_revolution":
                self.ticks_per_revolution = p.value
                self.get_logger().info(f"Updated ticks_per_revolution → {p.value}")                

        return SetParametersResult(successful=True)


def main():
    """
    Main function to initialize and run the ROS2 node.
    """
    rclpy.init()
    driver = yahboomcar_driver('driver_node')

    # Register the cleanup method to be called on shutdown
    rclpy.get_default_context().on_shutdown(driver.cleanup)
    rclpy.spin(driver)




	#callback function
def cmd_vel_callback(self,msg):
        # Car motion control, subscriber callback function
		if not isinstance(msg, Twist): return
        # Issue linear vel and angular vel
		vx = msg.linear.x*1.0
        #vy = msg.linear.y/1000.0*180.0/3.1416    #Radian system
		vy = msg.linear.y*1.0
		angular = msg.angular.z*1.0     # wait for chang
		self.car.set_car_motion(vx, vy, angular)
		'''
		print("cmd_vx: ",vx)
		print("cmd_vy: ",vy)
		print("cmd_angular: ",angular)
		'''
        #rclpy.loginfo("nav_use_rot:{}".format(self.nav_use_rotvel))
        #print(self.nav_use_rotvel)
def RGBLightcallback(self,msg):
		if not isinstance(msg, Int32): return
		# print ("RGBLight: ", msg.data)
		for i in range(3): self.car.set_colorful_effect(msg.data, 6, parm=1)
	
def Buzzercallback(self,msg):
		if not isinstance(msg, Bool): return
		if msg.data:
			for i in range(3): self.car.set_beep(1)
		else:
			for i in range(3): self.car.set_beep(0)

	#pub data
def pub_data(self):
		time_stamp = Clock().now()
		imu = Imu()
		twist = Twist()
		battery = Float32()
		edition = Float32()
		mag = MagneticField()
		state = JointState()
		state.header.stamp = time_stamp.to_msg()
		state.header.frame_id = "joint_states"
		if len(self.Prefix)==0:
			state.name = ["back_right_joint", "back_left_joint","front_left_steer_joint","front_left_wheel_joint",
							"front_right_steer_joint", "front_right_wheel_joint"]
		else:
			state.name = [self.Prefix+"back_right_joint",self.Prefix+ "back_left_joint",self.Prefix+"front_left_steer_joint",self.Prefix+"front_left_wheel_joint",
							self.Prefix+"front_right_steer_joint", self.Prefix+"front_right_wheel_joint"]
		
		#print ("mag: ",self.car.get_magnetometer_data())		
		edition.data = self.car.get_version()*1.0
		battery.data = self.car.get_battery_voltage()*1.0
		ax, ay, az = self.car.get_accelerometer_data()
		gx, gy, gz = self.car.get_gyroscope_data()
		mx, my, mz = self.car.get_magnetometer_data()
		mx = mx * 1.0
		my = my * 1.0
		mz = mz * 1.0
		vx, vy, angular = self.car.get_motion_data()
		'''print("vx: ",vx)
		print("vy: ",vy)
		print("angular: ",angular)'''
		# Publish gyroscope data
		imu.header.stamp = time_stamp.to_msg()
		imu.header.frame_id = self.imu_link
		imu.linear_acceleration.x = ax*1.0
		imu.linear_acceleration.y = ay*1.0
		imu.linear_acceleration.z = az*1.0
		imu.angular_velocity.x = gx*1.0
		imu.angular_velocity.y = gy*1.0
		imu.angular_velocity.z = gz*1.0

		mag.header.stamp = time_stamp.to_msg()
		mag.header.frame_id = self.imu_link
		mag.magnetic_field.x = mx*1.0
		mag.magnetic_field.y = my*1.0
		mag.magnetic_field.z = mz*1.0
		
		# Publish the current linear vel and angular vel of the car
		twist.linear.x = vx *1.0
		twist.linear.y = vy *1.0
		twist.angular.z = angular*1.0    
		self.velPublisher.publish(twist)
		# print("ax: %.5f, ay: %.5f, az: %.5f" % (ax, ay, az))
		# print("gx: %.5f, gy: %.5f, gz: %.5f" % (gx, gy, gz))
		# print("mx: %.5f, my: %.5f, mz: %.5f" % (mx, my, mz))
		# rclpy.loginfo("battery: {}".format(battery))
		# rclpy.loginfo("vx: {}, vy: {}, angular: {}".format(twist.linear.x, twist.linear.y, twist.angular.z))
		self.imuPublisher.publish(imu)
		self.magPublisher.publish(mag)
		self.volPublisher.publish(battery)
		self.EdiPublisher.publish(edition)
		
def cleanup(self):
		self.car.set_car_motion(0, 0, 0)
		self.velPublisher.unregister()
		self.imuPublisher.unregister()
		self.EdiPublisher.unregister()
		self.volPublisher.unregister()
        ## TODO: self.staPublisher.unregister()
		self.magPublisher.unregister()
		self.sub_cmd_vel.unregister()
		## TODO: self.sub_RGBLight.unregister()
		self.sub_Buzzer.unregister()
        # Always stop the robot when shutting down the node
		rclpy.loginfo("Close the robot...")
		rclpy.sleep(1)
			
def main():
	rclpy.init() 
	driver = yahboomcar_driver('x3plus_wrapper')

	# Register the cleanup method to be called on shutdown
	rclpy.get_default_context().on_shutdown(driver.cleanup)
	rclpy.spin(driver)

'''if __name__ == '__main__':
	main()'''

		
		
