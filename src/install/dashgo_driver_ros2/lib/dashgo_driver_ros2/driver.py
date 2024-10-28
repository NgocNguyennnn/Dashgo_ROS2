#!/usr/bin/env python3

import rclpy
import rclpy.duration
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
import os, time
from time import sleep
import threading
from threading import Lock

from math import pi as PI, degrees, radians, sin, cos
import os
import time
import sys, traceback
import serial
from serial.serialutil import SerialException
from serial import Serial

# import roslib
import math

from geometry_msgs.msg import Twist, Quaternion, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Int16,Int32
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

from std_srvs.srv import Trigger

from sensor_msgs.msg import Range, PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2
# import sensor_msgs.point_cloud2 as pc2

ODOM_POSE_COVARIANCE = [float(value) for value in[  1e-3, 0, 0, 0, 0, 0,
                                                    0, 1e-3, 0, 0, 0, 0,
                                                    0, 0, 1e6, 0, 0, 0,
                                                    0, 0, 0, 1e6, 0, 0,
                                                    0, 0, 0, 0, 1e6, 0,
                                                    0, 0, 0, 0, 0, 1e3]]
ODOM_POSE_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0,
                         0, 1e-3, 1e-9, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e-9]

ODOM_TWIST_COVARIANCE = [float(value) for value in[1e-3, 0, 0, 0, 0, 0,
                         0, 1e-3, 0, 0, 0, 0,
                         0, 0, 1e6, 0, 0, 0,
                         0, 0, 0, 1e6, 0, 0,
                         0, 0, 0, 0, 1e6, 0,
                         0, 0, 0, 0, 0, 1e3]]
ODOM_TWIST_COVARIANCE2 = [1e-9, 0, 0, 0, 0, 0,
                          0, 1e-3, 1e-9, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e-9]


SERVO_MAX = 180
SERVO_MIN = 0

class Arduino:
    ''' Configuration Parameters
    '''
    N_ANALOG_PORTS = 6
    N_DIGITAL_PORTS = 12

    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=0.5):

        self.PID_RATE = 30 # Do not change this!  It is a fixed property of the Arduino PID controller.
        self.PID_INTERVAL = 1000 / 30

        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.encoder_count = 0
        self.writeTimeout = timeout
        self.interCharTimeout = timeout / 30.

        # Keep things thread safe
        self.mutex = threading.Lock()

        # An array to cache analog sensor readings
        self.analog_sensor_cache = [None] * self.N_ANALOG_PORTS

        # An array to cache digital sensor readings
        self.digital_sensor_cache = [None] * self.N_DIGITAL_PORTS

    def connect(self):
        try:
            print ("Connecting to Arduino on port", self.port, "...")
            # self.serial_connection = Serial(
            #     port=self.port,
            #     baudrate=self.baudrate,
            #     timeout=self.timeout,
            #     write_timeout=self.writeTimeout
            # )
            self.serial_connection = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                write_timeout=self.writeTimeout
            )
            # The next line is necessary to give the firmware time to wake up.
            time.sleep(1)
            test = self.get_baud()
            print ("Received Baudrate 1: ",test)
            if test != self.baudrate:
                time.sleep(1)
                test = self.get_baud()
                print ("Received Baudrate 2: ",test)
                if test != self.baudrate:
                    # raise SerialException
                    raise SerialException("Baudrate mismatch")
            print ("Connected at", self.baudrate)
            print ("Arduino is ready.")

        except SerialException:
            # print ("Serial Exception:")
            # print (sys.exc_info())
            # print ("Traceback follows:")
            # traceback.print_exc(file=sys.stdout)
            # print ("Cannot connect to Arduino!")
            # os._exit(1)
            print("Serial Exception:")
            traceback.print_exc()
            print("Cannot connect to Arduino!")
            os._exit(1)

    def open(self):
        ''' Open the serial port.
        '''
        self.serial_connection.open()

    def close(self):
        ''' Close the serial port.
        '''
        self.serial_connection.close()

    def send(self, cmd):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        # self.serial_connection.write(cmd + '\r')
        # self.serial_connection.write(cmd.encode() + b'\r')
        self.serial_connection.write((cmd + '\r').encode())
        # self.serial_connection.write(cmd.encode())

    def recv(self, timeout=0.5):
        timeout = min(timeout, self.timeout)
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.  Note: we use read() instead of readline() since
            readline() tends to return garbage characters from the Arduino
        '''
        c = b''
        value = b''
        attempts = 0
        while c != b'\r':
            c = self.serial_connection.read(1)
            value += c
            attempts += 1
            # print(f"Received byte: {c}")  # Ghi log byte nhận được
            if attempts * self.interCharTimeout > timeout:
                return None

        # value = value.strip(b'\r')

        # return value.decode()
        return value.strip(b'\r').decode()

    def recv_ack(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        ack = self.recv(self.timeout)
        return ack == 'OK'

    def recv_int(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        value = self.recv(self.timeout)
        try:
            return int(value)
        except:
            return None

    def recv_array(self):
        ''' This command should not be used on its own: it is called by the execute commands
            below in a thread safe manner.
        '''
        try:
            values = self.recv(self.timeout * self.N_ANALOG_PORTS).split()
            return list(map(int, values))
        except:
            return []

    def execute(self, cmd):
        ''' Thread-safe execution of "cmd" on the Arduino returning a single integer value. '''
        with self.mutex:
            try:
                self.serial_connection.flushInput()
                self.send(cmd)
                value = self.recv()
                if value is None:
                    print(f"Timeout or invalid response for command '{cmd}'")
                    return None
                return int(value) if value.isdigit() else None
            except Exception as e:
                print(f"Error flushing input: {e}")
                pass
                # print(f"Exception executing command '{cmd}': {e}")
                # return None

            ntries = 1
            attempts = 0
            value = None

            try:
                # Gửi lệnh tới Arduino
                self.send(cmd)
                value = self.recv()
                
                # Thử lại nếu không nhận được giá trị hợp lệ
                while attempts < ntries and (value == '' or value == 'Invalid Command' or value is None):
                    try:
                        self.serial_connection.flushInput()  # Xóa buffer
                        self.send(cmd)  # Gửi lại lệnh
                        value = self.recv()  # Nhận lại dữ liệu
                    except Exception as e:
                        print(f"Exception executing command '{cmd}' on attempt {attempts + 1}: {e}")
                    attempts += 1

            except Exception as e:
                print(f"Exception executing command '{cmd}': {e}")
                value = None

            # Trả về giá trị dưới dạng số nguyên hoặc None nếu thất bại
            try:
                return int(value) if value is not None else None
            except ValueError:
                print(f"Invalid response for command '{cmd}': {value}")
                return None


    def execute_array(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning an array. '''
        with self.mutex:  # This automatically acquires and releases the lock
            try:
                self.serial_connection.flushInput()
            except Exception as e:
                self.get_logger().warn(f"Exception during flushInput: {str(e)}")

            ntries = 3
            attempts = 0

            try:
                self.send(cmd)
                values = self.recv_array()

                while attempts < ntries and (not values or values in ['', 'Invalid Command']):
                    try:
                        self.serial_connection.flushInput()
                        self.send(cmd )
                        values = self.recv_array()
                    except Exception as e:
                        self.get_logger().warn(f"Exception executing command: {cmd} - {str(e)}")
                    attempts += 1

            except Exception as e:
                self.get_logger().error(f"Exception executing command: {cmd} - {str(e)}")
                raise SerialException
                return []

            try:
                values = list(map(int, values))
            except (ValueError, TypeError):
                self.get_logger().warn("Failed to convert values to integers")
                values = []

        return values

    def execute_ack(self, cmd):
        ''' Thread safe execution of "cmd" on the Arduino returning True if response is ACK.
        '''
        self.mutex.acquire()

        try:
            self.serial_connection.flushInput()
        except:
            pass

        ntries = 1
        attempts = 0

        try:
            self.send(cmd)
            # print ("CMD:" + cmd)
            ack = self.recv(self.timeout)
            while attempts < ntries and (ack == '' or ack == 'Invalid Command' or ack == None):
                try:
                    self.serial_connection.flushInput()
                    self.send(cmd)
                    ack = self.recv(self.timeout)
                except:
                    print ("Exception executing command: " + cmd)
            attempts += 1
        except:
            self.mutex.release()
            print ("execute_ack exception when executing", cmd)
            print (sys.exc_info())
            return 0

        self.mutex.release()
        return ack == 'OK'

    def update_pid(self, Kp, Kd, Ki, Ko):
        ''' Set the PID parameters on the Arduino
        '''
        print ("Updating PID parameters")
        cmd = f'u {Kp}:{Kd}:{Ki}:{Ko}'
        self.execute_ack(cmd)

    def get_baud(self):
        ''' Get the current baud rate on the serial port.
        '''
        # return int(self.execute('b'))
        return self.execute('b')

    def get_encoder_counts(self):
        values = self.execute_array('e')
        if len(values) != 2:
            print ("Encoder count was not 2")
            raise SerialException
            return None
        else:
            return values

    def reset_encoders(self):
        ''' Reset the encoder counts to 0
        '''
        return self.execute_ack('r')

    def drive(self, right, left):
        ''' Speeds are given in encoder ticks per PID interval
        '''
        # print("Speed right: " + right)
        # print("Speed left: " + left)
        return self.execute_ack(f'm {right} {left}')

    def drive_m_per_s(self, right, left):
        ''' Set the motor speeds in meters per second.
        '''
        left_revs_per_second = float(left) / (self.wheel_diameter * PI)
        right_revs_per_second = float(right) / (self.wheel_diameter * PI)

        left_ticks_per_loop = int(left_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)
        right_ticks_per_loop  = int(right_revs_per_second * self.encoder_resolution * self.PID_INTERVAL * self.gear_reduction)

        self.drive(right_ticks_per_loop , left_ticks_per_loop )

    def stop(self):
        ''' Stop both motors.
        '''
        self.drive(0, 0)

    def ping(self):
        values = self.execute_array('p')
        if len(values) != 4:
            print ("ping count was not 4")
            raise SerialException
            return None
        else:
            return values

    def get_voltage(self):
        return self.execute('v')

    def get_emergency_button(self):
        return self.execute('j')

    def get_pidin(self):
        values = self.execute_array('i')
        if len(values) != 2:
            print ("get_pidin count was not 2")
            raise SerialException
            return None
        else:
            return values

    def get_pidout(self):
        values = self.execute_array('f')
        if len(values) != 2:
            print ("get_pidout count was not 2")
            raise SerialException
            return None
        else:
            return values

    def get_arduino_version(self):
        values = self.execute_array('V')
        if len(values) != 2:
            print ("get arduino version error")
            raise SerialException
            return None
        else:
            return values


""" Class to receive Twist commands and publish Odometry data """
class BaseController(Node):
    def __init__(self, arduino, base_frame):
        super().__init__('base_controller')
        self.arduino = arduino
        self.base_frame = base_frame
        self.rate = self.declare_parameter("base_controller_rate", 10.0).get_parameter_value().double_value
        self.timeout = self.declare_parameter("base_controller_timeout", 1.0).get_parameter_value().double_value
        self.stopped = False
        self.useImu = self.declare_parameter("useImu", False).get_parameter_value().bool_value
        self.useSonar = self.declare_parameter("useSonar", False).get_parameter_value().bool_value

        pid_params = {
            'wheel_diameter': self.declare_parameter("wheel_diameter", 0.128).get_parameter_value().double_value,
            'wheel_track': self.declare_parameter("wheel_track", 0.3559).get_parameter_value().double_value,
            'encoder_resolution': self.declare_parameter("encoder_resolution", 1200).get_parameter_value().double_value,
            'gear_reduction': self.declare_parameter("gear_reduction", 1.0).get_parameter_value().double_value,
            'Kp': self.declare_parameter("Kp", 20).get_parameter_value().double_value,
            'Kd': self.declare_parameter("Kd", 12).get_parameter_value().double_value,
            'Ki': self.declare_parameter("Ki", 0).get_parameter_value().double_value,
            'Ko': self.declare_parameter("Ko", 50).get_parameter_value().double_value
        }

        self.accel_limit = self.declare_parameter('accel_limit', 0.1).get_parameter_value().double_value
        self.motors_reversed = self.declare_parameter("motors_reversed", False).get_parameter_value().bool_value

        # Set up PID parameters and check for missing values
        self.setup_pid(pid_params)

        self.encoder_resolution = float(1200)
        # How many encoder ticks are there per meter?

        self.ticks_per_meter = self.encoder_resolution * self.gear_reduction  / (self.wheel_diameter * PI)

        # What is the maximum acceleration we will tolerate when changing wheel speeds?
        self.max_accel = self.accel_limit * self.ticks_per_meter / self.rate

        # Track how often we get a bad encoder count (if any)
        self.bad_encoder_count = 0

        self.encoder_min = self.declare_parameter('encoder_min', -32768).get_parameter_value().integer_value
        self.encoder_max = self.declare_parameter('encoder_max', 32768).get_parameter_value().integer_value
        self.encoder_low_wrap = self.declare_parameter('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min).get_parameter_value().double_value
        self.encoder_high_wrap = self.declare_parameter('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min).get_parameter_value().double_value
        self.l_wheel_mult = 0
        self.r_wheel_mult = 0

        now = self.get_clock().now()
        self.then = now # time for determining dx/dy
        self.t_delta = rclpy.duration.Duration(seconds=1.0 / self.rate)
        self.t_next = self.get_clock().now() + self.t_delta


        # Internal data
        self.enc_left = None            # encoder readings
        self.enc_right = None
        self.x = 0                      # position in xy plane
        self.y = 0
        self.th = 0                     # rotation in radians
        self.v_left = 0
        self.v_right = 0
        self.v_des_left = 0             # cmd_vel setpoint
        self.v_des_right = 0
        self.last_cmd_vel = now

        # Subscriptions
        self.create_subscription(Twist, "smoother_cmd_vel", self.cmdVelCallback, 10)
        # self.create_subscription(Twist,"cmd_vel",self.cmdVelCallback, 10)
        # Clear any old odometry info
        self.arduino.reset_encoders()

        # Set up the odometry broadcaster
        self.odomPub = self.create_publisher(Odometry, 'odom', 10)
        self.odomBroadcaster = TransformBroadcaster(self)

        self.lEncoderPub = self.create_publisher(Int16, 'Lencoder', 10)
        self.rEncoderPub = self.create_publisher(Int16, 'Rencoder', 10)
        self.lVelPub = self.create_publisher(Int16, 'Lvel', 10)
        self.rVelPub = self.create_publisher(Int16, 'Rvel', 10)

        ## sonar
        self.sonar0_pub = self.create_publisher(Range, 'sonar0', 10)
        self.sonar1_pub = self.create_publisher(Range, 'sonar1', 10)
        self.sonar2_pub = self.create_publisher(Range, 'sonar2', 10)
        self.sonar3_pub = self.create_publisher(Range, 'sonar3', 10)

        self.sonar_r0 = 0.0
        self.sonar_r1 = 0.0
        self.sonar_r2 = 0.0
        self.sonar_r3 = 0.0

        self.safe_range_0 = 10
        self.safe_range_1 = 7

        self.sonar_pub_cloud = self.create_publisher(PointCloud2, "/sonar_cloudpoint", 10)

        self.sonar_height = self.declare_parameter("sonar_height", 0.115).get_parameter_value().double_value
        self.sonar_maxval = 3.5

        self.point_offset = self.declare_parameter("point_offset", 0.08).get_parameter_value().double_value
        self.sonar0_offset_yaw = self.declare_parameter("sonar0_offset_yaw", 0.524).get_parameter_value().double_value
        self.sonar0_offset_x = self.declare_parameter("sonar0_offset_x", 0.18).get_parameter_value().double_value
        self.sonar0_offset_y = self.declare_parameter("sonar0_offset_y", 0.10).get_parameter_value().double_value

        self.sonar1_offset_yaw = self.declare_parameter("sonar1_offset_yaw", 0.0).get_parameter_value().double_value
        self.sonar1_offset_x = self.declare_parameter("sonar1_offset_x", 0.20).get_parameter_value().double_value
        self.sonar1_offset_y = self.declare_parameter("sonar1_offset_y", 0.0).get_parameter_value().double_value

        self.sonar2_offset_yaw = self.declare_parameter("sonar2_offset_yaw", -0.524).get_parameter_value().double_value
        self.sonar2_offset_x = self.declare_parameter("sonar2_offset_x", 0.18).get_parameter_value().double_value
        self.sonar2_offset_y = self.declare_parameter("sonar2_offset_y", -0.10).get_parameter_value().double_value

        self.sonar3_offset_yaw = self.declare_parameter("sonar3_offset_yaw", 3.14).get_parameter_value().double_value
        self.sonar3_offset_x = self.declare_parameter("sonar3_offset_x", -0.20).get_parameter_value().double_value
        self.sonar3_offset_y = self.declare_parameter("sonar3_offset_y", 0.0).get_parameter_value().double_value


        self.sonar_cloud = [[100.0,0.105,0.1],[100.0,-0.105,0.1],[0.2,100.0,0.1],[0.2,-100.0,0.1]]

        self.sonar_cloud[0][0] = self.sonar0_offset_x + self.sonar_maxval * math.cos(self.sonar0_offset_yaw)
        self.sonar_cloud[0][1] = self.sonar0_offset_y + self.sonar_maxval * math.sin(self.sonar0_offset_yaw)
        self.sonar_cloud[0][2] = self.sonar_height

        self.sonar_cloud[1][0] = self.sonar1_offset_x + self.sonar_maxval * math.cos(self.sonar1_offset_yaw)
        self.sonar_cloud[1][1] = self.sonar1_offset_y + self.sonar_maxval * math.sin(self.sonar1_offset_yaw)
        self.sonar_cloud[1][2] = self.sonar_height

        self.sonar_cloud[2][0] = self.sonar2_offset_x + self.sonar_maxval * math.cos(self.sonar2_offset_yaw)
        self.sonar_cloud[2][1] = self.sonar2_offset_y + self.sonar_maxval * math.sin(self.sonar2_offset_yaw)
        self.sonar_cloud[2][2] = self.sonar_height

        self.sonar_cloud[3][0] = self.sonar3_offset_x + self.sonar_maxval * math.cos(self.sonar3_offset_yaw)
        self.sonar_cloud[3][1] = self.sonar3_offset_y + self.sonar_maxval * math.sin(self.sonar3_offset_yaw)
        self.sonar_cloud[3][2] = self.sonar_height

        self.voltage_val = 0
        self.voltage_pub = self.create_publisher(Int32, 'voltage_value', 10)

        self.emergencybt_val = 0
        self.emergencybt_pub = self.create_publisher(Int16, 'emergencybt_status', 10)

        self.create_subscription(Int16, "is_passed", self.isPassedCallback, 10)
        self.isPassed = True


    def setup_pid(self, pid_params):
        # Check to see if any PID parameters are missing
        missing_params = False
        for param in pid_params:
            if pid_params[param] == "":
                print("*** PID Parameter " + param + " is missing. ***")
                missing_params = True

        if missing_params:
            rclpy.shutdown()
            os._exit(1)

        self.wheel_diameter = pid_params['wheel_diameter']
        self.wheel_track = pid_params['wheel_track']
        self.encoder_resolution = pid_params['encoder_resolution']
        self.gear_reduction = pid_params['gear_reduction']

        self.Kp = 20
        self.Kd = 12
        self.Ki = 0
        self.Ko = 50

        self.arduino.update_pid(self.Kp, self.Kd, self.Ki, self.Ko)

    def poll(self):
        now = self.get_clock().now()
        # self.get_logger().info(f"Enter Poll Function")
        if now > self.t_next:
            self.get_logger().info(f"")
            if (self.useSonar == True) :
                pcloud = PointCloud2()
                try:
                    self.sonar_r0, self.sonar_r1, self.sonar_r2, self.sonar_r3= self.arduino.ping()
                    sonar0_range = Range()
                    sonar0_range.header.stamp = now
                    sonar0_range.header.frame_id = "/sonar0"
                    sonar0_range.radiation_type = Range.ULTRASOUND
                    sonar0_range.field_of_view = 0.3
                    sonar0_range.min_range = 0.03
                    sonar0_range.max_range = 0.8
                    sonar0_range.range = self.sonar_r0/100.0
                    if sonar0_range.range>=sonar0_range.max_range or sonar0_range.range == 0.0:
                        sonar0_range.range = sonar0_range.max_range
                    self.sonar0_pub.publish(sonar0_range)
                    if sonar0_range.range>=0.5 or sonar0_range.range == 0.0:
                        self.sonar_cloud[0][0] = self.sonar0_offset_x + self.sonar_maxval * math.cos(self.sonar0_offset_yaw)
                        self.sonar_cloud[0][1] = self.sonar0_offset_y + self.sonar_maxval * math.sin(self.sonar0_offset_yaw)
                    else:
                        self.sonar_cloud[0][0] = self.sonar0_offset_x + sonar0_range.range * math.cos(self.sonar0_offset_yaw)
                        self.sonar_cloud[0][1] = self.sonar0_offset_y + sonar0_range.range * math.sin(self.sonar0_offset_yaw)

                    sonar1_range = Range()
                    sonar1_range.header.stamp = now
                    sonar1_range.header.frame_id = "/sonar1"
                    sonar1_range.radiation_type = Range.ULTRASOUND
                    sonar1_range.field_of_view = 0.3
                    sonar1_range.min_range = 0.03
                    sonar1_range.max_range = 0.8
                    sonar1_range.range = self.sonar_r1/100.0
                    if sonar1_range.range>=sonar0_range.max_range or sonar1_range.range == 0.0:
                        sonar1_range.range = sonar1_range.max_range
                    self.sonar1_pub.publish(sonar1_range)
                    if sonar1_range.range>=0.5 or sonar1_range.range == 0.0:
                        self.sonar_cloud[1][0] = self.sonar1_offset_x + self.sonar_maxval * math.cos(self.sonar1_offset_yaw)
                        self.sonar_cloud[1][1] = self.sonar1_offset_y + self.sonar_maxval * math.sin(self.sonar1_offset_yaw)
                    else:
                        self.sonar_cloud[1][0] = self.sonar1_offset_x + sonar1_range.range * math.cos(self.sonar1_offset_yaw)
                        self.sonar_cloud[1][1] = self.sonar1_offset_y + sonar1_range.range * math.sin(self.sonar1_offset_yaw)

                    sonar2_range = Range()
                    sonar2_range.header.stamp = now
                    sonar2_range.header.frame_id = "/sonar2"
                    sonar2_range.radiation_type = Range.ULTRASOUND
                    sonar2_range.field_of_view = 0.3
                    sonar2_range.min_range = 0.03
                    sonar2_range.max_range = 0.8
                    sonar2_range.range = self.sonar_r2/100.0
                    if sonar2_range.range>=sonar2_range.max_range or sonar2_range.range == 0.0:
                        sonar2_range.range = sonar2_range.max_range
                    self.sonar2_pub.publish(sonar2_range)
                    if sonar2_range.range>=0.5 or sonar2_range.range == 0.0:
                        self.sonar_cloud[2][0] = self.sonar2_offset_x + self.sonar_maxval * math.cos(self.sonar2_offset_yaw)
                        self.sonar_cloud[2][1] = self.sonar2_offset_y + self.sonar_maxval * math.sin(self.sonar2_offset_yaw)
                    else:
                        self.sonar_cloud[2][0] = self.sonar2_offset_x + sonar2_range.range * math.cos(self.sonar2_offset_yaw)
                        self.sonar_cloud[2][1] = self.sonar2_offset_y + sonar2_range.range * math.sin(self.sonar2_offset_yaw)

                    sonar3_range = Range()
                    sonar3_range.header.stamp = now
                    sonar3_range.header.frame_id = "/sonar3"
                    sonar3_range.radiation_type = Range.ULTRASOUND
                    sonar3_range.field_of_view = 0.3
                    sonar3_range.min_range = 0.03
                    sonar3_range.max_range = 0.8
                    sonar3_range.range = self.sonar_r3/100.0
                    if sonar3_range.range>=sonar3_range.max_range or sonar3_range.range == 0.0:
                        sonar3_range.range = sonar3_range.max_range
                    self.sonar3_pub.publish(sonar3_range)
                    if sonar3_range.range>=0.5 or sonar3_range.range == 0.0:
                        self.sonar_cloud[3][0] = self.sonar3_offset_x + self.sonar_maxval * math.cos(self.sonar3_offset_yaw)
                        self.sonar_cloud[3][1] = self.sonar3_offset_y + self.sonar_maxval * math.sin(self.sonar3_offset_yaw)
                    else:
                        self.sonar_cloud[3][0] = self.sonar3_offset_x + sonar3_range.range * math.cos(self.sonar3_offset_yaw)
                        self.sonar_cloud[3][1] = self.sonar3_offset_y + sonar3_range.range * math.sin(self.sonar3_offset_yaw)
                except:
                    self.bad_encoder_count += 1
                    self.get_logger().error(f"Ping exception count: {self.bad_encoder_count}")
                    return

                pcloud.header.frame_id="base_footprint"
                # pcloud.header.frame_id="/base_footprint"
                pcloud = pc2.create_cloud_xyz32(pcloud.header, self.sonar_cloud)
                self.sonar_pub_cloud.publish(pcloud)

        try:
            self.voltage_val = self.arduino.get_voltage() * 10
            self.voltage_pub.publish(Int32(data=self.voltage_val))
        except:
            self.voltage_pub.publish(Int32(data=-1))

        try:
            emergency_button = self.arduino.get_emergency_button()
            if emergency_button is not None:
                self.emergencybt_val = int(emergency_button)
            else:
                self.emergencybt_val = 0 

            self.emergencybt_pub.publish(Int16(data=self.emergencybt_val))
        except:
            self.emergencybt_pub.publish(Int16(data=-1))

        try:
            left_enc, right_enc = self.arduino.get_encoder_counts()
            #rospy.loginfo("left_enc: " + str(left_enc)+"right_enc: " + str(right_enc))
            self.lEncoderPub.publish(Int16(data=left_enc))
            self.rEncoderPub.publish(Int16(data=right_enc))
        except:
            self.bad_encoder_count += 1
            self.get_logger().error(f"Encoder exception count: {self.bad_encoder_count}")
            return

        dt = (now - self.then).nanoseconds / 1e9
        self.then = now

            # Calculate odometrynow.sec
        if self.enc_left == None:
            dright = 0
            dleft = 0
        else:
            if (left_enc < self.encoder_low_wrap and self.enc_left > self.encoder_high_wrap) :
                self.l_wheel_mult = self.l_wheel_mult + 1
            elif (left_enc > self.encoder_high_wrap and self.enc_left < self.encoder_low_wrap) :
                self.l_wheel_mult = self.l_wheel_mult - 1
            else:
                    self.l_wheel_mult = 0
            if (right_enc < self.encoder_low_wrap and self.enc_right > self.encoder_high_wrap) :
                self.r_wheel_mult = self.r_wheel_mult + 1
            elif (right_enc > self.encoder_high_wrap and self.enc_right < self.encoder_low_wrap) :
                self.r_wheel_mult = self.r_wheel_mult - 1
            else:
                    self.r_wheel_mult = 0
            dleft = 1.0 * (left_enc + self.l_wheel_mult * (self.encoder_max - self.encoder_min)-self.enc_left) / self.ticks_per_meter
            dright = 1.0 * (right_enc + self.r_wheel_mult * (self.encoder_max - self.encoder_min)-self.enc_right) / self.ticks_per_meter
        

        self.enc_right = right_enc
        self.enc_left = left_enc

        dxy_ave = (dright + dleft) / 2.0
        dth = (dright - dleft) / self.wheel_track
        vxy = dxy_ave / dt
        vth = dth / dt

        if (dxy_ave != 0):
            dx = cos(dth) * dxy_ave
            dy = -sin(dth) * dxy_ave
            self.x += (cos(self.th) * dx - sin(self.th) * dy)
            self.y += (sin(self.th) * dx + cos(self.th) * dy)

        if (dth != 0):
            self.th += dth

        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = sin(self.th / 2.0)
        quaternion.w = cos(self.th / 2.0)

        
        transform = TransformStamped()

        
        transform.header.stamp = now.to_msg()          # Chuyển đổi sang thông điệp
        transform.header.frame_id = "odom"      # Khung toạ độ gốc
        transform.child_frame_id = self.base_frame  # Khung toạ độ con

        
        transform.transform.translation.x = float(self.x)
        transform.transform.translation.y = float(self.y)
        transform.transform.translation.z = 0.0

        
        transform.transform.rotation.x = quaternion.x
        transform.transform.rotation.y = quaternion.y
        transform.transform.rotation.z = quaternion.z
        transform.transform.rotation.w = quaternion.w


            # Create the odometry transform frame broadcaster.
        if (self.useImu == False) :
            self.odomBroadcaster.sendTransform(transform)

        odom = Odometry()
        odom.header.frame_id = "odom"
        odom.child_frame_id = self.base_frame
        odom.header.stamp = now.to_msg()
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = float(0)
        odom.pose.pose.orientation = quaternion
        odom.twist.twist.linear.x = vxy
        odom.twist.twist.linear.y = float(0)
        odom.twist.twist.angular.z = vth

        odom.pose.covariance = ODOM_POSE_COVARIANCE
        odom.twist.covariance = ODOM_TWIST_COVARIANCE

        self.odomPub.publish(odom)

        if now > (self.last_cmd_vel + rclpy.duration.Duration(seconds=self.timeout)):
            self.v_des_left = 0
            self.v_des_right = 0

        if self.v_left < self.v_des_left:
            self.v_left += self.max_accel
            if self.v_left > self.v_des_left:
                self.v_left = self.v_des_left
        else:
            self.v_left -= self.max_accel
            if self.v_left < self.v_des_left:
                self.v_left = self.v_des_left

        if self.v_right < self.v_des_right:
            self.v_right += self.max_accel
            if self.v_right > self.v_des_right:
                self.v_right = self.v_des_right
        else:
            self.v_right -= self.max_accel
            if self.v_right < self.v_des_right:
                self.v_right = self.v_des_right
        
        self.v_left = int(self.v_left)
        self.v_right = int(self.v_right)
        self.lVelPub.publish(Int16(data=self.v_left))
        self.rVelPub.publish(Int16(data=self.v_right))

        # Set motor speeds in encoder ticks per PID loop
        if not self.stopped:
            self.get_logger().info(f"{self.v_left}, {self.v_right}")
            self.arduino.drive(self.v_left, self.v_right)

        self.t_next = now + self.t_delta


    def stop(self):
        self.stopped = True
        self.arduino.drive(0, 0)

    def cmdVelCallback(self, req):
        # Handle velocity-based movement requests
        
        self.last_cmd_vel = self.get_clock().now()

        x = req.linear.x         # m/s
        th = req.angular.z       # rad/s

        self.get_logger().info(f"linear x: {x}, angular z: {th}")

        if (self.useSonar == True) :
            if((self.sonar_r0<=self.safe_range_0 and self.sonar_r0>=2) and (x>0)):
                x= 0.0
                th=-0.2
            if((self.sonar_r2<=self.safe_range_0 and self.sonar_r2>=2) and (x>0)):
                x=0.0
                th=0.2
            if((self.sonar_r1<=self.safe_range_1 and self.sonar_r1>=2) and (x>0)):
                x=0.0
                th= 0.0
            if((self.sonar_r3<=self.safe_range_1 and self.sonar_r3>=2) and (x<0)):
                x=0.0
                th= 0.0

        if not self.isPassed and x>0 :
            x = 0

        if x == 0:
            # Turn in place
            if th>0.0 and th<0.2:
                th=0.2
            elif th>-0.2 and th<0.0:
                th=-0.2
            right = th * self.wheel_track  * self.gear_reduction / 2.0
            left = -right
        elif th == 0:
            # Pure forward/backward motion
            left = right = x
        else:
            # Rotation about a point in space
            if (th>0.0 and th<0.2) and (x>-0.05 and x<0.05):
                th=0.2
            elif (th>-0.2 and th<0.0) and (x>-0.05 and x<0.05):
                th=-0.2
            left = x - th * self.wheel_track  * self.gear_reduction / 2.0
            right = x + th * self.wheel_track  * self.gear_reduction / 2.0

        self.v_des_left = int(left * self.ticks_per_meter / self.arduino.PID_RATE)
        self.v_des_right = int(right * self.ticks_per_meter / self.arduino.PID_RATE)


    def isPassedCallback(self, msg):
        if(msg.data>2):
            self.isPassed = False
        else:
            self.isPassed = True

class ArduinoROS(Node):
    def __init__(self):
        super().__init__('Arduino')
        self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)

        self.port = self.declare_parameter("port", "/dev/ttyUSB0").value
        self.baud = self.declare_parameter("baud", 115200).value
        self.timeout = self.declare_parameter("timeout", 0.5).value
        self.base_frame = self.declare_parameter("base_frame", 'base_link').value

        # Overall loop rate: should be faster than fastest sensor rate
        self.rate = self.declare_parameter("rate", 50).value

        # Rate at which summary SensorState message is published. Individual sensors publish
        # at their own rates.
        self.sensorstate_rate = self.declare_parameter("sensorstate_rate", 10).value

        self.use_base_controller = self.declare_parameter("use_base_controller", True).value

        # Set up the time for publishing the next SensorState message
        now = self.get_clock().now()
        self.t_delta_sensors = rclpy.duration.Duration(seconds=(1.0 / self.sensorstate_rate))
        self.t_next_sensors = now + self.t_delta_sensors

        # Initialize a Twist message
        self.cmd_vel = Twist()

        # A cmd_vel publisher so we can stop the robot when shutting down
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        # Initialize the controlller
        self.controller = Arduino(self.port, self.baud, self.timeout)

        # Make the connection
        self.controller.connect()

        self.get_logger().info(f"Connected to Arduino on port {self.port} at {self.baud} baud")

        # Reserve a thread lock
        self.mutex = threading.Lock()

        # get arduino firmware version
        self.year, self.date = self.controller.get_arduino_version()
        self.get_logger().info(f"Arduino firmware version is: {self.year} {self.date}")

        # Initialize the base controller if used
        if self.use_base_controller:
            self.myBaseController = BaseController(self.controller, self.base_frame)

        self.get_logger().info(f"Pass BaseController")

        self.timer = self.create_timer(1.0 / self.rate, self.loop)

        # Start polling the sensors and base controller
    def loop(self):
        # Main loop method for handling data and logic
        # If base controller is used, poll the base controller
        if self.use_base_controller:
            with self.mutex:
                self.myBaseController.poll()
                rclpy.spin_once(self.myBaseController)

    def shutdown(self):
        # Stop the robot
        try:
            self.get_logger().info("Stopping the robot...")
            self.cmd_vel_pub.publish(Twist())
            sleep(2)
        except:
            pass
        self.get_logger().info("Shutting down Arduino Node...")

def main(args=None):
    rclpy.init(args=args)  # Khởi tạo hệ thống ROS 2
    my_arduino = ArduinoROS()  # Khởi tạo node

    try:
        rclpy.spin(my_arduino)  # Giữ cho node chạy
    except KeyboardInterrupt:
        pass
    finally:
        my_arduino.shutdown()  # Thực hiện các tác vụ shutdown
        my_arduino.destroy_node()  # Hủy node
        rclpy.shutdown()  # Tắt hệ thống ROS 2

if __name__ == '__main__':
    main()