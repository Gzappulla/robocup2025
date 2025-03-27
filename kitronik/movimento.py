import PicoRobotics
from pimoroni import PID
import time
import machine
from bno055 import BNO055


class RobotMotors:
    def __init__(self, kp, ki, kd, target_speed, max_steering):
        """
        Initializes the RobotMotors class with PID controllers and a target speed.

        :param kp: Proportional gain for PID.
        :param ki: Integral gain for PID.
        :param kd: Derivative gain for PID.
        :param target_speed: Maximum target speed for the motors (in RPM).
        :param max_steering: Maximum steering value (positive and negative).
        """
        self.target_speed = target_speed  # Maximum speed for motors
        self.max_steering = max_steering  # Maximum steering value

        # Initialize the board
        self.board = PicoRobotics.KitronikPicoRobotics()
        # Initialize the gyroscope
        self.i2c = machine.I2C(1, sda=machine.Pin(9), scl=machine.Pin(10))
        self.imu = BNO055(self.i2c)
        print("BNO055 connected")
        self.calibrated = False
        
        # Configure motors
        self.motor_left = self.board.motors[MOTOR_B]
        self.motor_right = self.board.motors[MOTOR_A]
        self.motor_left.speed_scale(100)
        self.motor_right.speed_scale(100)

        # Configure encoders
        self.encoder_left = Pin(0, Pin.IN, Pin.PULL_UP)#self.board.encoders[MOTOR_B]
        self.encoder_right = Pin(2, Pin.IN, Pin.PULL_UP)#self.board.encoders[MOTOR_A]

        # Motor and encoder directions (adjust as needed) normal = 1, reversed = 0
        self.motor_left.direction(0)
        self.encoder_left.direction(0)
        self.motor_right.direction(1)
        self.encoder_right.direction(1)

        # Initialize PID controllers
        self.pid_left = PID(kp, ki, kd, 1 / 100)  # Assuming 100 updates per second
        self.pid_right = PID(kp, ki, kd, 1 / 100)

        # Enable motors
        self.motor_left.enable()
        self.motor_right.enable()

    def normalize_speed(self, speed):
        """
        Normalize the input speed (0-100) to the target range (0 to target_speed),
        setting speeds below 10% of target_speed to 0.

        :param speed: Input speed (0-100).
        :return: Normalized speed (0 to target_speed).
        """
        normalized = (speed / 100) * self.target_speed
        return max(normalized, 0) if normalized >= 0.10 * self.target_speed else 0

    def move(self, steering, speed):
        """
        Control the motors using steering and speed inputs.

        :param steering: Steering value (-max_steering to max_steering).
        :param speed: Target speed (0-100).
        """
        # Calculate speed adjustments based on steering
        left_speed_adjustment = 1
        right_speed_adjustment = 1
        if steering < 0:
            left_speed_adjustment = 1 - (abs(steering) / abs(self.max_steering - self.max_steering*0.3)) * 2.2
        elif steering > 0:
            right_speed_adjustment = 1 - (abs(steering) / abs(self.max_steering - self.max_steering*0.3)) * 2.2
    
    
        # Normalize taSERVO_1rget speed
        base_speed = self.normalize_speed(speed)

        # Calculate final motor speeds
        target_speed_left = base_speed * left_speed_adjustment
        target_speed_right = base_speed * right_speed_adjustment

        # Set motor speeds
        self.motor_left.speed(target_speed_left)
        self.motor_right.speed(target_speed_right)

        print(f"Steering: {steering}, Speed: {speed}")
        print(f"Motor Speeds: Left={target_speed_left}, Right={target_speed_right}")

    def stop(self):
        """Stops both motors."""
        self.motor_left.stop()
        self.motor_right.stop()
        
    def reset_gyro(self):
        """Resets the gyroscope value to 0."""
        self.imu.reset()

    def get_gyro_angle(self):
        """Returns the current gyroscope angle."""
        return self.imu.euler()[0]  # Assuming the first value is the yaw angle

    def map_angle_value(self, value):
        if 0 <= value <= 180:
            return value
        elif 181 <= value <= 360:
            return value - 360
        else:
            return value

    def turn_to_angle(self, target_angle):
        """Turns the robot to the specified angle using the gyroscope."""
        time.sleep(1)
        self.stop()
        self.reset_gyro()
        while True:
            current_angle = self.get_gyro_angle()
            current_mapped_angle = self.map_angle_value(current_angle)
            if abs(current_mapped_angle - target_angle) < 3:  # Allow a small margin of error
                print("proceding forward a bit")
                self.motor_left.speed(100)
                self.motor_right.speed(100)
                time.sleep(0.3)
                return
            elif current_mapped_angle < target_angle:
                self.motor_left.speed(100)
                self.motor_right.speed(-100)
            else:
                self.motor_left.speed(-100)
                self.motor_right.speed(100)

    def op_code_handler(self, op_code):
        if op_code == 1:
            print("turn right")
            self.turn_to_angle(90)
        elif op_code == 2:
            print("turn left")
            self.turn_to_angle(-90)
        elif op_code == 3:
            print("ignore")
        elif op_code == 4:
            print("go back")
            self.turn_to_angle(180)
        elif op_code == 69:
            self.stop()
        else:
            print(f"Invalid OP_CODE received: {op_code}")


    def avoid_obstacle(self):
        self.turn_to_angle(-90)
        self.stop()
        while True:
            target_angle = 80
            current_angle = self.get_gyro_angle()
            current_mapped_angle = self.map_angle_value(current_angle)
            if abs(current_mapped_angle - target_angle) < 3:
                print("getting back to line")
                self.turn_to_angle(-100)
                break
            print(abs(current_mapped_angle - target_angle))
            if 89 < abs(current_mapped_angle - target_angle) < 91:  # Allow a small margin of error
                print("proceding forward a bit")
                self.motor_left.speed(100)
                self.motor_right.speed(100)
                time.sleep(0.65)
            elif current_mapped_angle < target_angle:
                self.motor_left.speed(100)
                self.motor_right.speed(35)
            else:
                self.motor_left.speed(35)
                self.motor_right.speed(100)
            
        