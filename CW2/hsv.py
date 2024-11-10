""" ARAP Webots Standard Controller """
from controller import Robot, Motor, LED, DistanceSensor, Camera
import sys
import time
import numpy as np
import cv2 as cv

class ARAP:
    # Robot constants
    MOTORS_NUMBER = 2
    DISTANCE_SENSORS_NUMBER = 8
    #GROUND_SENSORS_NUMBER = 3
    LEDS_NUMBER = 10
    LEFT = 0
    RIGHT = 1
    LED_ON = 255
    LED_OFF = 0
    MAX_SPEED = 6.28
    DELAY = 0.5
    MULTIPLIER = 0.5
    OBSTACLE_DISTANCE = 0.02
    
    motor_names = ("left wheel motor", "right wheel motor")
    distance_sensors_names = ("ps0", "ps1", "ps2", "ps3", "ps4", "ps5", "ps6", "ps7")
    #ground_sensors_names = ("gs0", "gs1", "gs2")
    leds_names = ("led0", "led1", "led2", "led3", "led4", "led5", "led6", "led7", "led8", "led9")
    camera_names = "camera"
    
    # Braitenberg constants
    weights = [ [-1.3, -1.0], 
                [-1.3, -1.0], 
                [-0.5, 0.5], 
                [0.0, 0.0], 
                [0.0, 0.0], 
                [0.05, -0.5], 
                [-0.75, 0.0], 
                [-0.75, 0.0] ]
    
    lookup_table = [ [0.0, 4095.0, 0.002], 
                 [0.005, 2133.33, 0.003], 
                 [0.01, 1465.73, 0.007], 
                 [0.015, 601.46, 0.0406], 
                 [0.02, 383.84, 0.01472], 
                 [0.03, 234.93, 0.0241], 
                 [0.04, 158.03, 0.0287], 
                 [0.05, 120.0, 0.04225], 
                 [0.06, 104.09, 0.03065], 
                 [0.07, 67.19, 0.04897] ]
    
    offsets = [MULTIPLIER * MAX_SPEED, MULTIPLIER * MAX_SPEED]
    colors_rgb = {
      "red": [[175, 187, 71], [176, 188, 72], [179, 219, 212], [179, 202, 208]],
      "blue": [[119, 211, 99], [119, 211, 99], [120, 223, 217], [120, 206, 213]],
      "green": [[65, 199, 77], [61, 223, 213], [61, 205, 209], [65, 199, 77]]
    }
  
    def __init__(self):
        # Robot instance
        self.robot = Robot()
        self.time_step = int(self.robot.getBasicTimeStep())
        
        # Robot instance attributes
        self.distance_sensors = []
        self.distance_sensors_values = []
        self.distance_range = 0.0
        self.leds = []
        self.leds_values = []  # values: 0 - 255
        self.speeds = [0.0, 0.0]
        self.camera = None
        self.left_motor = None
        self.right_motor = None
        
        # Support attributes
        self.counter = 0
        self.camera_interval = 0
        self.red = 0
        self.green = 0
        self.blue = 0
        self.seen_red = False
        self.seen_blue = False
        self.seen_green = False
        self.red_tally = 0
        self.blue_tally = 0
        self.green_tally = 0
        self.red_in_sight = False
        self.blue_in_sight = False
        self.green_in_sight = False
        self.default_red = 100  # Example default value for red
        self.default_green = 150  # Example default value for green
        self.default_blue = 50   # Example default value for blue
        
        # Initialize last_minute_time
        self.last_minute_time = time.time()  # Initialize here
        self.minutes_passed = 0  # Track elapsed minutes

        # Run init methods
        self.init_devices()


    def init_devices(self):
        # Distance sensors initialisation
        for i in range(self.DISTANCE_SENSORS_NUMBER):
            self.distance_sensors.append(self.robot.getDevice(self.distance_sensors_names[i]))
            self.distance_sensors_values.append(0.0)
            self.distance_sensors[i].enable(self.time_step)
            
        # Ground sensors initialisation
        #for i in range(self.GROUND_SENSORS_NUMBER):
        #    self.ground_sensors.append(self.robot.getDevice(self.ground_sensors_names[i]))
        #    self.ground_sensors_values.append(0.0)
        #    self.ground_sensors[i].enable(self.time_step)
            
        # LEDs initialisation
        for i in range(self.LEDS_NUMBER):
            self.leds.append(self.robot.getDevice(self.leds_names[i]))
            self.leds_values.append(self.LED_OFF)
            if self.leds[i].get() > self.LED_OFF:
                self.leds[i].set(self.LED_OFF)
            
        # Camera initialisation
        self.camera = self.robot.getDevice(self.camera_names)
        self.camera.enable(self.time_step)
        
        # Motors initialisation
        self.left_motor = self.robot.getDevice(self.motor_names[self.LEFT])
        self.right_motor = self.robot.getDevice(self.motor_names[self.RIGHT])
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(self.MAX_SPEED * 0.0)
        self.right_motor.setVelocity(self.MAX_SPEED * 0.0)
        
        self.step()
        
    def wait(self, sec):
        start_time = self.robot.getTime()
        elapsed_time = start_time
        while start_time + sec > self.robot.getTime():
            self.step()
        return True
            
    def reset_actuator_values(self):
        for i in range(2):
            self.speeds[i] = 0.0
        
        for i in range(self.LEDS_NUMBER):
            self.leds_values[i] = self.LED_OFF
            
        for i in range(self.DISTANCE_SENSORS_NUMBER):
            self.distance_sensors_values[i] = 0.0

    def set_actuators(self):
        for i in range(self.LEDS_NUMBER):
            self.leds[i].set(self.leds_values[i])
        self.left_motor.setVelocity(self.speeds[self.LEFT])
        self.right_motor.setVelocity(self.speeds[self.RIGHT])

    def blink_leds(self):
        brightness = int(((self.counter / 10) % self.LEDS_NUMBER) * 255)
        if brightness > self.LED_ON:
            self.counter = 0
        
        for i in range(self.LEDS_NUMBER):
            self.leds_values[i] = brightness
            #print("LED[", i, "] =", self.leds_values[i])  # DEBUG
        self.counter += 1

    def get_sensor_input(self):
        for i in range(self.DISTANCE_SENSORS_NUMBER):
            self.distance_sensors_values[i] = self.distance_sensors[i].getValue()
            sensor_total = self.distance_sensors_values[0] + self.distance_sensors_values[7]  # get front distance range from ps0 and ps7
            
            # Translate sensor_total value to lookup_table values
            for j in range(len(self.lookup_table)):
                if sensor_total >= self.lookup_table[j][1]:
                    self.distance_range = self.lookup_table[j][0]
                    break
            #print(self.distance_range)  # DEBUG
            
            # Normalise distance sensor values between 0.0 and 1.0
            self.distance_sensors_values[i] /= 4096;  # 1.0 = avoid, 0.0 = no avoid
            if self.distance_sensors_values[i] > 1.0:
                self.distance_sensors_values[i] = 1.0  # truncate max value to 1.0
            #print("Sensor[", i, "] =", self.distance_sensors_values[i])  # DEBUG
        return self.distance_range

    def get_time_step(self):
        self.time_step = -1
        if self.time_step == -1:
            self.time_step = int(self.robot.getBasicTimeStep())
        return self.time_step
    
    def step(self):
        if self.robot.step(self.get_time_step()) == -1:
            sys.exit(0)

  def get_camera_image(self, interval):
    # Convert RGB shades to HSV bounds
    def get_hsv_bounds(rgb_value, buffer=10):
        hsv_value = cv.cvtColor(np.uint8([[rgb_value]]), cv.COLOR_RGB2HSV)[0][0]
        lower_bound = np.clip(hsv_value - buffer, 0, 255)
        upper_bound = np.clip(hsv_value + buffer, 0, 255)
        return lower_bound, upper_bound

    color_bounds = {color: [get_hsv_bounds(shade) for shade in shades] for color, shades in self.colors_rgb.items()}

    width = self.camera.getWidth()
    height = self.camera.getHeight()
    image_data = self.camera.getImage()

    # Initialize RGB values
    red_avg = 0
    green_avg = 0
    blue_avg = 0

    # Capture a new image every specified interval
    if self.camera_interval >= interval:
        # Reset color flags
        red_detected = False
        green_detected = False
        blue_detected = False

        # Convert Webots image data to OpenCV format
        img = np.frombuffer(image_data, np.uint8).reshape((height, width, 4))[:, :, :3]
        img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

        # Process each color and its shades
        for color, bounds in color_bounds.items():
            color_mask = np.zeros((height, width), dtype=np.uint8)
            for lower_bound, upper_bound in bounds:
                # Apply mask for each shade
                shade_mask = cv.inRange(img_hsv, lower_bound, upper_bound)
                color_mask = cv.bitwise_or(color_mask, shade_mask)

            # Check if the color is detected in the current image and calculate mean RGB values
            if cv.countNonZero(color_mask) > 0:
                filtered_img = cv.bitwise_and(img, img, mask=color_mask)
                mean_red = np.mean(filtered_img[:, :, 2])
                mean_green = np.mean(filtered_img[:, :, 1])
                mean_blue = np.mean(filtered_img[:, :, 0])

                if color == "red" and not self.red_in_sight:
                    print("Detected Red!")
                    self.red_tally += 1
                    self.red_in_sight = True
                    red_avg = mean_red
                elif color == "blue" and not self.blue_in_sight:
                    print("Detected Blue!")
                    self.blue_tally += 1
                    self.blue_in_sight = True
                    blue_avg = mean_blue
                elif color == "green" and not self.green_in_sight:
                    print("Detected Green!")
                    self.green_tally += 1
                    self.green_in_sight = True
                    green_avg = mean_green

        # Track time for one-minute interval and log tallies
        current_time = time.time()
        if current_time - self.last_minute_time >= 60:
            self.minutes_passed += 1
            print(f"Encounters in the last minute (red: {self.red_tally}, green: {self.green_tally}, blue: {self.blue_tally}) during {self.minutes_passed} minutes.")
            self.last_minute_time = current_time
            # Reset tallies for next minute
            self.red_tally = 0
            self.green_tally = 0
            self.blue_tally = 0

        # Reset interval counter
        self.camera_interval = 0
    else:
        # Increment the camera interval to wait until the next capture
        self.camera_interval += 1

    return red_avg, green_avg, blue_avg  # Return average RGB values


    def step(self):
        if self.robot.step(self.time_step) == -1:
            sys.exit(0)

    #def ground_obstacles_detected(self):
    #    for i in range(self.GROUND_SENSORS_NUMBER):
    #        if not self.ground_sensors[i]:
    #            return False
    #        if self.ground_sensors_values[i] < 500.0:
    #            return True
    #    else:
    #        return False
    
    def front_obstacles_detected(self):
        average = ( self.distance_sensors_values[0] + self.distance_sensors_values[7] ) / 2.0
        #print("Front sensor:", average)  # DEBUG
        if average > self.OBSTACLE_DISTANCE:
            return True
        else:
            return False
        
    def back_obstacles_detected(self):
        average = ( self.distance_sensors_values[3] + self.distance_sensors_values[4] ) / 2.0
        #print("Back sensor:", average)  # DEBUG
        if average > self.OBSTACLE_DISTANCE:
            return True
        else:
            return False
        
    def left_obstacles_detected(self):
        average = ( self.distance_sensors_values[5] + self.distance_sensors_values[6] ) / 2.0
        #print("Left sensor:", average)  # DEBUG
        if average > self.OBSTACLE_DISTANCE:
            return True
        else:
            return False
        
    def right_obstacles_detected(self):
        average = ( self.distance_sensors_values[1] + self.distance_sensors_values[2] ) / 2.0
        #print("Right sensor:", average)  # DEBUG
        if average > self.OBSTACLE_DISTANCE:
            return True
        else:
            return False
    
    def run_braitenberg(self):
        for i in range(2):
            self.speeds[i] = 0.0
            
            for j in range(self.DISTANCE_SENSORS_NUMBER):
                self.speeds[i] += self.distance_sensors_values[j] * self.weights[j][i]
            
            self.speeds[i] = self.offsets[i] + self.speeds[i] * self.MAX_SPEED
            if self.speeds[i] > self.MAX_SPEED:
                self.speeds[i] = self.MAX_SPEED
            elif self.speeds[i] < -self.MAX_SPEED:
                self.speeds[i] = -self.MAX_SPEED
        #print("Speeds: left =", self.speeds[self.LEFT], ", right =", self.speeds[self.RIGHT])  # DEBUG

    def move(self, left_multiplier, right_multiplier):
        # Left and Right multiplier values must be between 0.0 to -/+1.0
        self.left_motor.setVelocity(left_multiplier * self.MAX_SPEED)
        self.right_motor.setVelocity(right_multiplier * self.MAX_SPEED)
        self.wait(self.DELAY)

    def move_forward(self):
        self.left_motor.setVelocity(self.MAX_SPEED * self.MULTIPLIER)
        self.right_motor.setVelocity(self.MAX_SPEED * self.MULTIPLIER)
        self.wait(self.DELAY)
        
    def move_backward(self):
        self.left_motor.setVelocity(-self.MAX_SPEED * self.MULTIPLIER)
        self.right_motor.setVelocity(-self.MAX_SPEED * self.MULTIPLIER)
        self.wait(self.DELAY)

    def turn_left(self):
        self.left_motor.setVelocity(-self.MAX_SPEED * self.MULTIPLIER)
        self.right_motor.setVelocity(self.MAX_SPEED * self.MULTIPLIER)
        self.wait(self.DELAY)
    
    def turn_right(self):
        self.left_motor.setVelocity(self.MAX_SPEED * self.MULTIPLIER)
        self.right_motor.setVelocity(-self.MAX_SPEED * self.MULTIPLIER)
        self.wait(self.DELAY)
