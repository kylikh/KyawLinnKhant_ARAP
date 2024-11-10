""" ARAP Webots Standard Controller """
from controller import Robot, Motor, LED, DistanceSensor, Camera
import sys
import time
import cv2 as cv
import numpy as np

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
        self.camera_interval = 0
        self.red_tally, self.green_tally, self.blue_tally = 0, 0, 0
        self.red_in_sight, self.green_in_sight, self.blue_in_sight = False, False, False
        self.camera = None  # Assume this is your Webots camera instance
        self.last_minute_time = time.time()
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
        width = self.camera.getWidth()
        height = self.camera.getHeight()
        image = self.camera.getImage()
    
        # Initialize average RGB values
        avg_red, avg_green, avg_blue = 0, 0, 0
    
        if self.camera_interval >= interval:
            # Convert the Webots image to OpenCV format and make it writable
            img = np.frombuffer(image, np.uint8).reshape((height, width, 4))[:, :, :3].copy()
    
            # Define center region of interest (ROI) to avoid edge detections
            roi_y_start, roi_y_end = int(0.1 * height), int(0.9 * height)
            roi_x_start, roi_x_end = int(0.1 * width), int(0.9 * width)
            img_hsv = cv.cvtColor(img[roi_y_start:roi_y_end, roi_x_start:roi_x_end], cv.COLOR_BGR2HSV)
    
            # Define refined HSV ranges for each color
            lower_dark_red = np.array([0, 80, 20])
            upper_dark_red = np.array([10, 255, 150])
            lower_light_red = np.array([170, 80, 20])
            upper_light_red = np.array([180, 255, 255])
            lower_green = np.array([35, 50, 50])
            upper_green = np.array([85, 255, 255])
            lower_dark_blue = np.array([100, 70, 50])  # Adjusted blue range
            upper_dark_blue = np.array([130, 255, 200])
            lower_light_blue = np.array([105, 120, 90])
            upper_light_blue = np.array([125, 255, 220])
            
            # Create masks for each color within the ROI
            red_mask = cv.inRange(img_hsv, lower_dark_red, upper_dark_red) | cv.inRange(img_hsv, lower_light_red, upper_light_red)
            green_mask = cv.inRange(img_hsv, lower_green, upper_green)
            blue_mask = cv.inRange(img_hsv, lower_dark_blue, upper_dark_blue)| cv.inRange(img_hsv, lower_light_blue, upper_light_blue)

    
            # Apply morphological operations to refine masks
            kernel = np.ones((5, 5), np.uint8)
            red_mask = cv.morphologyEx(red_mask, cv.MORPH_CLOSE, kernel)
            green_mask = cv.morphologyEx(green_mask, cv.MORPH_CLOSE, kernel)
            blue_mask = cv.morphologyEx(blue_mask, cv.MORPH_CLOSE, kernel)
    
            # Display the grid with detection overlay and apply masking
            self.display_grid_with_detection(img, red_mask, green_mask, blue_mask)

            # Consistency check with a minimum pixel threshold to ensure a significant detection
            red_detected = cv.countNonZero(red_mask) > 550  # Adjust threshold for significance
            green_detected = cv.countNonZero(green_mask) > 550
            blue_detected = cv.countNonZero(blue_mask) > 550
    
            # Tally and first-time detection messages
            if red_detected and not self.red_in_sight:
                if not self.seen_red:
                    print("First time seeing the Red box!")
                    self.seen_red = True
                self.red_in_sight = True
                self.red_tally += 1  # Increment red tally when detected
            elif not red_detected and self.red_in_sight:
                self.red_in_sight = False
    
            if green_detected and not self.green_in_sight:
                if not self.seen_green:
                    print("First time seeing the Green box!")
                    self.seen_green = True
                self.green_in_sight = True
                self.green_tally += 1  # Increment green tally when detected
            elif not green_detected and self.green_in_sight:
                self.green_in_sight = False
    
            if blue_detected and not self.blue_in_sight:
                if not self.seen_blue:
                    print("First time seeing the Blue box!")
                    self.seen_blue = True
                self.blue_in_sight = True
                self.blue_tally += 1  # Increment blue tally when detected
            elif not blue_detected and self.blue_in_sight:
                self.blue_in_sight = False
    
            # Minute-based tally reporting
            current_time = time.time()
            if current_time - self.last_minute_time >= 60:
                self.minutes_passed += 1
                print(f"We've encountered (red: {self.red_tally}, green: {self.green_tally}, blue: {self.blue_tally}) times during the last {self.minutes_passed} minutes.")
                self.last_minute_time = current_time
    
                # Reset `in_sight` flags for the next minute
                self.red_in_sight = False
                self.green_in_sight = False
                self.blue_in_sight = False
    
            # Reset interval counter
            self.camera_interval = 0
        else:
            # Increment the camera interval to wait until the next capture
            self.camera_interval += 1
    
        # Return average RGB values if needed
        return avg_red, avg_green, avg_blue
    
    def display_grid_with_detection(self, img, red_mask, green_mask, blue_mask):
        # Define a region of interest (ROI) covering more of the image vertically and horizontally
        height, width = img.shape[:2]
        roi_y_start = int(0.1 * height)  # Start 10% from the top
        roi_y_end = int(0.9 * height)    # End 90% from the top
        roi_x_start = 0  # Start at the left edge of the image
        roi_x_end = width  # End at the right edge of the image
    
        # Define grid dimensions for full coverage
        grid_rows, grid_cols = 10, 13 # 15 rows for more vertical coverage, 5 columns for horizontal coverage
        segment_height = (roi_y_end - roi_y_start) // grid_rows
        segment_width = (roi_x_end - roi_x_start) // grid_cols
        min_pixels = 0.05 * (segment_width * segment_height)  # Minimum pixels for a detection
    
        # Copy image to draw on
        img_with_grid = img.copy()
    
        # Loop through each grid segment within the ROI
        for i in range(grid_rows):
            for j in range(grid_cols):
                y_start = roi_y_start + i * segment_height
                y_end = roi_y_start + (i + 1) * segment_height
                x_start = roi_x_start + j * segment_width
                x_end = roi_x_start + (j + 1) * segment_width
    
                # Check each segment for color detection
                red_segment = red_mask[y_start:y_end, x_start:x_end]
                green_segment = green_mask[y_start:y_end, x_start:x_end]
                blue_segment = blue_mask[y_start:y_end, x_start:x_end]
    
                if cv.countNonZero(red_segment) > min_pixels:
                    # Fill detected red segments with semi-transparent red
                    cv.rectangle(img_with_grid, (x_start, y_start), (x_end, y_end), (0, 0, 255), -1)
                    cv.addWeighted(img_with_grid[y_start:y_end, x_start:x_end], 0.3, img[y_start:y_end, x_start:x_end], 0.7, 0, img_with_grid[y_start:y_end, x_start:x_end])
    
                if cv.countNonZero(green_segment) > min_pixels:
                    # Fill detected green segments with semi-transparent green
                    cv.rectangle(img_with_grid, (x_start, y_start), (x_end, y_end), (0, 255, 0), -1)
                    cv.addWeighted(img_with_grid[y_start:y_end, x_start:x_end], 0.3, img[y_start:y_end, x_start:x_end], 0.7, 0, img_with_grid[y_start:y_end, x_start:x_end])
    
                if cv.countNonZero(blue_segment) > min_pixels:
                    # Fill detected blue segments with semi-transparent blue
                    cv.rectangle(img_with_grid, (x_start, y_start), (x_end, y_end), (255, 0, 0), -1)
                    cv.addWeighted(img_with_grid[y_start:y_end, x_start:x_end], 0.3, img[y_start:y_end, x_start:x_end], 0.7, 0, img_with_grid[y_start:y_end, x_start:x_end])
    
                # Draw subtle grid lines
                cv.rectangle(img_with_grid, (x_start, y_start), (x_end, y_end), (200, 200, 200), 1)
    
        # Display the image with the grid overlay
        cv.imshow("Grid with Detection (Expanded Vertically and Horizontally)", img_with_grid)
        cv.waitKey(1)
    
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
