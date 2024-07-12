import cv2
import robomaster
from robomaster import robot
from robomaster import vision
import time
import numpy as np
import math
from math import radians, degrees, atan2

# PID Controller Class
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0
        self.integral = 0

    def compute(self, error):
        self.integral += error
        derivative = error - self.previous_error

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error

        return output

# Initializing variables
markers = []
sensor1_distance = 0
sensor2_distance = 0
linear_velocity = 0.15
angular_velocity = 20


class MarkerInfo:
    def __init__(self, x, y, w, h, info):
        self._x = x
        self._y = y
        self._w = w
        self._h = h
        self._info = info

    @property
    def pt1(self):
        return int((self._x - self._w / 2) * 1280), int((self._y - self._h / 2) * 720)

    @property
    def pt2(self):
        return int((self._x + self._w / 2) * 1280), int((self._y + self._h / 2) * 720)

    @property
    def center(self):
        return int(self._x * 1280), int(self._y * 720)
    
    @property
    def x_center(self):
        return int(self._x * 1280)

    @property
    def text(self):
        return self._info

def on_detect_marker(marker_info):
    number = len(marker_info)
    markers.clear()
    for i in range(0, number):
        x, y, w, h, info = marker_info[i]
        markers.append(MarkerInfo(x, y, w, h, info))
        time.sleep(1)

def marker_detection1():
    pid_controller = PIDController(kp=1/900, ki=0, kd=0)
    result = ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)
    setpoint = 640  # Center of the image
    error = 1000

    while abs(error) > 30:
        result = ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)

        measured_value = markers[0].x_center
        print(f"Center = {measured_value}")
        error = setpoint - measured_value
        print(f"Error = {error}")
        correction = pid_controller.compute(error)
        print(f"Correction = {correction}")

        if error > 0:
            ep_chassis.drive_speed(x=0, y=correction, z=0, timeout=0.1)
        else:
            ep_chassis.drive_speed(x=0, y=-correction, z=0, timeout=0.1)

    ep_chassis.drive_speed(x=0, y=0, z=0)


def sub_data_handler(sub_info):
    global sensor1_distance
    sensor1_distance = sub_info[1]
    global sensor2_distance
    sensor2_distance = sub_info[2]

def move_to_marker():
    ep_sensor.sub_distance(freq=5, callback=sub_data_handler)
    time.sleep(1)
    distance_average = (sensor1_distance + sensor2_distance) / 2

    while distance_average > 800:
        ep_chassis.drive_speed(x=linear_velocity, y=0, z=0)
        time.sleep(0.5)
        distance_average = (sensor1_distance + sensor2_distance) / 2

def marker_detection2():
    pid_controller = PIDController(kp=1/950, ki=0, kd=0)
    result = ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)
    setpoint = 640  # Center of the image
    error = 1000

    while abs(error) > 10:
        result = ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)

        measured_value = markers[0].x_center
        print(f"Center = {measured_value}")
        error = setpoint - measured_value
        print(f"Error = {error}")
        correction = pid_controller.compute(error)
        print(f"Correction = {correction}")

        if error > 0:
            ep_chassis.drive_speed(x=0, y=correction, z=0, timeout=0.1)
        else:
            ep_chassis.drive_speed(x=0, y=-correction, z=0, timeout=0.1)

    ep_chassis.drive_speed(x=0, y=0, z=0)

def grab_object():
    ep_chassis.drive_speed(x=0.2, y=0, z=0, timeout=1)
    time.sleep(1)

    ep_gripper.open()
    time.sleep(2)

    ep_arm.move(x=70, y=100).wait_for_completed()
    time.sleep(1)

    ep_chassis.move(x=0.1, y=0, z=0, xy_speed=0.1).wait_for_completed()
    time.sleep(1)

    ep_gripper.close(power=60)
    time.sleep(2)

    ep_chassis.move(x=-0.3, y=0, z=0, xy_speed=0.5).wait_for_completed()
    ep_arm.move(x=-70, y=-100).wait_for_completed()
    time.sleep(1)

def marker2_detection_heading():
    result = ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)
    
    # Rotate until marker 2 comes into the field of view
    while all(marker.text != "2" for marker in markers):
        ep_chassis.drive_speed(x=0, y=0, z=angular_velocity, timeout=0.1)
    
    error = 1

    while abs(error) > 0.08:
        ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)
        time.sleep(1)

        error = (markers[0].x_center / 1280) - 0.5

        if error < 0:
            ep_chassis.move(x=0, y=0, z=10, z_speed=20).wait_for_completed()
        elif error > 0:
            ep_chassis.move(x=0, y=0, z=-10, z_speed=20).wait_for_completed()
        else:
            ep_chassis.move(x=0, y=0, z=0).wait_for_completed()
            time.sleep(1)

def align_marker():
    ep_sensor.sub_distance(freq=5, callback=sub_data_handler)
    time.sleep(1)
    error = sensor1_distance - sensor2_distance

    while abs(error) > 20:
        if error < 0:
            ep_chassis.move(x=0, y=0, z=5, z_speed=5).wait_for_completed()
        elif error > 0:
            ep_chassis.move(x=0, y=0, z=-5, z_speed=5).wait_for_completed()
        else:
            ep_chassis.move(x=0, y=0, z=0).wait_for_completed()
        
        error = sensor1_distance - sensor2_distance
        print(error)

def drop_object():
    ep_chassis.move(x=0.3, y=0, z=0, xy_speed=0.2).wait_for_completed()
    time.sleep(1)

    ep_arm.move(x=40, y=120).wait_for_completed()
    time.sleep(1)

    ep_chassis.move(x=0.15, y=0, z=0, xy_speed=0.1).wait_for_completed()
    time.sleep(1)

    ep_gripper.open()
    time.sleep(2)

    ep_chassis.move(x=-0.2, y=0, z=0, xy_speed=0.5).wait_for_completed()
    time.sleep(1)

    ep_arm.move(x=-40, y=-120).wait_for_completed()
    time.sleep(1)

    ep_gripper.close(power=50)
    time.sleep(2)

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="ap")

    ep_vision = ep_robot.vision
    ep_camera = ep_robot.camera
    ep_chassis = ep_robot.chassis
    ep_sensor = ep_robot.sensor
    ep_gripper = ep_robot.gripper
    ep_arm = ep_robot.robotic_arm

    ep_camera.start_video_stream(display=False)
    result = ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)

    img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
    for j in range(0, len(markers)):
        cv2.rectangle(img, markers[j].pt1, markers[j].pt2, (255, 255, 255))
        cv2.putText(img, markers[j].text, markers[j].center, cv2.FONT_HERSHEY_SIMPLEX, 1.5, (255, 255, 255), 3)
    cv2.imshow("Markers", img)
    cv2.waitKey(1)
    time.sleep(1)
    cv2.destroyAllWindows()
    
    ep_sensor.sub_distance(freq=5, callback=sub_data_handler)
    time.sleep(1)
    ep_sensor.unsub_distance()

    time.sleep(5)
    print("Finding Marker 1...")
    
    marker_detection1()
    print("Detected Marker 1")
    time.sleep(1)
    print("Moving towards Marker 1")
    
    move_to_marker()
    print("Reached its center")
    time.sleep(1)

    ep_sensor.unsub_distance()
    align_marker()
    time.sleep(1)
    
    # marker_detection2()
    # time.sleep(1)

    print("Aligning Marker 1")
    ep_sensor.unsub_distance()
    
    align_marker()
    time.sleep(1)
    print("Grabbing object")
    
    grab_object()
    time.sleep(1)
    print("Detecting marker 2")
    
    marker2_detection_heading()
    print("Marker 2 detected successfully")
    time.sleep(1)
    ep_sensor.unsub_distance()
    print("Moving towards Marker 2")
    
    move_to_marker()
    time.sleep(1)
    print("Aligning Marker 2")
    ep_sensor.unsub_distance()
    
    align_marker()
    time.sleep(1)
    print("Dropping object")
    
    drop_object()

    ep_camera.stop_video_stream()
    ep_robot.close()

