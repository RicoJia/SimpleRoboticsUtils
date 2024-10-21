#!/usr/bin/env python3

"""
Visualization: frame should be pointing to the first laser point int the received message
max range = 12m
datasheet: https://www.slamtec.ai/wp-content/uploads/2023/11/LD108_SLAMTEC_rplidar_datasheet_A1M8_v3.0_en.pdf
"""
import os
from math import cos, sin, pi, floor, sqrt
from adafruit_rplidar import RPLidar
from typing import List
import subprocess

# used to scale data to fit on the screen
# in meters
RANGE_MAX = 12
DISPLAY_MAX_PIXEL = 4000
TOTAL_NUM_ANGLES = 360


def find_lidar_usb_device() -> str:
    """Find the USB device path"""
    try:
        DEVICE_PATH = "/dev/serial/by-id"
        result_str = subprocess.check_output(["ls", DEVICE_PATH], text=True)
        results = result_str.split("\n")
        for r in results:
            if "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller" in r:
                return os.path.join(DEVICE_PATH, r)
    except Exception as e:
        pass
    # This is very ugly, and I have not found a robust way to find the device yet.
    return "/dev/ttyUSB0"


def find_laser_scanner_rpi():
    serial_path = "/dev/serial/by-path/"
    device_paths = os.listdir(serial_path)
    cp210x_devices = []
    for device in device_paths:
        full_path = os.path.join(serial_path, device)
        real_device = os.path.realpath(full_path)  # Get the real device file path (resolves symlinks)
        try:
            # Use udevadm to get the device attributes
            result = subprocess.run(
                ["udevadm", "info", "--name=" + real_device, "--attribute-walk"], text=True, capture_output=True
            )
            # Check if the output contains the line indicating it uses the cp210x driver
            if 'DRIVERS=="cp210x"' in result.stdout:
                cp210x_devices.append(real_device)
        except subprocess.CalledProcessError:
            print(f"Failed to get udev info for {real_device}")
    if len(cp210x_devices) != 1:
        raise Exception(f"Found cp210x devices: {cp210x_devices}, expected only 1")
    return cp210x_devices[0]


def draw_arrow(screen, color, start, end, arrow_size=10):
    """
    Draw an arrow between two points.
    Args:
        screen: The Pygame display surface to draw on.
        color: The color of the arrow.
        start: A tuple (x, y) for the start point of the arrow.
        end: A tuple (x, y) for the end point of the arrow.
        arrow_size: The size of the arrow head.
    """
    # Calculate the direction of the arrow
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    # TODO: use sqrt.
    length = sqrt(dx**2 + dy**2)
    if length == 0:
        return  # Cannot draw an arrow if length is 0

    # Normalize the direction
    dx /= length
    dy /= length

    # Draw the main line of the arrow
    pygame.draw.line(screen, color, start, end, 2)

    # Calculate the points of the arrow head
    angle = pi / 6  # 30 degrees for each side of the arrow head
    sin_angle = sin(angle)
    cos_angle = cos(angle)

    right_dx = dx * cos_angle + dy * sin_angle
    right_dy = -dx * sin_angle + dy * cos_angle
    left_dx = dx * cos_angle - dy * sin_angle
    left_dy = dx * sin_angle + dy * cos_angle

    # Points of the arrow head
    right_point = (end[0] - arrow_size * right_dx, end[1] - arrow_size * right_dy)
    left_point = (end[0] - arrow_size * left_dx, end[1] - arrow_size * left_dy)
    # Draw the sides of the arrow head
    pygame.draw.line(screen, color, end, right_point, 2)
    pygame.draw.line(screen, color, end, left_point, 2)


def process_data(data: List[float], lcd: RPLidar):
    """Displays laser range data from 0th angle to the last

    Args:
        data (): _description_
        lcd (_type_): _description_
    """
    global DISPLAY_MAX_PIXEL
    lcd.fill((0, 0, 0))
    width, height = lcd.get_size()  # Get current size of the display

    def _to_pixel(x, y):
        return (
            width // 2 + int(x / DISPLAY_MAX_PIXEL * (width // 2 - 1)),
            height // 2 + int(y / DISPLAY_MAX_PIXEL * (height // 2 - 1)),
        )

    for angle_id in range(TOTAL_NUM_ANGLES):
        #     # distance in data:
        distance = data[angle_id]
        if distance > 0:  # ignore initially ungathered data points
            radians = angle_id / TOTAL_NUM_ANGLES * 2 * pi
            x = distance * cos(radians)
            y = distance * sin(radians)
            # Adjust scale based on the new display size
            point = _to_pixel(x, y)
            lcd.set_at(point, pygame.Color(255, 255, 255))
            if angle_id == 0:
                center_point = (
                    width / 2,
                    height / 2,
                )  # Assuming this is the center of your display
                draw_arrow(lcd, pygame.Color(255, 0, 0), center_point, point)

    pygame.display.update()


if __name__ == "__main__":
    import pygame

    # 1. Set up pygame and the display
    os.putenv("SDL_FBDEV", "/dev/fb1")
    pygame.init()
    lcd = pygame.display.set_mode((1960, 1080))  # Increase to a larger size
    pygame.mouse.set_visible(False)
    lcd.fill((0, 0, 0))
    pygame.display.update()

    # 2. Setup the RPLidar
    device_address = find_lidar_usb_device()
    lidar = RPLidar(None, device_address)

    is_alive = True
    print(lidar.info)
    while is_alive:
        try:
            scan_data = [0] * TOTAL_NUM_ANGLES
            for scan in lidar.iter_scans():
                for _, angle, distance in scan:
                    # preprocessing
                    # distance is in milimeters
                    scan_data[min(359, floor(angle))] = distance
                process_data(scan_data, lcd)
        except KeyboardInterrupt:
            is_alive = False
            print("Stoping.")
        except Exception:
            pass
    lidar.stop()
    lidar.disconnect()
