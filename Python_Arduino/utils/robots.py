import numpy as np

import serial  # pip install pyserial
import time
from pynput import keyboard
from djitellopy import tello


# for Linux:
# ser = serial.Serial(port='/dev/ttyAMC0',
#                     baudrate=9600,
#                     bytesize=serial.EIGHTBITS,
#                     parity=serial.PARITY_NONE,
#                     stopbits=serial.STOPBITS_ONE,
#                     timeout=5,
#                     xonxoff=False,
#                     rtscts=False,
#                     dsrdtr=False,
#                     writeTimeout=2)




class UGV:
    def __init__(self, port='COM11', baudrate=9600, timeout=.1, wheel_speed=30):
        self.max_speed = 200
        self.wheel_speed = wheel_speed
        self.motion_time = 1  # second
        self.cache = None
        # self.move_uav = None
        print("wait...")
        time.sleep(1)
        print("Robot is ready!")

        # Default direction: LeftFrontWheel, LeftBackWheel,RightFrontWheel, RightBackWheel
        self.dir = np.array([0, 0, 0, 0])
        self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)


    #     self.drone = tello.Tello()
    #     self.drone.speed = 10
    #
    #
    # def drone_conect(self):
    #     self.drone.connect()
    #     return self
    #
    # def drone_takeoff(self):
    #     self.drone.takeoff()
    #     return self
    #
    # # def rotate(self, angle):
    # #     if angle > 0:
    # #         self.drone.rotate_clockwise(angle)
    # #     else:
    # #         self.drone.rotate_counter_clockwise(angle)
    #
    # def drone_move(self, x, y, z):
    #     self.drone.go_xyz_speed(x, y, z, self.drone.speed)

    def move(self, direction='rotate_right'):
        """
        Directions: moveForward, moveBackward, rotateRight, etc.
        speed: [-255, 255]
        duration:  milliseconds
        """

        if direction == 'moveForward':
            self.dir = np.array([1, 1, 1, 1])
        elif direction == 'moveBackward':
            self.dir = np.array([-1, -1, -1, -1])
        elif direction == 'moveSidewaysRight':
            self.dir = np.array([1, -1, -1, 1])
        elif direction == 'moveSidewaysLeft':
            self.dir = np.array([-1, 1, 1, -1])
        elif direction == 'rotateLeft':
            self.dir = np.array([-1, -1, 1, 1])
        elif direction == 'rotateRight':
            self.dir = np.array([1, 1, -1, -1])
        elif direction == 'moveRightForward':
            self.dir = np.array([1, 0, 0, 1])
        elif direction == 'moveRightBackward':
            self.dir = np.array([0, -1, -1, 0])
        elif direction == 'moveLeftForward':
            self.dir = np.array([0, 1, 1, 0])
        elif direction == 'moveLeftBackward':
            self.dir = np.array([-1, 0, 0, -1])
        return self

    def stop(self):
        self.dir = np.array([0, 0, 0, 0])
        return self

    def test(self, wheel=0):
        """ 
        Test if each wheel works
        """
        self.dir = np.array([0, 0, 0, 0])
        self.dir[direction]=1
        return self

    def keyboard_on_press(self, key):
        print(key)
        if key == self.cache:
            pass
        elif key == keyboard.Key.up:
            self.cache = key
            self.move(direction='moveForward').to_arduino()
            # self.move_uav = 'move_Forward'
        elif key == keyboard.Key.down:
            self.cache = key
            self.move(direction='moveBackward').to_arduino()
        elif key == keyboard.Key.right:
            self.cache = key
            self.move(direction='rotateRight').to_arduino()
            # self.drone_move(20, 0, 0)
        elif key == keyboard.Key.left:
            self.cache = key
            self.move(direction='rotateLeft').to_arduino()

        elif key == keyboard.KeyCode.from_char('l'):
            self.cache = key
            self.move(direction='moveSidewaysLeft').to_arduino()
        elif key == keyboard.KeyCode.from_char('r'):
            self.cache = key
            self.move(direction='moveSidewaysRight').to_arduino()

        elif key == keyboard.KeyCode.from_char(']'):
            self.wheel_speed += 5
        elif key == keyboard.KeyCode.from_char('['):
            self.wheel_speed -= 5


    def keyboard_on_release(self, key):
        self.stop().to_arduino()
        self.cache = None

        # print(self.wheel_speed)

        # print(type(key))

        if key == keyboard.Key.esc:
            # Stop listener
            return False

    def control_by_keyboard(self):
        """
        Control the robot using the keyboard keys (listed above)
        :return:
        """
        with keyboard.Listener(
                on_press=self.keyboard_on_press,
                on_release=self.keyboard_on_release) as listener:
            listener.join()

    def calibrate(self):
        """
        Rotate 360 to calibrate rotation speed for surfaces
        :return: calculate time to rotate 90-degrees (not implemented yet!)
        """
        self.dir = [1, 1, -1, -1]  # rotate right
        '''some codes here!'''
        self.stop()  # stop after calibrating
        return self

    def to_arduino(self):
        """
        Send commands to arduino in one string
        :return: print the received commands by arduino
        """
        servo_speeds = self.dir * self.wheel_speed
        serial_command = format(servo_speeds[0], '04d') + format(servo_speeds[1], '04d') + \
                         format(servo_speeds[2], '04d') + format(servo_speeds[3], '04d')

        self.serial.write(bytes(serial_command, 'utf-8'))
        # time.sleep(self.motion_time / 1000)
        received_data = self.serial.readline()
        print(received_data)
