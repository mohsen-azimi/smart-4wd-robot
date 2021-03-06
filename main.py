"""
Smart 4WD Robot for SHM
Copyright (c) Mohsen Azimi, 2022
"""

import argparse
import cv2
import numpy as np

from sensors.realsense_l515.camera import L515
import utils.cv2_utils as utils
from robots import AGV, Fake_AGV


def get_args_parser():
    parser = argparse.ArgumentParser('Set Robot parameters', add_help=False)

    # AGV
    parser.add_argument('--controller', default='aruco', type=str,
                        help="Select the controller, keyboard, aruco, auto, [Telegram]")
    parser.add_argument('--wheel_speed', default=30, type=int,
                        help="Try and find one suitable based on the DC motors")
    # --port
    parser.add_argument('--port', default='/dev/ttyUSB0', type=str,
                        help="Select the port (/dev/ttyUSB0 for linux, COM# for windows), dmesg | grep ttyUSB,"
                             " if failed: sudo usermod -a -G tty $USER &&  sudo usermod -a -G dialout $USER")
    parser.add_argument('--baudrate', default=9600, type=int)
    parser.add_argument('--timeout', default=0.1, type=float)

    # -- Camera (RGB-d)
    # parser.add_argument('--camera', default='l515', type=str,
    #                     help="Select the sensor type")
    # parser.add_argument('--imshow', default=False, type=bool,
    #                     help="Show RGB-D frames")

    # -- uav?
    parser.add_argument('--with_uav', default=False, type=bool,
                        help="Select if the AGV communicates with a UAV")

    # Python
    parser.add_argument('--seed', default=66, type=int)
    parser.add_argument('--output_dir', default='output/',
                        help='path where to save')
    # --model
    parser.add_argument('--detector', default='ArUco',
                        help='select the detector, ArUco, yolov5s, etc.')
    return parser


def main(args):
    print(args)
    if args.detector == 'ArUco':
        obj_detector = utils.ArUco_marker()

    robot = AGV(port=args.port, baudrate=args.baudrate, timeout=args.timeout, wheel_speed=args.wheel_speed)
    # robot = Fake_AGV(port=args.port, baudrate=args.baudrate, timeout=args.timeout, wheel_speed=args.wheel_speed)
    camera = L515(read_bag=0, record_bag=0)

    # camera.reset()
    while True:
        # This call waits until a new coherent set of frames is available on a device maintain frame timing
        frame = camera.get_frame()
        color_image = frame.color_image
        depth_image = frame.depth_image
        # ir_image = frame.ir_image
        # accel = frame.accel
        # gyro = frame.gyro
        # pcd = frame.point_cloud

        # depth_clipped = camera.clip_distance(depth_image, color_image, 1, 3)

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        # depth_image_colorised = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=.03), cv2.COLORMAP_JET)

        # # Flip image ?
        # color_image = cv.flip(color_image, 1)
        # depth_image_colorised = cv.flip(depth_image_colorised, 1)
        # infrared = cv.flip(infrared, 1)

        obj_detector.find_object(color_image, depth_image, camera.depth_scale,
                                 obj_id=1, draw=True)

        if obj_detector.objects:
            for obj in obj_detector.objects:
                if obj['id'] == 1:
                    print(obj['distance'])
                    # direction = 'moveForward'
                    (w, h, _) = color_image.shape

                    if obj['distance'] > 0.5:

                        if obj['center'][0] / w < 0.5 - 0.1:
                            direction = 'rotateLeft'
                        elif obj['center'][0] / w > 0.5 + 0.1:
                            direction = 'rotateRight'
                        else:
                            direction = 'moveForward'

                        robot.move(direction=direction).to_arduino()
                        robot.cache = direction
                        print(direction)

                    else:

                        if obj['center'][0] / w < 0.5 - 0.02:
                            direction = 'rotateLeft'
                            robot.move(direction=direction).to_arduino()
                            robot.cache = direction

                        elif obj['center'][0] / w > 0.5 + 0.02:
                            direction = 'rotateRight'
                            robot.move(direction=direction).to_arduino()
                            robot.cache = direction
                        else:
                            robot.stop().to_arduino()
                            robot.cache = None




        else:
            robot.stop().to_arduino()
            robot.cache = None


        #
        #
        #
        # else:
        #     robot.stop().to_arduino()
        #     robot.cache = None

        if camera.enable_rgbd:
            utils.im_show(640, 480, ('color_image', color_image))

            # for (marker_corner, marker_id) in zip(corners, ids):
            #     # print(marker_id, "-----")
            #     if marker_id[0] == 1:
            #         # print(robot.moves[marker_id[0] - 1])
            #         break

            #
            # # cv.namedWindow('IR', cv.WINDOW_AUTOSIZE)
            # # cv.imshow('IR', infrared)
            #
            # # cv.setMouseCallback('Depth', mouse_coord)  # to show distance on mouse
            # # # Show distance for a specific point
            # # cv.circle(depth_image_colorised, point, 5, (0, 0, 255))
            # # distance = depth_image[point[1], point[0]] * camera.depth_scale
            # #
            # # cv.putText(depth_image_colorised, f'{distance:.3f} m', (point[0], point[1] - 20), cv.FONT_HERSHEY_PLAIN, 2,
            # #            (0, 255, 255), 4)
            #

            # # save to png
            # if camera.save_png:
            #     if i % 1 == 0:
            #         cv.imwrite('outputs/depth/depth_' + str(i) + '.png', depth_image_colorised)
            #         cv.imwrite('outputs/depth_clipped/depth_clipped_' + str(i) + '.png', depth_clipped)
            #         cv.imwrite('outputs/color/color_' + str(i) + '.png', color_image)
            #
            # i += 1
            # print(i)
            # print(pcd)
            # o3d.visualization.draw_geometries(pcd, zoom=.8)

        if cv2.waitKey(1) & 0xff == 27:  # 27 = ESC
            break

    # finally:
    # Stop streaming
    camera.pipeline.stop()


if __name__ == '__main__':
    parser = argparse.ArgumentParser('Robot', parents=[get_args_parser()])
    args = parser.parse_args()
    # if args.output_dir:
    #     Path(args.output_dir).mkdir(parents=True, exist_ok=True)
    main(args)
