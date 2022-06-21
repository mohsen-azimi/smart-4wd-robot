"""
Smart 4WD Robot for SHM
Copyright (c) Mohsen Azimi, 2022
"""

import argparse

def get_args_parser():
    parser = argparse.ArgumentParser('Set Robot parameters', add_help=False)


    # AGV
    parser.add_argument('--controller', default='keyboard', type=str,
                        help="Select the controller, keyboard, auto, [Telegram]")

    # -- Camera (RGB-d)
    parser.add_argument('--camera', default='l515', type=str,
                        help="Select the sensor type")
    parser.add_argument('--imshow', default=False, type=bool,
                        help="Show RGB-D frames")


    # --port
    parser.add_argument('--port', default='/dev/ttyUSB0', type=str,
                        help="Select the port for arduino (/dev/ttyUSB0 for linux, COM# for windows")
    parser.add_argument('--baudrate', default=9600, type=int)
    parser.add_argument('--timeout', default=0.1, type=float)

    # -- uav?
    parser.add_argument('--with_uav', default=False, type=bool,
                        help="Select if the AGV communicates with a UAV")

    # Python
    parser.add_argument('--seed', default=66, type=int)
    parser.add_argument('--output_dir', default='output/',
                        help='path where to save')
    # --model
    parser.add_argument('--model', default='yolov5s.pt')

    # more parser: https://github.com/mohsen-azimi/detr/blob/3513d7d3a4aaee1da9aa0e22c365ffb64922eb15/main_face.py#L20
    return parser


def main(args):
    print(args)


if __name__ == '__main__':
    parser = argparse.ArgumentParser('DETR training and evaluation script', parents=[get_args_parser()])
    args = parser.parse_args()
    # if args.output_dir:
    #     Path(args.output_dir).mkdir(parents=True, exist_ok=True)
    main(args)
