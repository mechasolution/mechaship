# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of {copyright_holder} nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

# Author: Bishop Pearson

import os
import select
import sys
import rclpy

from rclpy.qos import QoSProfile

from mechaship_interfaces.srv import ThrottlePercentage, Key

if os.name == "nt":
    import msvcrt
else:
    import termios
    import tty

OMO_R1MINI_MAX_LIN_VEL = 0.50
OMO_R1MINI_MAX_ANG_VEL = 5.70

THROTTLE_STEP_SIZE = 5
KEY_STEP_SIZE = 5

msg = """
Control Your mechaship!
---------------------------
Moving around:
        w
   a    s    d
        x
w/x : increase/decrease throttle percentage (0~100%)
a/d : increase/decrease key degree (60~120°)
space key, s : force stop
CTRL-C to quit
"""

e = """
Communications Failed
"""


def get_key(settings):
    if os.name == "nt":
        return msvcrt.getch().decode("utf-8")
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ""

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_vels(target_throttle, target_key):
    print(
        "currently:\tthrottle percentage {0}%\t key degree {1}°".format(
            target_throttle, target_key
        )
    )


def make_simple_profile(output, input, slop):
    if input > output:
        output = min(input, output + slop)
    elif input < output:
        output = max(input, output - slop)
    else:
        output = input

    return output


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_linear_limit_velocity(velocity):
    return constrain(velocity, 0, 100)


def check_angular_limit_velocity(velocity):
    return constrain(velocity, 60, 120)


def main():
    settings = None
    if os.name != "nt":
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node("teleop_keyboard")
    throttle_client = node.create_client(
        ThrottlePercentage, "actuators/throttle/set_percentage"
    )
    key_client = node.create_client(Key, "actuators/key/set")

    status = 0
    target_throttle = 0
    target_key = 90

    try:
        print(msg)
        while 1:
            key = get_key(settings)
            if key == "w":
                target_throttle = check_linear_limit_velocity(
                    target_throttle + THROTTLE_STEP_SIZE
                )
                status = status + 1
                print_vels(target_throttle, target_key)
            elif key == "x":
                target_throttle = check_linear_limit_velocity(
                    target_throttle - THROTTLE_STEP_SIZE
                )
                status = status + 1
                print_vels(target_throttle, target_key)
            elif key == "a":
                target_key = check_angular_limit_velocity(target_key - KEY_STEP_SIZE)
                status = status + 1
                print_vels(target_throttle, target_key)
            elif key == "d":
                target_key = check_angular_limit_velocity(target_key + KEY_STEP_SIZE)
                status = status + 1
                print_vels(target_throttle, target_key)
            elif key == " " or key == "s":
                target_throttle = 0
                target_key = 90
                print_vels(target_throttle, target_key)
            else:
                if key == "\x03":
                    break
                continue

            if status == 20:
                print(msg)
                status = 0

            throttle_set = ThrottlePercentage.Request()
            key_set = Key.Request()

            throttle_set.percentage = target_throttle
            key_set.degree = target_key

            throttle_client.call_async(throttle_set)
            key_client.call_async(key_set)

    except Exception as e:
        print(e)

    finally:
        throttle_set = ThrottlePercentage.Request()
        key_set = Key.Request()

        throttle_set.percentage = 0
        key_set.degree = 90

        throttle_client.call_async(throttle_set)
        key_client.call_async(key_set)

        if os.name != "nt":
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    main()
