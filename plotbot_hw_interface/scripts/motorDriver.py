#!/usr/bin/env python

import rospy
import numpy as np
from gpiozero import RotaryEncoder
from sensor_msgs import JointState
from gpiozero import Motor

class Encoder:
    """_summary_
    """
    def __init__(self, pin_1, pin_2) -> None:
        self.cpr = 64
        self.gear_ratio = 131.25
        self.pin_1 = pin_1
        self.pin_2 = pin_2
        self.encoder = RotaryEncoder(pin_1, pin_2)

    def measure(self):
        """_summary_
        """
        print("Measure")


class MotorDriver:
    def __init__(self, node_handle) -> None:
        self.node_handle = node_handle
        self.frequency = rospy.get_param("frequency", 50)

        self.command_topic = rospy.get_param("command_topic")
        self.subscriber = rospy.Subscriber(self.command_topic, JointState, self.command_callback, queue_size=1)

        self.encoder_R = Encoder(27, 17)
        self.encoder_L = Encoder(24, 23)

        self.motor_R = Motor(pwm=26, forward = 20, backward = 21)
        self.motor_L = Motor(pwm=12, forward = 5, backward = 13)

    def command_callback(self, msg):
        for cmd in msg:
            
            
        self.motor_R.forward(0.3)
        self.encoder_R.measure()
        self.encoder_L.measure()

# # Assigning parameter values
# ppr = 300.8  # Pulses Per Revolution of the encoder
# tstop = 20  # Loop execution duration (s)
# tsample = 0.02  # Sampling period for code execution (s)
# tdisp = 0.5  # Sampling period for values display (s)

# # Creating encoder object using GPIO pins 24 and 25
# encoder = RotaryEncoder(24, 25, max_steps=0)

# # Initializing previous values and starting main clock
# anglecurr = 0
# tprev = 0
# tcurr = 0
# tstart = time.perf_counter()

# # Execution loop that displays the current
# # angular position of the encoder shaft
# print('Running code for', tstop, 'seconds ...')
# print('(Turn the encoder.)')
# while tcurr <= tstop:
#     # Pausing for `tsample` to give CPU time to process encoder signal
#     time.sleep(tsample)
#     # Getting current time (s)
#     tcurr = time.perf_counter() - tstart
#     # Getting angular position of the encoder
#     # roughly every `tsample` seconds (deg.)
#     anglecurr = 360 / ppr * encoder.steps
#     # Printing angular position every `tdisp` seconds
#     if (np.floor(tcurr/tdisp) - np.floor(tprev/tdisp)) == 1:
#         print("Angle = {:0.0f} deg".format(anglecurr))
#     # Updating previous values
#     tprev = tcurr

# print('Done.')
# # Releasing GPIO pins
# encoder.close()
