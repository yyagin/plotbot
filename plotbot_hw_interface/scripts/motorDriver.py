#!/usr/bin/env python3

import rospy
import math
import pigpio
from sensor_msgs.msg import JointState  
import rotaryEncoder 
from simple_pid import PID


JOINT_NAMES = ["left_wheel_joint", "right_wheel_joint"]
PINS = [[12, 5, 13, 23, 24], [18, 21, 20, 27, 17]]

class Motor:
    def __init__(self, NAME, PWM, DIR_1, DIR_2, ENC_A, ENC_B) -> None:
        self.pi = pigpio.pi()
        self.name = NAME
        self.kp = 5.0
        self.ki = 5.0
        self.kd = 0.0
        self.pid = PID(Kp=self.kp, Ki=self.ki, Kd=self.kd, setpoint=0.0, auto_mode=True)
        self.pid.output_limits = (0,100)
        
        self.DIR_1 = DIR_1
        self.DIR_2 = DIR_2
        self.PWM = PWM
        self.ENC_A = ENC_A
        self.ENC_B = ENC_B
        
        self.cpr = 16
        self.frequency = 1000
        self.gear_ratio = 131.25
        self.wheel_diameter = 0.08

        self.last_count = 0
        self.last_time = rospy.Time.now()
        self.last_time_pid = rospy.Time.now()
        self.steps = 0
        self.speed = 0.0
        self.position = 0.0
        self.travel_per_pulse = (self.wheel_diameter * math.pi) / (self.gear_ratio * self.cpr)
        
        self.pi.set_mode(DIR_1, pigpio.OUTPUT)
        self.pi.set_mode(DIR_2, pigpio.OUTPUT)
        self.pi.set_mode(self.PWM, pigpio.OUTPUT)
        # self.encoder = RotaryEncoder(a = ENC_A,b = ENC_B, max_steps=0)
        self.decoder = rotaryEncoder.decoder(self.pi, self.ENC_A, self.ENC_B, self.encoderCallback)
   
    def encoderCallback(self,way) -> None:
        self.steps -= way
         
    def getName(self):
        return self.name
    
    def getCount(self) -> int:
        return self.steps
    
    def getSpeed(self) -> float:
        return self.speed

    def getPosition(self) -> float:
        return self.position

    def measure(self) -> None:
        count = self.steps
        self.count_diff = count - self.last_count
        pos_diff =  self.travel_per_pulse * float(self.count_diff)
        self.position += pos_diff
        
        # counts_per_min =  (float(self.count_diff) / float((rospy.Time.now() - self.last_time).to_sec())) * 60
        # motor_rpm = counts_per_min / ( self.cpr * self.gear_ratio)
        self.speed = ((self.count_diff / ( self.cpr * self.gear_ratio)) / float((rospy.Time.now() - self.last_time).to_sec())) * 60.0
        
    
        
        self.last_count = count
        self.last_time = rospy.Time.now()
        
    def forward(self, cmd) -> None:
        self.pi.write(self.DIR_2, pigpio.HIGH)  
        self.pi.write(self.DIR_1, pigpio.LOW)
        self.pi.hardware_PWM(self.PWM, self.frequency, cmd * 100000)

    def backward(self, cmd) -> None:
        self.pi.write(self.DIR_1, pigpio.HIGH)  
        self.pi.write(self.DIR_2, pigpio.LOW)
        self.pi.hardware_PWM(self.PWM, self.frequency, cmd * 100000)
        
    def command(self,cmd) -> None:
        self.pid.setpoint = abs(cmd)
        print("cmd: "+ str(cmd))
        self.measure()
        output = int(self.pid(abs(self.speed),float((rospy.Time.now() - self.last_time_pid).to_sec())))
        self.last_time_pid = rospy.Time.now()
        print("Speed: "+ str(self.speed))
        print("output: "+ str(output))
        if cmd > 1:
            self.forward(abs(output))
        elif cmd < -1:
            self.backward(abs(output))
        else:
            self.forward(0)

    def stop(self):
        self.pi.hardware_PWM(self.PWM, self.frequency, 0 * 10000)
        self.pid.reset()
        
    def __exit__(self) -> None:
        self.decoder.cancel()
        self.pi.stop()



class MotorDriver:
    def __init__(self) -> None:
        self.frequency = rospy.get_param("frequency", 50)
        self.command_topic = "/motor/commands"
        self.state_topic = "/motor/states"

        self.subs = rospy.Subscriber(
            self.command_topic, JointState, self.command_callback, queue_size=1
        )
        self.pub = rospy.Publisher(self.state_topic, JointState, queue_size=1)
        self.motors = []
        
        i = 0
        for jn in JOINT_NAMES:
            pins = PINS[i]
            self.motors.insert(
                i,
                Motor(
                    NAME=jn,
                    PWM=pins[0],
                    DIR_1=pins[1],
                    DIR_2=pins[2],
                    ENC_A=pins[3],
                    ENC_B=pins[4],
                ),
            )
            i += 1

    def __exit__(self):
        for m in self.motors:
            m.forward(0)

    def command_callback(self, msg):
        state = JointState()
        for i in range(len(JOINT_NAMES)):
            cmd = msg.velocity[i]
            self.motors[i].command(cmd)
        for i in range(len(JOINT_NAMES)):
            state.name.append(self.motors[i].getName())
            state.velocity.append(self.motors[i].getSpeed())
            state.position.append(self.motors[i].getPosition())
        self.pub.publish(state)

    def stop(self):
        for m in self.motors:
            m.stop()
        
    def test(self):
        for m in self.motors:
            # cmd = 30
            # m.forward(cmd)
            m.measure()
            # print(m.getCount())
            # print(m.getSpeed())
            # print(m.getPosition())

if __name__ == "__main__":
    rospy.init_node("driver_node")
    driver = MotorDriver()

    # rate = rospy.Rate(50)
    # while not rospy.is_shutdown():
    #     driver.test()
    #     # print("test")
    #     rate.sleep()
    rospy.spin()
    driver.stop()

