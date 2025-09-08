import ros_robot_controller_sdk as rcc
import time
board = rcc.Board()

def pwm(degree):  
    pw = round(degree * 11.1 + 1500)
    return max(0, min(65535, pw))

MAX_TURN_DEGREE = 55
MID_SERVO = 0

board.pwm_servo_set_position(0.1, [[1, pwm(MID_SERVO)]])

while True:
    angle = int(input("enter angle: "))
    board.pwm_servo_set_position(0.1, [[1, pwm(angle+MID_SERVO)]])