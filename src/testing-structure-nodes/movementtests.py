import time
import ros_robot_controller_sdk as rcc

board = rcc.Board()

MAX_TURN_DEGREE = 50
STRAIGHT_DEGREE = 128

def degreestopulse(deg):
    return int(deg*11.1+500)

board.pwm_servo_set_position(0.1, [[1, degreestopulse(STRAIGHT_DEGREE)]])
time.sleep(3)

#board.pwm_servo_set_position(0.1, [[2, 1500]])

board.pwm_servo_set_position(0.1, [[1, degreestopulse(STRAIGHT_DEGREE+MAX_TURN_DEGREE)]])
time.sleep(3)

#board.pwm_servo_set_position(0.1, [[2, 1550]])

board.pwm_servo_set_position(0.1, [[1, degreestopulse(STRAIGHT_DEGREE)]])
time.sleep(3)

board.pwm_servo_set_position(0.1, [[1, degreestopulse(STRAIGHT_DEGREE-MAX_TURN_DEGREE)]])
time.sleep(3)


"""board.pwm_servo_set_position(0.1, [[2, 1600]])
time.sleep(3)"""
