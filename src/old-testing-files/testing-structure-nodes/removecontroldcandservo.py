
import sys
import time
import ros_robot_controller_sdk as rcc
from readchar import readkey, key

board = rcc.Board()
def pwm(degree):
	return round(degree*11.1 + 1500)
mid = 20
max_turn_degree = 60
board.pwm_servo_set_position(0.1, [[1, pwm(mid)]])
# 'Arm' the ESC
board.pwm_servo_set_position(6, 1500, 100) 
print("Ready\n")


if __name__ == '__main__':
    b = 1500
    s = mid
    while True:
        k = readkey()
        #remote control system
         
        if k == ' ':
            board.pwm_servo_set_position(0.1, [[2, 1500]])
            board.pwm_servo_set_position(0.1, [[1, pwm(mid)]])
            time.sleep(0.01)
            print("stop")
            break
        elif k == 'a':
            s = mid+max_turn_degree
            print("left")
        elif k == 'd':
            s = mid-max_turn_degree
            print("right")
        elif k == 'w':
            b = 1550
            print("forward")
        elif k == 's':
            b = 1400
            print("backward")
        elif k == 'x':
            s = mid
            print("straight")
        elif k == 'q':
            if b >= 1500:
                b += 50
                if b>2000:
                    b = 2000
            else:
                b -= 50
                if b <1000:
                    b = 1000
            print(f"speed up: {b}")
        elif k == 'e':
            if b >= 1500:
                b -= 50
                if b < 1500:
                    b = 1500
            else:
                b += 50
                if b > 1500:
                    b = 1500
            print(f"speed down: {b}")
        #bldc
        pw = pwm(s)
        board.pwm_servo_set_position(0.1, [[1, pw]])
        board.pwm_servo_set_position(0.1, [[2, b]])
        time.sleep(0.01)