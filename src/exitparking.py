import ros_robot_controller_sdk as rcc
import time

def unpark(board: rcc.Board, TRACK_DIR: int) -> None:
    def pwm(degree):  
        pw = round(degree * 11.1 + 1500)
        return max(0, min(65535, pw))
    board.pwm_servo_set_position(0.1, [[1, pwm(45)]])
    board.pwm_servo_set_position(0.1, [[2, 1600]])
    time.sleep(0.5)

    board.pwm_servo_set_position(0.1, [[1, pwm(-45)]])
    board.pwm_servo_set_position(0.1, [[2, 1380]])
    time.sleep(0.5)

    board.pwm_servo_set_position(0.1,[[1,pwm(45)]])
    board.pwm_servo_set_position(0.1,[[2,1600]])
    time.sleep(1.1)

    board.pwm_servo_set_position(0.1,[[1,pwm(0)]])
    board.pwm_servo_set_position(0.1,[[2,1600]])
    time.sleep(0.8)

    board.pwm_servo_set_position(0.1,[[1,pwm(-45)]])
    board.pwm_servo_set_position(0.1,[[2,1380]])
    time.sleep(0.8)

    board.pwm_servo_set_position(0.1, [[1, pwm(0)]])
    board.pwm_servo_set_position(0.1, [[2, 1600]])
    time.sleep(2)

    print("Choosing direction")


    # car current facing outwars right now

    if TRACK_DIR == 1:
        board.pwm_servo_set_position(0.1, [[1, pwm(45)]])
        board.pwm_servo_set_position(0.1, [[2, 1600]])
        time.sleep(2)

        board.pwm_servo_set_position(0.1, [[1, pwm(-15)]])
        board.pwm_servo_set_position(0.1, [[2, 1380]])
        time.sleep(1.7)


    else:
        board.pwm_servo_set_position(0.1, [[1, pwm(0)]])
        board.pwm_servo_set_position(0.1, [[2, 1600]])
        time.sleep(3.5)

        board.pwm_servo_set_position(0.1,[[1,pwm(45)]])
        board.pwm_servo_set_position(0.1,[[2,1380]])
        time.sleep(2)

        board.pwm_servo_set_position(0.1,[[1,pwm(-10)]])
        board.pwm_servo_set_position(0.1,[[2,1380]])
        time.sleep(0.3)

    board.pwm_servo_set_position(0.1, [[1, pwm(0)]])
    board.pwm_servo_set_position(0.1, [[2, 1500]])
    time.sleep(0.1)









import ros_robot_controller_sdk as rcc
import time

def unpark(board: rcc.Board, TRACK_DIR: int) -> None:
    def pwm(degree):  
        pw = round(degree * 11.1 + 1500)
        return max(0, min(65535, pw))
    
    if TRACK_DIR == -1:
        board.pwm_servo_set_position(0.1, [[1, pwm(45)]])
        board.pwm_servo_set_position(0.1, [[2, 1600]])
        time.sleep(0.5)

        board.pwm_servo_set_position(0.1, [[1, pwm(-45)]])
        board.pwm_servo_set_position(0.1, [[2, 1380]])
        time.sleep(0.5)

        board.pwm_servo_set_position(0.1,[[1,pwm(45)]])
        board.pwm_servo_set_position(0.1,[[2,1600]])
        time.sleep(1.1)

        board.pwm_servo_set_position(0.1,[[1,pwm(0)]])
        board.pwm_servo_set_position(0.1,[[2,1600]])
        time.sleep(0.8)

        board.pwm_servo_set_position(0.1,[[1,pwm(-45)]])
        board.pwm_servo_set_position(0.1,[[2,1380]])
        time.sleep(0.8)

        board.pwm_servo_set_position(0.1, [[1, pwm(0)]])
        board.pwm_servo_set_position(0.1, [[2, 1600]])
        time.sleep(2)

        print("Choosing direction")


        # car current facing outwars right now
        board.pwm_servo_set_position(0.1, [[1, pwm(0)]])
        board.pwm_servo_set_position(0.1, [[2, 1600]])
        time.sleep(3.5)

        board.pwm_servo_set_position(0.1,[[1,pwm(45)]])
        board.pwm_servo_set_position(0.1,[[2,1380]])
        time.sleep(2)

        board.pwm_servo_set_position(0.1,[[1,pwm(-10)]])
        board.pwm_servo_set_position(0.1,[[2,1380]])
        time.sleep(0.3)
        
    else:
        board.pwm_servo_set_position(0.1, [[1, pwm(-45)]])
        board.pwm_servo_set_position(0.1, [[2, 1600]])
        time.sleep(0.5)

        board.pwm_servo_set_position(0.1, [[1, pwm(45)]])
        board.pwm_servo_set_position(0.1, [[2, 1380]])
        time.sleep(0.5)

        board.pwm_servo_set_position(0.1,[[1,pwm(-45)]])
        board.pwm_servo_set_position(0.1,[[2,1600]])
        time.sleep(1.1)

        board.pwm_servo_set_position(0.1,[[1,pwm(0)]])
        board.pwm_servo_set_position(0.1,[[2,1600]])
        time.sleep(0.8)

        board.pwm_servo_set_position(0.1,[[1,pwm(45)]])
        board.pwm_servo_set_position(0.1,[[2,1380]])
        time.sleep(0.8)

        board.pwm_servo_set_position(0.1, [[1, pwm(0)]])
        board.pwm_servo_set_position(0.1, [[2, 1600]])
        time.sleep(2)
        print("facing out")

        board.pwm_servo_set_position(0.1, [[1, pwm(0)]])
        board.pwm_servo_set_position(0.1, [[2, 1600]])
        time.sleep(3.5)

        board.pwm_servo_set_position(0.1,[[1,pwm(-45)]])
        board.pwm_servo_set_position(0.1,[[2,1380]])
        time.sleep(2)

        board.pwm_servo_set_position(0.1,[[1,pwm(10)]])
        board.pwm_servo_set_position(0.1,[[2,1380]])
        time.sleep(0.3)

    board.pwm_servo_set_position(0.1, [[1, pwm(0)]])
    board.pwm_servo_set_position(0.1, [[2, 1500]])
    time.sleep(0.1)

if __name__ == "__main__":
    board = rcc.Board()
    unpark(board, -1)
