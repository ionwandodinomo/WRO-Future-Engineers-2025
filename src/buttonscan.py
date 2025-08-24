import os
import time
import subprocess
import threading
import ros_robot_controller_sdk as rrc

BUTTON_ID_1 = 1
BUTTON_ID_2 = 2


def listen_to_button_events():
    command = 'source /home/ubuntu/.zshrc && ros2 topic echo /ros_robot_controller/button'
    process = subprocess.Popen(
        ['docker', 'exec', '-u', 'ubuntu', '-w', '/home/ubuntu', 'MentorPi', '/bin/zsh', '-c', command],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True
    )

    while True:
        output = process.stdout.readline()
        if output:
            line = output.strip()
            if line.startswith("id:"):
                button_id = int(line.split(":")[1].strip())
            elif line.startswith("state:"):
                state = int(line.split(":")[1].strip())

                if button_id == 1:
                    if state == 1:
                        print("Button 1 pressed")
                elif button_id == 2:
                    if state == 1:
                        print("Button 2 pressed")
                        return
        else:
            continue

def check_node_status():
    command = 'source /home/ubuntu/.zshrc && ros2 topic list'
    result = subprocess.run(['docker', 'exec', '-u', 'ubuntu', '-w', '/home/ubuntu', 'MentorPi', '/bin/zsh', '-c', command], capture_output=True, text=True)
    res = result.stdout
    return '/ros_robot_controller/button' in res

if __name__ == "__main__":
    if check_node_status():
        print("ROS2 node detected")
        listener_thread = threading.Thread(target=listen_to_button_events, daemon=True)
        listener_thread.start()
        while listener_thread.is_alive():
            listener_thread.join(1)
        print("ended")

        