import subprocess

command = "source ~/fe_ws/install/setup.bash && ros2 run one_file obs"

process = subprocess.Popen(
    ["bash", "-c", command],
    stdout=subprocess.PIPE,
    stderr=subprocess.STDOUT,
    text=True,
    bufsize=1,
    executable="/bin/bash"
)

try:
    for line in process.stdout:
        print(f"[ROS2 OUT]: {line.strip()}")
except KeyboardInterrupt:
    print("Stopping ROS2 process...")
finally:
    process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
