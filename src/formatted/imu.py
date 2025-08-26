import subprocess, math, time, serial

# ---------------- ROS2 / Docker config ---------------- #
CONTAINER = "MentorPi"
SHELL     = "/bin/zsh"
USER      = "ubuntu"
WORKDIR   = "/home/ubuntu"
ENV_SRC   = "source /home/ubuntu/.zshrc"
TOPIC     = "/imu/rpy/filtered"  # geometry_msgs/Vector3Stamped

# ---------------- IMU streaming ---------------- #
def start_imu_stream():
    """Start a persistent ROS2 echo process that streams IMU messages."""
    cmd = f"{ENV_SRC} && ros2 topic echo {TOPIC}"
    p = subprocess.Popen(
        ["docker", "exec", "-u", USER, "-w", WORKDIR, CONTAINER, SHELL, "-c", cmd],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, bufsize=1
    )
    return p

def read_imu_line(proc):
    """Read one line of IMU output (non-blocking)."""
    return proc.stdout.readline().strip()

def stop_imu_stream(proc):
    """Stop the persistent ROS2 echo process."""
    proc.terminate()
    try:
        proc.wait(timeout=2)
    except subprocess.TimeoutExpired:
        proc.kill()