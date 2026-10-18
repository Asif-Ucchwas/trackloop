import sys


def test_no_ros_paths_on_sys_path():
    leaked = [p for p in sys.path if "/opt/ros" in p or "ros2_ws" in p]
    assert not leaked, f"ROS paths on sys.path: {leaked}. Run 'unset PYTHONPATH' after activating the venv."
