
build the package :
    colcon build --packages-select respeaker_ros
    colcon build --packages-select keyboard_control

uploade the arduino code :
    chmod +x arduino_upload/upload_arduino.py 
    ./arduino_upload/upload_arduino.py






create any new package 
    ros2 pkg create --build-type ament_python keyboard_control