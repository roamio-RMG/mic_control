
build the package :
    colcon build --packages-select respeaker_ros
    colcon build --packages-select keyboard_control

uploade the arduino code :
    chmod +x src/respeaker_ros/download_arduino/upload_keyboard.py 
    ./src/respeaker_ros/download_arduino/upload_keyboard.py 


launch 
ros2 launch respeaker_ros respeaker_doa.launch.py

./fix_respeaker_usb.py




create any new package 
    ros2 pkg create --build-type ament_python keyboard_control