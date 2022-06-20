# Robot Controller

## rosbridge_server
roslaunch rosbridge_server rosbridge_websocket.launch address:=localhost

## object detection
python3 /root/catkin_ws/src/ros_yolo/scripts/final_yolo.py

python3 ~/catkin_ws/src/yolov5-face/final_yolo5face.py

roslaunch obstacle_detector nodes.launch
roslaunch robot-photographer obstacle_detector.launch

## Scan
roslaunch velodyne_pointcloud VLP16_points.launch

## Odom
python ~/catkin_ws/src/robot-photographer/pub_odom.py

## SLAM
rosrun tf static_transform_publisher 0 0 0 0 0 0 map odom 100

rosrun tf static_transform_publisher 0 0 0 0 0 0 base_link velodyne 100

roslaunch robot-photographer gmapping_test.launch

## teleop
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/diff_drive_controller/cmd_vel

python catkin_ws/src/turtlebot3/turtlebot3_teleop/nodes/turtlebot3_teleop_key

## Main
cd ~/catkin_ws/src/robot-photographer/scripts/
python robot_photographer.py