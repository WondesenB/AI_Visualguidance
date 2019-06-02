# AI_Visualguidance

	mkdir -p ~/catkin_ws/src
	cd ~/catkin_ws/src
	git clone https://github.com/usdl2019/AI_Visualguidance.git 
	catkin_make
	

## launch zed_wrapper node from zed-ros-wrapper


	cd ~/catkin_ws
        chmod +x AI_Visualguidance/zed-ros-wrapper/zed_wrapper/cfg/Zed.cfg

	catkin_make
	source devel/setup.bash
	roslaunch zed_wrapper zed.launch

## launch darknet_ros node from darknet_ros 

	cd ~/catkin_ws
	catkin_make
	source devel/setup.bash

        copy training weights and cfg files
        cp /path/xxx.weights ~/catkin_ws/src/AI_Visualguidance/darknet_ros/darknet_ros/yolo_network_config/weights/
        cp /path/xxx.cfg    ~/catkin_ws/src/AI_Visualguidance/darknet_ros/darknet_ros/yolo_network_config/cfg
	Then modify 
	/AI_Visualguidance/darknet_ros/darknet_ros/config/yolov3_custom.yaml 

## launch local_mapping package

 	roslaunch local_mapping local_mapping.launch
## launch mission node
        rosrun mission  mission_node      
## All node launch

      roslaunch ai_drone ai_drone.launch
