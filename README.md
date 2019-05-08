# AI_Visualguidance

## launch zed_wrapper node from zed-ros-wrapper

	mkdir -p ~/catkin_ws/src
	cd ~/catkin_ws/src
	git clone https://github.com/usdl2019/AI_Visualguidance.git 

	chmod +x AI_Visualguidance/zed-ros-wrapper/zed_wrapper/cfg/Zed.cfg
        copy training weights and cfg files
        cp /path/xxx.weights ~/catkin_ws/src/AI_Visualguidance/darknet_ros/darknet_ros/yolo_network_config/weights/
        cp /path/xxx.cfg    ~/catkin_ws/src/AI_Visualguidance/darknet_ros/darknet_ros/yolo_network_config/cfg
	Then modify 
	/AI_Visualguidance/darknet_ros/darknet_ros/config/yolov3_custom.yaml 

	cd ~/catkin_ws

	catkin_make
	source devel/setup.bash
	roslaunch zed_wrapper zed.launch

## launch zed_data_subscribe node

 	roslaunch zed_data_subscribe zed_data.launch
