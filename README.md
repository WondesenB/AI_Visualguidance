# AI_Visualguidance

## launch zed_wrapper node from zed-ros-wrapper

	mkdir -p ~/catkin_ws/src
	cd ~/catkin_ws/src
	git clone https://github.com/usdl2019/AI_Visualguidance.git 
	cd ~/catkin_ws
	catkin_make
	source devel/setup.bash
	roslaunch zed_wrapper zed.launch

## launch zed_data_subscribe node

 	roslaunch zed_data_subscribe zed_data.launch
