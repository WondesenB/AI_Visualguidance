# AI_Visualguidance

## launch zed_wrapper node from zed-ros-wrapper

	mkdir -p ~/catkin_ws/src
	cd ~/catkin_ws/src
	git clone https://github.com/usdl2019/AI_Visualguidance.git 
<<<<<<< HEAD
	chmod +x AI_Visualguidance/zed-ros-wrapper/zed_wrapper/cfg/Zed.cfg
        cd ~/catkin_ws
=======
	cd ~/catkin_ws
>>>>>>> 1b7f380c60f5538edbf8c945e5b8c519830c2bcb
	catkin_make
	source devel/setup.bash
	roslaunch zed_wrapper zed.launch

## launch zed_data_subscribe node

 	roslaunch zed_data_subscribe zed_data.launch
