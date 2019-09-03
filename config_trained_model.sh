rm -rf ~/catkin_ws/src/AI_Visualguidance/darknet_ros/darknet_ros/yolo_network_config/weights/yolov3_final1_endale.weights
cp -rf ~/Desktop/trained_model/yolov3_final1_endale.weights ~/catkin_ws/src/AI_Visualguidance/darknet_ros/darknet_ros/yolo_network_config/weights/

rm -rf ~/catkin_ws/src/AI_Visualguidance/darknet_ros/darknet_ros/yolo_network_config/cfg/yolov3_endale1.cfg
cp -rf ~/Desktop/trained_model/yolov3_endale1.cfg ~/catkin_ws/src/AI_Visualguidance/darknet_ros/darknet_ros/yolo_network_config/cfg/

rm -rf ~/catkin_ws/src/AI_Visualguidance/darknet_ros/darknet_ros/config/yolov3_custom.yaml
cp -rf ~/Desktop/trained_model/yolov3_custom.yaml ~/catkin_ws/src/AI_Visualguidance/darknet_ros/darknet_ros/config/