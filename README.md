# ur_grasping
INSTALL
	pip2 install robot_controller
	cd src/
	git clone https://github.com/mbed92/arm-lightweight-controller.git
	cd ..
	source devel/setup.bash
	catkin_make

LAUNCH
	source devel/setup.bash
	roscore

	--------IN NEW TAB--------

	source devel/setup.bash
	rosrun ggcnn_kinova_grasping run_ggcnn.py
