sudo apt-get install ros-melodic-sensor-msgs ros-melodic-pcl-ros ros-melodic-tf
sudo apt-get install ros-melodic-calibration-toolkit ros-melodic-ros-calibration


roslaunch ros_calibration stereo_auto_calibrate.launch

rosrun calibration_toolkit data_collector <data

rosrun ros_calibration stereo_calibration --approximate 0.1 --square 0.033 --size 9x6 --file <data


