crazyflie_description
=====================

URDF Model for the Crazyflie Nano Quadcopter from Bitcraze.

## Model

* The model is based on https://github.com/bitcraze/bitcraze-mechanics/blob/master/models/cf_model_s_revA.skp
* Use SketchUp (Tested with 14.1) to update/change the model
* Export using the following options: ![Export settings](export.png)

## Issues

* Inertia matrix not correct yet
* No collision model yet

change xacro into urdf:
rosrun xacro xacro --inorder -o underwater_snakerobot.urdf underwater_snakerobot.urdf.xacro

check urdf model visually:
cd ~/catkin_ws/src
catkin_create_pkg my_urdf_test_pkg urdf
roscd my_urdf_test_pkg
mkdir urdf
gedit urdf/my_simple_model.urdf
sudo apt install liburdfdom-tools
roslaunch urdf_tutorial display.launch model:=urdf/my_simple_model.urdf
