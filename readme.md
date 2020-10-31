roscore

rosrun ros_control_example my_robot_node 

rosparam load `rospack find ros_control_example`/config/controllers.yaml

ROBOT_URDF=`rospack find ros_control_example`/description/my_robot.urdf
rosparam set robot_description "`cat $ROBOT_URDF`"

rosservice call /my_robot/controller_manager/load_controller "name: 'joint_state_controller'"

rostopic list

rosservice call /my_robot/controller_manageswitch_controller "start_controllers: ['joint_state_controller']
stop_controllers: ['']
strictness: 0
start_asap: false
timeout: 0.0"

rostopic echo /my_robot/joint_states

rosservice call /my_robot/controller_manager/load_controller "name: 'joint_1_position_controller'"
rosservice call /my_robot/controller_manager/load_controller "name: 'joint_2_position_controller'"
rosservice call /my_robot/controller_manager/switch_controller "start_controllers: ['joint_1_position_controller', 'joint_2_position_controller']  
stop_controllers: ['']
strictness: 0
start_asap: false
timeout: 0.0"

rostopic pub /my_robot/joint_1_position_controller/command std_msgs/Float64 "data: 1.0"
