#include <iostream>
#include <memory>
#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>


using namespace std;


class SimpleJoint {
public:
  SimpleJoint(double m=1.0, double k=1.0, double b=1.0) : _m(m), _k(k), _b(b), _effort(0.0), _velocity(0.0), _position(0.0) {}
  virtual ~SimpleJoint() {}
  
  void setCommand(double command) { _effort = command; }
  
  double update(double dt=0.01) {    
    double dx1 = -_k/_m * _position - _b/_m * _velocity + 1.0/_m * _effort;
    double dx2 = _velocity;
    
    _velocity += dx1 * dt;
    _position += dx2 * dt;
    
    return _position;
  }
  
  double getEffort() const { return _effort; }
  double getVelocity() const { return _velocity; }
  double getPosition() const { return _position; }

private:
  double _m, _k, _b;
  double _effort, _velocity, _position;
};


class MyRobot : public hardware_interface::RobotHW {
public:
  MyRobot(ros::NodeHandle& nh) : _nh(nh) {
    // set up the state interface
    hardware_interface::JointStateHandle joint_state_handle_1("joint_1", &_position[0], &_velocity[0], &_effort[0]);
    _state_interface.registerHandle(joint_state_handle_1);
    hardware_interface::JointStateHandle joint_state_handle_2("joint_2", &_position[1], &_velocity[1], &_effort[1]);
    _state_interface.registerHandle(joint_state_handle_2);
    
    registerInterface(&_state_interface);

    // set up the position control interface
    hardware_interface::JointHandle joint_handle_1(_state_interface.getHandle("joint_1"), &_command[0]);
    _command_interface.registerHandle(joint_handle_1);
    hardware_interface::JointHandle joint_handle_2(_state_interface.getHandle("joint_2"), &_command[1]);
    _command_interface.registerHandle(joint_handle_2);
    
    registerInterface(&_command_interface);
    
    // set up the controller manager
    _controller_manager = new controller_manager::ControllerManager(this, _nh);
    
    // set up ROS timer for robot updating
    _timer = _nh.createTimer(10.0, &MyRobot::update, this);
  }
  
  virtual ~MyRobot() {
    delete _controller_manager;
  }
  
  void update(const ros::TimerEvent& e) {
    ros::Time current_time = ros::Time::now();
    ros::Duration time_step = current_time - _last_update_time;
    double dt = time_step.toSec();
    read();
    _controller_manager->update(current_time, time_step);
    write(time_step.toSec());
    cout << "dt: " << dt << ", j1: " << _position[0] << ", j2: " << _position[1] << endl;
    _last_update_time = current_time;
  }
  
  double read() {
    _position[0] = _joint_1.getPosition();
    _position[1] = _joint_2.getPosition();
    _velocity[0] = _joint_1.getVelocity();
    _velocity[1] = _joint_2.getVelocity();
    _effort[0] = _joint_1.getEffort();
    _effort[1] = _joint_2.getEffort();
  }
  
  double write(double dt) {
    _joint_1.setCommand(_command[0]);
    _joint_2.setCommand(_command[1]);
    
    /* this is required since we are running joint numerical simulation */
    _joint_1.update(dt);
    _joint_2.update(dt);
  }

private:
  ros::NodeHandle& _nh;
  
  SimpleJoint _joint_1, _joint_2;
  double _position[2], _velocity[2], _effort[2], _command[2];
  
  hardware_interface::JointStateInterface _state_interface;
  hardware_interface::EffortJointInterface _command_interface;
  controller_manager::ControllerManager* _controller_manager;
  
  ros::Timer _timer;
  ros::Time _last_update_time;
};


int main(int argc, char* argv[]) {
  ros::init(argc, argv, "my_robot");
  ros::NodeHandle nh("~");
  
  MyRobot my_robot(nh);
  
  ros::MultiThreadedSpinner spinner(0);
  spinner.spin();
  
  return 0;
}
