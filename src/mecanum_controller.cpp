#include <cmath>

#include <tf/transform_datatypes.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf/urdfdom_compatibility.h>
#include <boost/assign.hpp>
#include <mecanum_controller/mecanum_controller.h>

#include <exception>


namespace mecanum_controller{

  MecanumController::MecanumController()
  {
  }

  bool MecanumController::init(hardware_interface::VelocityJointInterface* hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle &controller_nh)
  {
    const std::string complete_ns = controller_nh.getNamespace();
    std::size_t id = complete_ns.find_last_of("/");
    name_ = complete_ns.substr(id + 1);

    ROS_ERROR("...init");

  try {
      front_right_joint_ = hw->getHandle("wheel_front_right_joint");
      front_left_joint_ = hw->getHandle("wheel_front_left_joint");
      rear_right_joint_ = hw->getHandle("wheel_rear_right_joint");
      rear_left_joint_ = hw->getHandle("wheel_rear_left_joint");
    }
    catch (std::exception& e)
    {
      throw;
    }

    // Odometry related:

    // Twist command related:

    // Get the joint object to use in the realtime loop

    // Initialize dynamic parameters

    // Initialize dynamic_reconfigure server

    return true;
  }

  void MecanumController::update(const ros::Time& time, const ros::Duration& period)
  {
    ROS_ERROR("...update");
    front_right_joint_.setCommand(1.0);
    front_left_joint_.setCommand(2.0);
    rear_right_joint_.setCommand(3.0);
    rear_left_joint_.setCommand(4.0);
    // update parameter from dynamic reconf

    // Apply (possibly new) multipliers:

    // COMPUTE AND PUBLISH ODOMETRY

    // Publish odometry message

    // MOVE ROBOT
    // Retreive current velocity command and time step:

    // Brake if cmd_vel has timeout:

    // Limit velocities and accelerations:

    // Publish limited velocity:

    // Compute wheels velocities:

    // Set wheels velocities:
  }

  void MecanumController::starting(const ros::Time& time)
  {
    ROS_ERROR("...starting");
    brake();

    // Register starting time used to keep fixed rate
  }

  void MecanumController::stopping(const ros::Time& /*time*/)
  {
    ROS_ERROR("...stopping");
    brake();
  }

  void MecanumController::brake()
  {
    ROS_ERROR("...brake");
  }

  void MecanumController::cmdVelCallback(const geometry_msgs::Twist& command)
  {
    ROS_ERROR("...cmdVelCallback");
    if (isRunning())
    {
      // check that we don't have multiple publishers on the command topic
    }
    else
    {
      ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
    }
  }

} // namespace mecanum_controller
