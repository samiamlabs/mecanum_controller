#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <dynamic_reconfigure/server.h>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

// #include <mecanum_controller/odometry.h>
// #include <mecanum_controller/speed_limiter.h>
// #include <mecanum_controller/MecanumControllerConfig.h>

namespace mecanum_controller{

  /**
   * This class makes some assumptions on the model of the robot:
   *  - the rotation axes of wheels are collinear
   *  - the wheels are identical in radius
   * Additional assumptions to not duplicate information readily available in the URDF:
   *  - the wheels have the same parent frame
   *  - a wheel collision geometry is a cylinder or sphere in the urdf
   *  - a wheel joint frame center's vertical projection on the floor must lie within the contact patch
   */
  class MecanumController
      : public controller_interface::Controller<hardware_interface::VelocityJointInterface>
  {
  public:
    MecanumController();

    /**
     * \brief Initialize controller
     * \param hw            Velocity joint interface for the wheels
     * \param root_nh       Node handle at root namespace
     * \param controller_nh Node handle inside the controller namespace
     */
    bool init(hardware_interface::VelocityJointInterface* hw,
              ros::NodeHandle& root_nh,
              ros::NodeHandle &controller_nh);

    /**
     * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
     * \param time   Current time
     * \param period Time since the last called to update
     */
    void update(const ros::Time& time, const ros::Duration& period);

    /**
     * \brief Starts controller
     * \param time Current time
     */
    void starting(const ros::Time& time);

    /**
     * \brief Stops controller
     * \param time Current time
     */
    void stopping(const ros::Time& /*time*/);

  private:
    std::string name_;

    // Hardware handles
    hardware_interface::JointHandle front_right_joint_;
    hardware_interface::JointHandle front_left_joint_;
    hardware_interface::JointHandle rear_right_joint_;
    hardware_interface::JointHandle rear_left_joint_;

  private:
    /**
     * \brief Brakes the wheels, i.e. sets the velocity to 0
     */
    void brake();

    /**
     * \brief Velocity command callback
     * \param command Velocity command message (twist)
     */
    void cmdVelCallback(const geometry_msgs::Twist& command);
  };

  PLUGINLIB_EXPORT_CLASS(mecanum_controller::MecanumController, controller_interface::ControllerBase);
} // namespace mecanum_controller
