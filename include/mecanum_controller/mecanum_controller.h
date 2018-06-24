#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.hpp>
#include <dynamic_reconfigure/server.h>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/tfMessage.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <mecanum_controller/odometry.h>
#include <mecanum_controller/speed_limiter.h>
// #include <mecanum_controller/MecanumControllerConfig.h>

#include <urdf_geometry_parser/urdf_geometry_parser.h>

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

    bool getWheelNames(ros::NodeHandle& controller_nh,
                                const std::string& wheel_param,
                                std::vector<std::string>& wheel_names);
    /**
     * \brief Updates controller, i.e. computes the odometry and sets the new velocity commands
     * \param time   Current time
     * \param period Time since the last called to update
     */
    void update(const ros::Time& time, const ros::Duration& period);

    void updateCommand(const ros::Time& time, const ros::Duration& period);

    void updateOdometry(const ros::Time& time);

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

    double track_;
    double wheel_base_;
    double wheel_radius_;

    std::string base_frame_id_;

    // Hardware handles
    hardware_interface::JointHandle front_right_joint_;
    hardware_interface::JointHandle front_left_joint_;
    hardware_interface::JointHandle rear_right_joint_;
    hardware_interface::JointHandle rear_left_joint_;

    /// Velocity command related:
    struct Commands
    {
      double lin_x;
      double lin_y;
      double ang;
      ros::Time stamp;

      Commands() : lin_x(0.0), lin_y(0.0), ang(0.0), stamp(0.0) {}
    };
    realtime_tools::RealtimeBuffer<Commands> command_;
    Commands command_struct_;

    ros::Subscriber sub_command_;
    boost::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped> > cmd_vel_pub_;

    /// Timeout to consider cmd_vel commands old:
    double cmd_vel_timeout_;

    /// Whether to allow multiple publishers on cmd_vel topic or not:
    bool allow_multiple_cmd_vel_publishers_;

    /// Odometry related:
    boost::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry> > odom_pub_;
    boost::shared_ptr<realtime_tools::RealtimePublisher<tf::tfMessage> > tf_odom_pub_;

    Odometry odometry_;

    ros::Duration publish_period_;
    ros::Time last_state_publish_time_;

    bool open_loop_;
    bool enable_odom_tf_;

    /// Speed limiters:
    Commands last1_cmd_;
    Commands last0_cmd_;
    SpeedLimiter limiter_lin_x;
    SpeedLimiter limiter_lin_y;
    SpeedLimiter limiter_ang_;

    /// Publish limited velocity:
    bool publish_cmd_;

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

    void setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh);
  };

  PLUGINLIB_EXPORT_CLASS(mecanum_controller::MecanumController, controller_interface::ControllerBase);
} // namespace mecanum_controller
