#include <cmath>

#include <tf/transform_datatypes.h>
#include <urdf_parser/urdf_parser.h>
#include <urdf/urdfdom_compatibility.h>
#include <boost/assign.hpp>
#include <mecanum_controller/mecanum_controller.h>

#include <exception>


namespace mecanum_controller{

  MecanumController::MecanumController()
    :
    track_(0.0),
    wheel_base_(0.0),
    wheel_radius_(0.0),
    base_frame_id_("base_footprint"),
    cmd_vel_timeout_(0.5),
    allow_multiple_cmd_vel_publishers_(true),
    publish_cmd_(true),
    open_loop_(true),
    enable_odom_tf_(true)
  {
  }

  bool MecanumController::init(hardware_interface::VelocityJointInterface* hw,
            ros::NodeHandle& root_nh,
            ros::NodeHandle &controller_nh)
  {
    const std::string complete_ns = controller_nh.getNamespace();
    std::size_t id = complete_ns.find_last_of("/");
    name_ = complete_ns.substr(id + 1);

    ROS_DEBUG("...init");

    // Get joint names from the parameter server
    std::vector<std::string> front_wheel_names, rear_wheel_names;
    if (!getWheelNames(controller_nh, "front_wheel", front_wheel_names) ||
        !getWheelNames(controller_nh, "rear_wheel", rear_wheel_names))
    {
      return false;
    }

    if (front_wheel_names.size() != rear_wheel_names.size())
    {
      ROS_ERROR_STREAM_NAMED(name_,
          "#front wheels (" << front_wheel_names.size() << ") != " <<
          "#rear wheels (" << rear_wheel_names.size() << ").");
      return false;
    }
    else if (front_wheel_names.size() != 2)
    {
      ROS_ERROR_STREAM_NAMED(name_,
          "#two wheels by axle (left and right) is needed; now : "<<front_wheel_names.size()<<" .");
      return false;
    }

    // Get the joint objects to use in the realtime loop
    try {
      front_left_joint_ = hw->getHandle(front_wheel_names[0]);
      front_right_joint_ = hw->getHandle(front_wheel_names[1]);
      rear_left_joint_ = hw->getHandle(rear_wheel_names[0]);
      rear_right_joint_ = hw->getHandle(rear_wheel_names[1]);
    }
    catch (std::exception& e)
    {
      throw;
    }

    // If either parameter is not available, we need to look up the value in the URDF
    bool lookup_track = !controller_nh.getParam("track", track_);
    bool lookup_wheel_radius = !controller_nh.getParam("wheel_radius", wheel_radius_);
    bool lookup_wheel_base = !controller_nh.getParam("wheel_base", wheel_base_);

    urdf_geometry_parser::UrdfGeometryParser uvk(root_nh, base_frame_id_);

    if(lookup_track)
      if(!uvk.getDistanceBetweenJoints(front_wheel_names[0], front_wheel_names[1], track_))
        return false;
      else
        controller_nh.setParam("track",track_);

    if(lookup_wheel_base)
      if(!uvk.getDistanceBetweenJoints(front_wheel_names[0], rear_wheel_names[0], wheel_base_))
        return false;
      else
        controller_nh.setParam("wheel_base",wheel_base_);

    if(lookup_wheel_radius)
      if(!uvk.getJointRadius(front_wheel_names[0], wheel_radius_))
        return false;
      else
        controller_nh.setParam("wheel_radius",wheel_radius_);

    ROS_DEBUG("Track: %f, Wheel base: %f, Radius: %f", track_, wheel_base_, wheel_radius_);

    // Odometry related:

    double publish_rate;
    controller_nh.param("publish_rate", publish_rate, 50.0);
    ROS_INFO_STREAM_NAMED(name_, "Controller state will be published at "
                          << publish_rate << "Hz.");
    publish_period_ = ros::Duration(1.0 / publish_rate);

    controller_nh.param("open_loop", open_loop_, open_loop_);

    int velocity_rolling_window_size = 10;
    odometry_.setVelocityRollingWindowSize(velocity_rolling_window_size);
    odometry_.setWheelParams(track_, wheel_base_, wheel_radius_);
    setOdomPubFields(root_nh, controller_nh);

    // Twist command related:
    controller_nh.param("cmd_vel_timeout", cmd_vel_timeout_, cmd_vel_timeout_);
    ROS_INFO_STREAM_NAMED(name_, "Velocity commands will be considered old if they are older than "
                          << cmd_vel_timeout_ << "s.");

    controller_nh.param("allow_multiple_cmd_vel_publishers", allow_multiple_cmd_vel_publishers_, allow_multiple_cmd_vel_publishers_);
    ROS_INFO_STREAM_NAMED(name_, "Allow mutiple cmd_vel publishers is "
                          << (allow_multiple_cmd_vel_publishers_?"enabled":"disabled"));


    // Velocity and acceleration limits:
    controller_nh.param("linear/x/has_velocity_limits"    , limiter_lin_x.has_velocity_limits    , limiter_lin_x.has_velocity_limits    );
    controller_nh.param("linear/x/has_acceleration_limits", limiter_lin_x.has_acceleration_limits, limiter_lin_x.has_acceleration_limits);
    controller_nh.param("linear/x/has_jerk_limits"        , limiter_lin_x.has_jerk_limits        , limiter_lin_x.has_jerk_limits        );
    controller_nh.param("linear/x/max_velocity"           , limiter_lin_x.max_velocity           ,  limiter_lin_x.max_velocity          );
    controller_nh.param("linear/x/min_velocity"           , limiter_lin_x.min_velocity           , -limiter_lin_x.max_velocity          );
    controller_nh.param("linear/x/max_acceleration"       , limiter_lin_x.max_acceleration       ,  limiter_lin_x.max_acceleration      );
    controller_nh.param("linear/x/min_acceleration"       , limiter_lin_x.min_acceleration       , -limiter_lin_x.max_acceleration      );
    controller_nh.param("linear/x/max_jerk"               , limiter_lin_x.max_jerk               ,  limiter_lin_x.max_jerk              );
    controller_nh.param("linear/x/min_jerk"               , limiter_lin_x.min_jerk               , -limiter_lin_x.max_jerk              );

    controller_nh.param("linear/y/has_velocity_limits"    , limiter_lin_y.has_velocity_limits    , limiter_lin_y.has_velocity_limits    );
    controller_nh.param("linear/y/has_acceleration_limits", limiter_lin_y.has_acceleration_limits, limiter_lin_y.has_acceleration_limits);
    controller_nh.param("linear/y/has_jerk_limits"        , limiter_lin_y.has_jerk_limits        , limiter_lin_y.has_jerk_limits        );
    controller_nh.param("linear/y/max_velocity"           , limiter_lin_y.max_velocity           ,  limiter_lin_y.max_velocity          );
    controller_nh.param("linear/y/min_velocity"           , limiter_lin_y.min_velocity           , -limiter_lin_y.max_velocity          );
    controller_nh.param("linear/y/max_acceleration"       , limiter_lin_y.max_acceleration       ,  limiter_lin_y.max_acceleration      );
    controller_nh.param("linear/y/min_acceleration"       , limiter_lin_y.min_acceleration       , -limiter_lin_y.max_acceleration      );
    controller_nh.param("linear/y/max_jerk"               , limiter_lin_y.max_jerk               ,  limiter_lin_y.max_jerk              );
    controller_nh.param("linear/y/min_jerk"               , limiter_lin_y.min_jerk               , -limiter_lin_y.max_jerk              );

    controller_nh.param("angular/z/has_velocity_limits"    , limiter_ang_.has_velocity_limits    , limiter_ang_.has_velocity_limits    );
    controller_nh.param("angular/z/has_acceleration_limits", limiter_ang_.has_acceleration_limits, limiter_ang_.has_acceleration_limits);
    controller_nh.param("angular/z/has_jerk_limits"        , limiter_ang_.has_jerk_limits        , limiter_ang_.has_jerk_limits        );
    controller_nh.param("angular/z/max_velocity"           , limiter_ang_.max_velocity           ,  limiter_ang_.max_velocity          );
    controller_nh.param("angular/z/min_velocity"           , limiter_ang_.min_velocity           , -limiter_ang_.max_velocity          );
    controller_nh.param("angular/z/max_acceleration"       , limiter_ang_.max_acceleration       ,  limiter_ang_.max_acceleration      );
    controller_nh.param("angular/z/min_acceleration"       , limiter_ang_.min_acceleration       , -limiter_ang_.max_acceleration      );
    controller_nh.param("angular/z/max_jerk"               , limiter_ang_.max_jerk               ,  limiter_ang_.max_jerk              );
    controller_nh.param("angular/z/min_jerk"               , limiter_ang_.min_jerk               , -limiter_ang_.max_jerk              );


    // Subscribe to command velocity
    sub_command_ = controller_nh.subscribe("cmd_vel", 1, &MecanumController::cmdVelCallback, this);

    // Publish executed command velocity
    if (publish_cmd_)
    {
      cmd_vel_pub_.reset(new realtime_tools::RealtimePublisher<geometry_msgs::TwistStamped>(controller_nh, "cmd_vel_out", 100));
    }

    // Initialize dynamic parameters

    // Initialize dynamic_reconfigure server

    return true;
  }

  bool MecanumController::getWheelNames(ros::NodeHandle& controller_nh,
                              const std::string& wheel_param,
                              std::vector<std::string>& wheel_names)
  {
      XmlRpc::XmlRpcValue wheel_list;
      if (!controller_nh.getParam(wheel_param, wheel_list))
      {
        ROS_ERROR_STREAM_NAMED(name_,
            "Couldn't retrieve wheel param '" << wheel_param << "'.");
        return false;
      }

      if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        if (wheel_list.size() == 0)
        {
          ROS_ERROR_STREAM_NAMED(name_,
              "Wheel param '" << wheel_param << "' is an empty list");
          return false;
        }

        for (int i = 0; i < wheel_list.size(); ++i)
        {
          if (wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString)
          {
            ROS_ERROR_STREAM_NAMED(name_,
                "Wheel param '" << wheel_param << "' #" << i <<
                " isn't a string.");
            return false;
          }
        }

        wheel_names.resize(wheel_list.size());
        for (int i = 0; i < wheel_list.size(); ++i)
        {
          wheel_names[i] = static_cast<std::string>(wheel_list[i]);
          //ROS_INFO_STREAM("wheel name "<<i<<" " << wheel_names[i]);
        }
      }
      else if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeString)
      {
        wheel_names.push_back(wheel_list);
      }
      else
      {
        ROS_ERROR_STREAM_NAMED(name_,
            "Wheel param '" << wheel_param <<
            "' is neither a list of strings nor a string.");
        return false;
      }
      return true;
  }

  void MecanumController::update(const ros::Time& time, const ros::Duration& period)
  {
    // update parameter from dynamic reconf
    updateCommand(time, period);
    updateOdometry(time);

  }

  void MecanumController::updateCommand(const ros::Time& time, const ros::Duration& period) {
    // Apply (possibly new) multipliers:


    // MOVE ROBOT
    // Retreive current velocity command and time step:
    Commands curr_cmd = *(command_.readFromRT());
    const double dt = (time - curr_cmd.stamp).toSec();

    // Brake if cmd_vel has timeout:
    if (dt > cmd_vel_timeout_)
    {
      curr_cmd.lin_x = 0.0;
      curr_cmd.lin_y = 0.0;
      curr_cmd.ang = 0.0;
    }

    // Limit velocities and accelerations:
    const double cmd_dt(period.toSec());

    limiter_lin_x.limit(curr_cmd.lin_x, last0_cmd_.lin_x, last1_cmd_.lin_x, cmd_dt);
    limiter_lin_y.limit(curr_cmd.lin_y, last0_cmd_.lin_y, last1_cmd_.lin_y, cmd_dt);
    limiter_ang_.limit(curr_cmd.ang, last0_cmd_.ang, last1_cmd_.ang, cmd_dt);

    last1_cmd_ = last0_cmd_;
    last0_cmd_ = curr_cmd;

    // Publish limited velocity:
    if (publish_cmd_ && cmd_vel_pub_ && cmd_vel_pub_->trylock())
    {
      cmd_vel_pub_->msg_.header.stamp = time;
      cmd_vel_pub_->msg_.twist.linear.x = curr_cmd.lin_x;
      cmd_vel_pub_->msg_.twist.linear.y = curr_cmd.lin_y;
      cmd_vel_pub_->msg_.twist.angular.z = curr_cmd.ang;
      cmd_vel_pub_->unlockAndPublish();
    }

    // Compute wheels velocities:
    // const double distance_left_to_right_wheel = 0.4;
    const double wheel_separation_width_ = wheel_base_/2;

    // const double distance_front_to_rear_wheel = 0.4;
    const double wheel_separation_length_ = track_/2;

    // const double wheel_radius_ = 0.05;

    const double wheel_front_left_vel =
      (1/wheel_radius_)*(curr_cmd.lin_x - curr_cmd.lin_y -
      (wheel_separation_width_ + wheel_separation_length_)*curr_cmd.ang);

    const double wheel_front_right_vel =
      (1/wheel_radius_)*(curr_cmd.lin_x + curr_cmd.lin_y +
      (wheel_separation_width_ + wheel_separation_length_)*curr_cmd.ang);

    const double wheel_rear_left_vel =
      (1/wheel_radius_)*(curr_cmd.lin_x + curr_cmd.lin_y -
      (wheel_separation_width_ + wheel_separation_length_)*curr_cmd.ang);

    const double wheel_rear_right_vel =
      (1/wheel_radius_)*(curr_cmd.lin_x - curr_cmd.lin_y +
      (wheel_separation_width_ + wheel_separation_length_)*curr_cmd.ang);

    // Set wheels velocities:
    front_right_joint_.setCommand(wheel_front_right_vel);
    front_left_joint_.setCommand(wheel_front_left_vel);
    rear_right_joint_.setCommand(wheel_rear_right_vel);
    rear_left_joint_.setCommand(wheel_rear_left_vel);
  }

  void MecanumController::updateOdometry(const ros::Time& time) {
    // COMPUTE AND PUBLISH ODOMETRY
    if(open_loop_) {
      odometry_.updateOpenLoop(last0_cmd_.lin_x, last0_cmd_.lin_y, last0_cmd_.ang, time);
    } else {
      odometry_.update(
        front_left_joint_.getVelocity(), front_right_joint_.getVelocity(),
        rear_left_joint_.getVelocity(), rear_right_joint_.getVelocity(),
        time
      );
    }

    ROS_DEBUG("Odom: x: %f, y: %f, heading: %f", odometry_.getX(), odometry_.getY(), odometry_.getHeading());

    // Publish odometry message
    if (last_state_publish_time_ + publish_period_ < time)
    {
      last_state_publish_time_ += publish_period_;
      // Compute and store orientation info
      const geometry_msgs::Quaternion orientation(
            tf::createQuaternionMsgFromYaw(odometry_.getHeading()));

      // Populate odom message and publish
      if (odom_pub_->trylock())
      {
        odom_pub_->msg_.header.stamp = time;
        odom_pub_->msg_.pose.pose.position.x = odometry_.getX();
        odom_pub_->msg_.pose.pose.position.y = odometry_.getY();
        odom_pub_->msg_.pose.pose.orientation = orientation;
        odom_pub_->msg_.twist.twist.linear.x  = odometry_.getLinearX();
        odom_pub_->msg_.twist.twist.linear.y  = odometry_.getLinearY();
        odom_pub_->msg_.twist.twist.angular.z = odometry_.getAngular();
        odom_pub_->unlockAndPublish();
      }

      // Publish tf /odom frame
      if (enable_odom_tf_ && tf_odom_pub_->trylock())
      {
        geometry_msgs::TransformStamped& odom_frame = tf_odom_pub_->msg_.transforms[0];
        odom_frame.header.stamp = time;
        odom_frame.transform.translation.x = odometry_.getX();
        odom_frame.transform.translation.y = odometry_.getY();
        odom_frame.transform.rotation = orientation;
        tf_odom_pub_->unlockAndPublish();
      }
    }
  }

  void MecanumController::starting(const ros::Time& time)
  {
    ROS_DEBUG("...starting");
    brake();

    // Register starting time used to keep fixed rate
    last_state_publish_time_ = time;

    odometry_.init(time);
  }

  void MecanumController::stopping(const ros::Time& /*time*/)
  {
    ROS_DEBUG("...stopping");
    brake();
  }

  void MecanumController::brake()
  {
    ROS_DEBUG("...brake");
    const double vel = 0.0;
    front_right_joint_.setCommand(vel);
    front_left_joint_.setCommand(vel);
    rear_right_joint_.setCommand(vel);
    rear_left_joint_.setCommand(vel);
  }

  void MecanumController::cmdVelCallback(const geometry_msgs::Twist& command)
  {
    if (isRunning())
    {
      // check that we don't have multiple publishers on the command topic
      if (!allow_multiple_cmd_vel_publishers_ && sub_command_.getNumPublishers() > 1)
      {
        ROS_ERROR_STREAM_THROTTLE_NAMED(1.0, name_, "Detected " << sub_command_.getNumPublishers()
            << " publishers. Only 1 publisher is allowed. Going to brake.");
        brake();
        return;
      }

      command_struct_.ang   = command.angular.z;
      command_struct_.lin_x   = command.linear.x;
      command_struct_.lin_y   = command.linear.y;
      command_struct_.stamp = ros::Time::now();
      command_.writeFromNonRT (command_struct_);
      ROS_DEBUG_STREAM_NAMED(name_,
                             "Added values to command. "
                             << "Ang: "   << command_struct_.ang << ", "
                             << "Lin x: "   << command_struct_.lin_x << ", "
                             << "Lin y: "   << command_struct_.lin_y << ", "
                             << "Stamp: " << command_struct_.stamp);
    }
    else
    {
      ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
    }
  }

  void MecanumController::setOdomPubFields(ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh)
  {
    // Get and check params for covariances
    XmlRpc::XmlRpcValue pose_cov_list;
    controller_nh.getParam("pose_covariance_diagonal", pose_cov_list);
    ROS_ASSERT(pose_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(pose_cov_list.size() == 6);
    for (int i = 0; i < pose_cov_list.size(); ++i)
      ROS_ASSERT(pose_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    XmlRpc::XmlRpcValue twist_cov_list;
    controller_nh.getParam("twist_covariance_diagonal", twist_cov_list);
    ROS_ASSERT(twist_cov_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
    ROS_ASSERT(twist_cov_list.size() == 6);
    for (int i = 0; i < twist_cov_list.size(); ++i)
      ROS_ASSERT(twist_cov_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);

    // Setup odometry realtime publisher + odom message constant fields
    odom_pub_.reset(new realtime_tools::RealtimePublisher<nav_msgs::Odometry>(controller_nh, "odom", 100));
    odom_pub_->msg_.header.frame_id = "odom";
    odom_pub_->msg_.child_frame_id = base_frame_id_;
    odom_pub_->msg_.pose.pose.position.z = 0;
    odom_pub_->msg_.pose.covariance = boost::assign::list_of
        (static_cast<double>(pose_cov_list[0])) (0)  (0)  (0)  (0)  (0)
        (0)  (static_cast<double>(pose_cov_list[1])) (0)  (0)  (0)  (0)
        (0)  (0)  (static_cast<double>(pose_cov_list[2])) (0)  (0)  (0)
        (0)  (0)  (0)  (static_cast<double>(pose_cov_list[3])) (0)  (0)
        (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[4])) (0)
        (0)  (0)  (0)  (0)  (0)  (static_cast<double>(pose_cov_list[5]));
    odom_pub_->msg_.twist.twist.linear.y  = 0;
    odom_pub_->msg_.twist.twist.linear.z  = 0;
    odom_pub_->msg_.twist.twist.angular.x = 0;
    odom_pub_->msg_.twist.twist.angular.y = 0;
    odom_pub_->msg_.twist.covariance = boost::assign::list_of
        (static_cast<double>(twist_cov_list[0])) (0)  (0)  (0)  (0)  (0)
        (0)  (static_cast<double>(twist_cov_list[1])) (0)  (0)  (0)  (0)
        (0)  (0)  (static_cast<double>(twist_cov_list[2])) (0)  (0)  (0)
        (0)  (0)  (0)  (static_cast<double>(twist_cov_list[3])) (0)  (0)
        (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[4])) (0)
        (0)  (0)  (0)  (0)  (0)  (static_cast<double>(twist_cov_list[5]));

    tf_odom_pub_.reset(new realtime_tools::RealtimePublisher<tf::tfMessage>(root_nh, "/tf", 100));
    tf_odom_pub_->msg_.transforms.resize(1);
    tf_odom_pub_->msg_.transforms[0].transform.translation.z = 0.0;
    tf_odom_pub_->msg_.transforms[0].child_frame_id = base_frame_id_;
    tf_odom_pub_->msg_.transforms[0].header.frame_id = "odom";
  }

} // namespace mecanum_controller
