// C++
#include <mutex>

// ROS
#include <std_msgs/Int8.h>
#include <geometry_msgs/TwistStamped.h>

// moveit_servo
#include <moveit_servo/servo.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/make_shared_from_pool.h>

// teaching_interface_ros
#include <teaching_interface_ros/TeachingCommand.h>


static const std::string LOGNAME = "servo_node";


// Class for monitoring status of moveit_servo
class StatusMonitor
{
public:
  StatusMonitor(ros::NodeHandle& nh, const std::string& topic)
  {
    sub_ = nh.subscribe(topic, 1, &StatusMonitor::statusCB, this);
  }

private:
  void statusCB(const std_msgs::Int8ConstPtr& msg)
  {
    moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
    if (latest_status != status_)
    {
      status_ = latest_status;
      const auto& status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
      ROS_INFO_STREAM_NAMED(LOGNAME, "Servo status: " << status_str);
    }
  }
  moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
  ros::Subscriber sub_;
};


// Class for passing commands from the teaching interface
class InterfaceLink
{
public:
  InterfaceLink(ros::NodeHandle& nh, moveit_servo::Servo& servo) : servo_(servo)
  {
    // Parameters
    nh.getParam("publish_rate", pub_rate_);
    loop_period_ = 1.0 / double(pub_rate_);
    nh.getParam("acceleration_limit_trans", acc_limit_trans_);
    nh.getParam("acceleration_limit_rot", acc_limit_rot_);

    // Subscribers and publishers
    sub_ = nh.subscribe("teaching_command", 1, &InterfaceLink::dataCB, this);
    twist_stamped_pub_ = nh.advertise<geometry_msgs::TwistStamped>(servo_.getParameters().cartesian_command_in_topic, 1);
  }

  void run()
  {
    // Loop and stream commands
    ros::Rate cmd_rate(pub_rate_);
    while (ros::ok())
    {
      // Make a Cartesian velocity message
      auto twist_msg = moveit::util::make_shared_from_pool<geometry_msgs::TwistStamped>();

      // Copy target values
      {
        const std::lock_guard<std::mutex> lock(mutex_);
        twist_msg->twist = twist_command_.twist;
        twist_msg->header.frame_id = twist_command_.header.frame_id;
      }

      // Limit acceleration
      twist_msg->twist.linear.x = limit_acceleration(twist_sent_.twist.linear.x, twist_msg->twist.linear.x, acc_limit_trans_);
      twist_msg->twist.linear.y = limit_acceleration(twist_sent_.twist.linear.y, twist_msg->twist.linear.y, acc_limit_trans_);
      twist_msg->twist.linear.z = limit_acceleration(twist_sent_.twist.linear.z, twist_msg->twist.linear.z, acc_limit_trans_);
      twist_msg->twist.angular.x = limit_acceleration(twist_sent_.twist.angular.x, twist_msg->twist.angular.x, acc_limit_rot_);
      twist_msg->twist.angular.y = limit_acceleration(twist_sent_.twist.angular.y, twist_msg->twist.angular.y, acc_limit_rot_);
      twist_msg->twist.angular.z = limit_acceleration(twist_sent_.twist.angular.z, twist_msg->twist.angular.z, acc_limit_rot_);

      // Send the message
      twist_msg->header.stamp = ros::Time::now();
      twist_stamped_pub_.publish(twist_msg);

      // Save sent message
      twist_sent_ = *twist_msg;

      cmd_rate.sleep();
    }
  }

private:
  void dataCB(const teaching_interface_ros::TeachingCommand& msg)
  {
    const std::lock_guard<std::mutex> lock(mutex_);

    // Copy message
    twist_command_.twist = msg.twist;

    // Set reference
    if (msg.reference == "base") {
      twist_command_.header.frame_id = servo_.getParameters().planning_frame;
    } else if (msg.reference == "ee") {
      twist_command_.header.frame_id = servo_.getParameters().ee_frame_name;
    } else {
      ROS_WARN_STREAM_NAMED(LOGNAME, "Unrecognized reference frame: " << msg.reference);
    }
  }

  double limit_acceleration(double current_vel, double target_vel, double limit)
  {
    const double delta_vel = target_vel - current_vel;
    const double delta_acc = delta_vel / loop_period_;
    if (fabs(delta_acc) < limit){
      return target_vel;
    }
    const double scale = fabs(limit / delta_acc);
    return current_vel + (delta_vel * scale);
  }

  int pub_rate_;
  double loop_period_;
  double acc_limit_trans_;
  double acc_limit_rot_;
  geometry_msgs::TwistStamped twist_command_;
  geometry_msgs::TwistStamped twist_sent_;
  ros::Subscriber sub_;
  ros::Publisher twist_stamped_pub_;
  moveit_servo::Servo& servo_;
  mutable std::mutex mutex_;
};


// Instantiate the C++ servo node interface.
int main(int argc, char** argv)
{
  ros::init(argc, argv, LOGNAME);
  ros::NodeHandle nh("~");
  ros::AsyncSpinner spinner(8);
  spinner.start();

  // Load the planning scene monitor
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
  planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
  if (!planning_scene_monitor->getPlanningScene())
  {
    ROS_ERROR_STREAM_NAMED(LOGNAME, "Error in setting up the PlanningSceneMonitor.");
    exit(EXIT_FAILURE);
  }

  planning_scene_monitor->startSceneMonitor();
  planning_scene_monitor->startWorldGeometryMonitor(
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
      planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
      false /* skip octomap monitor */);
  planning_scene_monitor->startStateMonitor();

  // Run the servo node C++ interface in a new timer to ensure a constant outgoing message rate.
  moveit_servo::Servo servo(nh, planning_scene_monitor);
  servo.start();

  // Subscribe to servo status (and log it when it changes)
  StatusMonitor status_monitor(nh, servo.getParameters().status_topic);

  // Create Interface Link
  InterfaceLink interface_link(nh, servo);

  // Run loop
  interface_link.run();

  // Wait for ros to shutdown
  // ros::waitForShutdown();

  // Stop the servo server
  servo.setPaused(true);

  return 0;
}
