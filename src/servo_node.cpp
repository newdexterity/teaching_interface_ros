#include <moveit_servo/servo.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <std_msgs/Int8.h>
#include <teaching_interface_ros/InterfaceData.h>

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
    sub_ = nh.subscribe("teaching_interface_data", 1, &InterfaceLink::dataCB, this);
    twist_stamped_pub_ = nh.advertise<geometry_msgs::TwistStamped>(servo_.getParameters().cartesian_command_in_topic, 1);
    reference_ = servo_.getParameters().robot_link_command_frame;
  }

private:
  void dataCB(const teaching_interface_ros::InterfaceData& msg)
  {
    // Change reference frame if required
    if (msg.reference != reference_) {
      reference_ = msg.reference;
      servo_.setPaused(true);
      servo_.changeRobotLinkCommandFrame(reference_);
      ROS_INFO_STREAM_NAMED(LOGNAME, "Reference frame changed to: " << reference_);
      servo_.setPaused(false);
      return;
    }

    // Make a Cartesian velocity message
    auto twist_msg = moveit::util::make_shared_from_pool<geometry_msgs::TwistStamped>();
    twist_msg->header.stamp = ros::Time::now();
    // twist_msg->header.frame_id = "base_link";
    twist_msg->header.frame_id = reference_;

    // Fill in the velocity commands
    twist_msg->twist.linear.x = msg.dial * msg.x_button * msg.direction;
    twist_msg->twist.linear.y = msg.dial * msg.y_button * msg.direction;
    twist_msg->twist.linear.z = msg.dial * msg.z_button * msg.direction;
    twist_msg->twist.angular.x = msg.dial * msg.roll_button * msg.direction;
    twist_msg->twist.angular.y = msg.dial * msg.pitch_button * msg.direction;
    twist_msg->twist.angular.z = msg.dial * msg.yaw_button * msg.direction;

    // Send the message
    twist_stamped_pub_.publish(twist_msg);
  }

  ros::Subscriber sub_;
  ros::Publisher twist_stamped_pub_;
  moveit_servo::Servo& servo_;
  std::string reference_;
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

  // Wait for ros to shutdown
  ros::waitForShutdown();

  // Stop the servo server
  servo.setPaused(true);

  return 0;
}
