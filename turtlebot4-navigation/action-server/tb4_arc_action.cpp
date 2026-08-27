#include <functional>
#include <memory>
#include <thread>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "irobot_create_msgs/action/drive_arc.hpp"
#include "nav_msgs/msg/odometry.hpp"

class TB4ArcActionServer : public rclcpp::Node
{
public:
  using Drive_Arc= irobot_create_msgs::action::DriveArc;

  explicit TB4ArcActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  :Node("tb4_arc_action_server", options)
  {
    // Initialise command velocity publisher shared pointer
    this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", rclcpp::SystemDefaultsQoS());
    using namespace std::placeholders;

    // Initialise the odometry subscriber shared pointer on the /odom topic, and bind callback function "odom_callback"
    // Because I am using this with the simulation created in Tutorial 2, I have had to subscribe to /diffdrive_controller/odom rather than just /odom
    this->odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>("/diffdrive_controller/odom", rclcpp::SensorDataQoS(), std::bind(&TB4ArcActionServer::odom_callback, this, std::placeholders::_1)
    );

    // Initialsie the drive arc action server with name as "drive_arc_prac2", and bind call back functions for handling of accepting a goal, cancelling a action, and process the accepted goal
    this->action_server_ = rclcpp_action::create_server<Drive_Arc>(this, "drive_arc_prac2", std::bind(&TB4ArcActionServer::handle_goal, this, _1, _2), std::bind(&TB4ArcActionServer::handle_cancel, this, _1), std::bind(&TB4ArcActionServer::handle_accepted, this, _1));

  }
private:
  // Define drive arc server shared pointer
  rclcpp_action::Server<Drive_Arc>::SharedPtr action_server_;
  // Define command velocity publisher shared pointer
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  // Define odometry subscriber shared pointer
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;

  // odometry pointer
  nav_msgs::msg::Odometry::SharedPtr odom_;
  // odometry subscriber callback
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

  // Callback function for handling goals
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const irobot_create_msgs::action::DriveArc::Goal> goal
  );

  // Callback function for handling cancellation:
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle);

  void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle);

  // Action processing and update
  void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle);
};


void TB4ArcActionServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
  /*TODO TASK - MILESTONE #3.1
    save the odom_msg to the class member variable odom_   
  */
  odom_ = odom_msg;
}

/*TODO TASK - MILESTONE #4.1
  complete the  call back function of "TB4ArcActionServer::handle_accepted" that handling the accepted goal 
*/
void TB4ArcActionServer::handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle)
{
  using namespace std::placeholders;
  std::thread{std::bind(&TB4ArcActionServer::execute, this, _1), goal_handle}.detach();
}

/* TODO TASK - MILESTONE #4.2
  complete the  call back function of "TB4ArcActionServer::handle_cancel" that cancel the goal 
  */
rclcpp_action::CancelResponse TB4ArcActionServer::handle_cancel(
  const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

/*TODO TASK - MILESTONE #4.3
  complete the  call back function of "TB4ArcActionServer::handle_goal" that accept goal, 
  you should also print the goal details in the terminal 
*/
rclcpp_action::GoalResponse TB4ArcActionServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const irobot_create_msgs::action::DriveArc::Goal> goal
)
{
  if (goal->radius < 0.0)
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid goal: radius must not be negative.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  else if (goal->max_translation_speed < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid goal: max translation speed must not be negative.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  else if (goal->angle < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid goal: angle must not be negative.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  else if (goal->translate_direction >= 1 && goal->translate_direction <= -1)
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid goal: translation direction must be 1 or more OR -1 or less");
    return rclcpp_action::GoalResponse::REJECT;
  }

  RCLCPP_INFO(this->get_logger(), "Received goal request %d direction, where positive is clockwise, travel angle at %f radians at a radius of %f m and maximum speed at %f m/s", goal->translate_direction, goal->angle, goal->radius, goal->max_translation_speed);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

/* TODO TASKS - MILESTONE #5.1 ~ #5.3
  complete the  thread function "execute" to proccess the goal in the action request
*/
void TB4ArcActionServer::execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<irobot_create_msgs::action::DriveArc>> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Executing goal");
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Drive_Arc::Feedback>();
  
  auto & remaining_angle_travel = feedback->remaining_angle_travel;
  auto result = std::make_shared<Drive_Arc::Result>();
  
  double distance = double(goal->radius*goal->angle);
  
  // Check whether it is to be clockwise or anticlockwise rotation
  // The direction_constant variable will be used to flip the sign of angular velocity z
  
  int direction_constant = 1;
  
  if (goal->translate_direction > 0)
  {
    direction_constant = 1;
  }
  else
  {
    direction_constant = -1;
  }
  
  double angular_velocity = double(direction_constant*goal->max_translation_speed/goal->radius);
  
  geometry_msgs::msg::Twist cmd_vel;
  cmd_vel.linear.set__x(goal->max_translation_speed);
  cmd_vel.angular.set__z(angular_velocity);
  
  int pub_freq = 100;
  rclcpp::Rate loop_rate(pub_freq);
  
  int count = int(pub_freq*distance/goal->max_translation_speed);
  
  geometry_msgs::msg::PoseStamped pose_stamped;
  
  // Below is checking the validation
  // Canceling or aborting the goals here does not work and causes server to crash (abort)
  // According to the error message this is because it is already executing and then canceling
  /*
  if (goal->radius <= 0.0)
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid goal: radius must be greater than zero.");
    //result->set__pose(pose_stamped);
    //goal_handle->canceled(result);
  }
  else if (goal->max_translation_speed <= 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid goal: max translation speed must be greater than zero.");
  }
  else if (goal->angle <= 0)
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid goal: angle must be greater than zero.");
  }
  else if (goal->translate_direction >= 1 && goal->translate_direction <= -1)
  {
    RCLCPP_ERROR(this->get_logger(), "Invalid goal: translation direction must be 1 or more OR -1 or less");
  }
  */
  
  for (int i = 0; (i<count) && rclcpp::ok(); ++i)
  {
    pose_stamped.header = odom_->header;
    pose_stamped.pose = odom_->pose.pose;
    
    if (goal_handle->is_canceling())
    {
      result->set__pose(pose_stamped);
      goal_handle->canceled(result);
      RCLCPP_INFO(this->get_logger(), "Goal canceled");
      return;
    }
    
    remaining_angle_travel = goal->angle - angular_velocity*i*direction_constant/pub_freq;
    // Publish the command velocity
    cmd_vel_publisher_->publish(cmd_vel);
    // Publish feedback
    goal_handle->publish_feedback(feedback);
    loop_rate.sleep();
  }
  
  // Check if goal is done
  if (rclcpp::ok())
  {
    result->set__pose(pose_stamped);
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
  }
}

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<TB4ArcActionServer>());
	rclcpp::shutdown();
	return 0;
}
