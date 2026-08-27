#include <chrono>  
#include <functional>
#include <memory>
#include <string>
#include <algorithm>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#define PI 3.14159265358
using rcl_interfaces::msg::ParameterType;


class WallFollower : public rclcpp::Node
{
public:
    WallFollower(): Node("wall_follower")
    {
        /*TODO TASK - MILESTONE # 4.1
            1. Declare all parameters used for configuring the "following distance", "following angle", and all control gains. Their default values should be given as well.
            2. Get all parameter values from the constructor, and save them to private class element variables.
            3. Print all parameter values here.
            4. Set the value of "following_angle_" after initialising all parameters
        */
        
        auto wall_side_desc = rcl_interfaces::msg::ParameterDescriptor{};
        wall_side_desc.description = "A positive value indicates that the wall will be on the left side of the robot, otherwise on the right";
        auto buffer_zone_desc = rcl_interfaces::msg::ParameterDescriptor{};
        buffer_zone_desc.description = "The distance that the robot is allowed to drift away from the desired following distance from the wall";
        
        // Declare parameters
        this->declare_parameter<double>("following_distance", 0.7);
        this->declare_parameter<int64_t>("wall_side", 1, wall_side_desc);
	this->declare_parameter<double>("buffer_zone", 0.4, buffer_zone_desc);
	this->declare_parameter<double>("forward_velocity", 0.4);
	this->declare_parameter<double>("angle_control_gain_1", 1.0);
	this->declare_parameter<double>("angle_control_gain_2", -0.05);
	//this->declare_parameter<double>("distance_control_gain", 0.5);
	
	// Get parameter values
	this->get_parameter("following_distance", following_distance_);
	this->get_parameter("wall_side", wall_side_);
	this->get_parameter("buffer_zone", buffer_zone_);
	this->get_parameter("forward_velocity", forward_velocity_);
	this->get_parameter("angle_control_gain_1", angle_control_gain_1_);
	this->get_parameter("angle_control_gain_2", angle_control_gain_2_);
	//this->get_parameter("distance_control_gain", distance_control_gain_);
	
	// Print parameter values
	RCLCPP_INFO(this->get_logger(), "following_distance: %.2f", following_distance_);
	RCLCPP_INFO(this->get_logger(), "wall_side: %ld", wall_side_);
	RCLCPP_INFO(this->get_logger(), "buffer_zone: %.2f", buffer_zone_);
	RCLCPP_INFO(this->get_logger(), "forward_velocity: %.2f", forward_velocity_);
	RCLCPP_INFO(this->get_logger(), "angle_control_gain_1: %.2f", angle_control_gain_1_);
	RCLCPP_INFO(this->get_logger(), "angle_control_gain_2: %.2f", angle_control_gain_2_);
	//RCLCPP_INFO(this->get_logger(), "distance_control_gain: %.2f", distance_control_gain_);
	
	// Set following angle depending on what side we want to follow the wall on
	if (wall_side_ > 0)
	    following_angle_ = PI/2;
	else
	    following_angle_ = -PI/2;

        /* TODO TASK - MILESTONE #4.3
            Initialise dynamic parameter handler by the rclcpp node method "add_on_set_parameters_callback"
        */
        
        dyn_params_handler_ = this->add_on_set_parameters_callback(std::bind(&WallFollower::dynamicParametersCallback, this, std::placeholders::_1));

        this->cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
             "/cmd_vel",
             rclcpp::SystemDefaultsQoS());
        using namespace std::placeholders;
        this->scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            rclcpp::SensorDataQoS(),
            std::bind(&WallFollower::scan_callback, this, _1)
        );
    }
private:
    std::recursive_mutex mutex_;
    // Define a command velocity publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    // Define a laser scan subscriber
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_;
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg);

    /* TODO TASK - MILESTONE #4.2
        define dynamic parameter call back handle.
    */
    
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr dyn_params_handler_;

    rcl_interfaces::msg::SetParametersResult
        dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters);

    double following_angle_;
    double following_distance_;
    int64_t wall_side_;
    double buffer_zone_;
    double forward_velocity_;
    double angle_control_gain_1_;
    double angle_control_gain_2_;
    //double distance_control_gain_;
};

void WallFollower::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
{
    std::lock_guard<std::recursive_mutex> cfl(mutex_);
    /*TODO TASKS
        MILESTONE # 6.1. Process the received scan_msg to get the location of the closest object in robot's environment. 
        NOTE: the four pillars of will be visible from the Lidar sensor, you have to remove the distance 
        measurements of these four pillars by ignoring any measurement less than 0.2 meter.
    */
    
    double threshold = 0.2;
    
    auto comparator = [threshold](double a, double b) {
        if (a < threshold) return false;	// If a is under threshold, then comparator is false
        if (b < threshold) return true;		// If b is under threshold, and a is above (satisfies first condition), then a is true (return a)
        return a < b;	// If both are over threshold, keep the smaller one
    };
    auto min_distance = std::min_element(scan_msg->ranges.begin(), scan_msg->ranges.end(), comparator);
    // Get actual distance value from the iterator
    double min_value = *min_distance;
    // Get the index of the iterator
    int min_index = std::distance(scan_msg->ranges.begin(), min_distance);
    
    /*
        MILESTONE # 6.2. You have to calculate the bearing and the range of the closest object with respect to the robot frame. You have         
        to check the LaserScan message definition, and how the Lidar sensor is mounted with respective to  the robot's coordinate.
    */
    
    double min_angle_lidar = scan_msg->angle_min + (min_index*scan_msg->angle_increment);
    double min_angle_robot = min_angle_lidar; // + PI/2; if you are using actual turtlebot (not simulation)
        
    /*
        MILESTONE # 6.3. Write a Wall Follow Reactive Control that takes the bearing and range information of the closest object in the environment 
        as the input and publish a message on topic /cmd_vel to control the motion of the robot. 
            3.1 If the robot is far away from the wall, it should move towards its nearest wall at a constant speed until the robot 
            arrives at a distance of desired value + buffer zone, with respect to its closest wall. 
            3.2 Next, the robot enter the wall follow mode with the control lawy in in Algorithm 1
            3.3 The robot should deal with corner cases by only using reactive control with properly tuned control gains. 
    */
    
    geometry_msgs::msg::Twist cmd_vel_msg;
    
    if (min_value < scan_msg->range_max)
    {
        // Robot moves towards wall at speed of forward_velocity_ if too far from wall
        if (min_value > (following_distance_ + buffer_zone_))
        {
            if (abs(min_angle_robot) > PI/4)	// If wall is far to one side, then rotate robot
            {
                if (min_angle_robot > PI/4)
                    cmd_vel_msg.angular.z = 1.0;	// If far off left, rotate left
                else
                    cmd_vel_msg.angular.z = -1.0;	// Otherwise, is far off right, rotate right
            }
            else	// If the wall is not too far to one side of the robot, head towards wall
            {
                cmd_vel_msg.angular.z = 0;
                cmd_vel_msg.linear.x = forward_velocity_;
            }
        }
        else	// Wall is within range, drive along side wall
        {
            double angle_tracking_error = min_angle_robot - following_angle_;
            double distance_tracking_error = min_value - following_distance_;
            if (wall_side_ > 0)
	    {
	        if (abs(angle_tracking_error) > PI/10)
	        {
	            cmd_vel_msg.angular.z = (angle_control_gain_1_*angle_tracking_error) + (angle_control_gain_2_*min_value*sin(angle_tracking_error)/angle_tracking_error);
	        }
	        else
	        {
	            cmd_vel_msg.angular.z = (angle_control_gain_1_*angle_tracking_error) + (angle_control_gain_2_*min_value);
	        }
	    }
            else
            {
                if (abs(angle_tracking_error) > PI/10)
	        {
	            cmd_vel_msg.angular.z = (angle_control_gain_1_*angle_tracking_error) - (angle_control_gain_2_*distance_tracking_error*sin(angle_tracking_error));
	        }
	        else
	        {
	            cmd_vel_msg.angular.z = angle_control_gain_1_*angle_tracking_error - angle_control_gain_2_*distance_tracking_error;
	        }
	    }
            /*
            double linear_x = forward_velocity_ + distance_control_gain_*(min_value - following_distance_);
            if (linear_x > 0.0) {
		cmd_vel_msg.linear.x = linear_x;
	    }
	    else {
	        RCLCPP_WARN(this->get_logger(), "Reverse wanted, not possible, so stop");
	        cmd_vel_msg.linear.x = 0.0;
	    }
	    */
	    RCLCPP_INFO(this->get_logger(),"ae:%f,de:%f,ra:%f,fa:%f,omega%f",angle_tracking_error,distance_tracking_error,min_angle_robot,following_angle_,cmd_vel_msg.angular.z);
	    cmd_vel_msg.linear.x = forward_velocity_;
	}   
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "No object is detected");
        cmd_vel_msg.linear.x = 0.0;
    }
    
    // Publish velocity command to /cmd_vel
    cmd_vel_publisher_->publish(cmd_vel_msg);
}


rcl_interfaces::msg::SetParametersResult 
WallFollower::dynamicParametersCallback(std::vector<rclcpp::Parameter> parameters){
    std::lock_guard<std::recursive_mutex> cfl(mutex_);
    rcl_interfaces::msg::SetParametersResult result;
    /*TODO TASK - MILESTONE #5.1 
      Check whether update of a parameter in the node is requested, if yes and save the updated
      parameter value.
    */
    
    for (auto parameter : parameters) {
      const auto & param_type = parameter.get_type();
      const auto & param_name = parameter.get_name();
      if (param_type == ParameterType::PARAMETER_DOUBLE) {
        if (param_name == "following_distance") {
          following_distance_ = parameter.as_double();
          if (following_distance_ < 0.3)
          {
            RCLCPP_WARN(this->get_logger(), "You've set following_distance to be negative," " this isn't allowed, so the alpha1 will be set to 0.3 (to leave some space for the TurtleBot.");
            following_distance_ = 0.3;
          }
        }
        else if (param_name == "buffer_zone") {
          buffer_zone_ = parameter.as_double();
          if (buffer_zone_ < 0.0)
          {
            RCLCPP_WARN(this->get_logger(), "You've set buffer_zone to be negative," " this isn't allowed, so the alpha4 will be set to zero.");
            buffer_zone_ = 0.0;
          }
        }
        else if (param_name == "forward_velocity") {
          forward_velocity_ = parameter.as_double();
          if (forward_velocity_ < 0.0)
          {
            RCLCPP_WARN(this->get_logger(), "You've set forward_velocity to be negative," " TurtleBot cannot travel backward, so alpha5 will be set to zero (robot will not move).");
            forward_velocity_ = 0.0;
          }
        }
        else if (param_name == "angle_control_gain_1") {
          angle_control_gain_1_ = parameter.as_double();
          if (angle_control_gain_1_ < 0.0)
          {
            RCLCPP_WARN(this->get_logger(), "You've set angle_control_gain_1 to be negative," " this isn't allowed, so the alpha6 will be set to zero (robot will not rotate).");
            angle_control_gain_1_ = 0.0;
          }
        }
        /*
        else if (param_name == "angle_control_gain_2") {
          angle_control_gain_2_ = parameter.as_double();
          if (angle_control_gain_2_ < 0.0)
          {
            RCLCPP_WARN(this->get_logger(), "You've set angle_control_gain_2 to be negative," " this isn't allowed, so the alpha7 will be set to zero (robot will not rotate).");
            angle_control_gain_2_ = 0.0;
          }
        }
        */
        /*
        else if (param_name == "distance_control_gain") {
          distance_control_gain_ = parameter.as_double();
          if (distance_control_gain_ < 0.0)
          {
            RCLCPP_WARN(this->get_logger(), "You've set distance_control_gain to be negative," " this isn't allowed, so the alpha8 will be set to zero (robot will not move).");
            distance_control_gain_ = 0.0;
          }
        }
        */
      }
      else if (param_type == ParameterType::PARAMETER_INTEGER) {
        if (param_name == "wall_side") {
          wall_side_ = parameter.as_int();
          if (wall_side_ > 0)
	    following_angle_ = PI/2;
	  else
	    following_angle_ = -PI/2;
        }
      }
    }
  result.successful = true;
  return result;
}

int main(int argc, char ** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<WallFollower>());
	rclcpp::shutdown();
	return 0;
}
