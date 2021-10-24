#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include "ball_chaser/DriveToTarget.h"

class ResponseAndPublish
{
private:
    float lin_x_;
    float ang_z_;

    ros::ServiceServer server_;

    ros::Publisher motor_command_publisher_;

    // Create a ROS NodeHandle
    ros::NodeHandle nh_;
    
public:
    ResponseAndPublish();
    bool handle_drive_request(ball_chaser::DriveToTarget::Request &req, ball_chaser::DriveToTarget::Response &res);
}; // End of class ResponseAndPublish


ResponseAndPublish::ResponseAndPublish()
{
    // Inform ROS master that we will be publishing a message of type geometry_msgs::Twist
    // on the robot actuation topic with a publishing queue size of 10
    motor_command_publisher_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    server_ = nh_.advertiseService("/ball_chaser/command_robot", &ResponseAndPublish::handle_drive_request, this);   
    ROS_INFO("Ready to command robot"); 
}


bool ResponseAndPublish::handle_drive_request(ball_chaser::DriveToTarget::Request &req, 
    ball_chaser::DriveToTarget::Response &res)
{
    ROS_INFO("DriveToTargetRequest received - linear_x:%1.2f, angular_z:%1.2f", (float)req.linear_x, (float)req.angular_z);

    // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;

    // Publish motor_command to drive the robot
    motor_command_publisher_.publish(motor_command);

    res.msg_feedback = "Wheel angles set (linear_x, angular_z) = (" + std::to_string(req.linear_x) + ", " + std::to_string(req.angular_z) + ")";
    ROS_INFO_STREAM(res.msg_feedback);
    
    return true;
}

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create an object of class ResponseAndPublish that will take care of everything
    ResponseAndPublish RAP;
    
    // Handle ROS communication events
    ros::spin();
}