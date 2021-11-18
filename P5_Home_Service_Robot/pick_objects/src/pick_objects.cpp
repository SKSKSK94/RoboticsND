#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
    // Initialize the pick_objects node
    ros::init(argc, argv, "pick_objects");

    // Tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    // Wait for move_base action server to come up every 5 seconds
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    double PICK_UP_X;
    double PICK_UP_Y;
    double DROP_OFF_X;
    double DROP_OFF_Y;

    // Create a ROS NodeHandle
    ros::NodeHandle nh;

    // Get node name
    std::string node_name = ros::this_node::getName();

    // Get joints min and max parameters
    nh.getParam(node_name + "/PICK_UP_X", PICK_UP_X);
    nh.getParam(node_name + "/PICK_UP_Y", PICK_UP_Y);
    nh.getParam(node_name + "/DROP_OFF_X", DROP_OFF_X);
    nh.getParam(node_name + "/DROP_OFF_Y", DROP_OFF_Y);

    move_base_msgs::MoveBaseGoal goal;

    /********************* Move to the desired pickup zone *********************/
    // Set up the frame parameters
    // Tell the move_base to move in the "map" coordinate frame.
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach
    goal.target_pose.pose.position.x = PICK_UP_X;
    goal.target_pose.pose.position.y = PICK_UP_Y;
    goal.target_pose.pose.orientation.z = 0.9238; // 135 degree
    goal.target_pose.pose.orientation.w = 0.3827; // 135 degree

    // Send the goal position and orientation for the robot to reach
    ROS_INFO("Robot is traveling to the pickup zone");
    ac.sendGoal(goal);    

    // Wait an infinite time for the results
    // wait for the goal to finish using the ac.waitForGoalToFinish call 
    // which will block until the move_base action is done processing 
    // the goal we sent it
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Robot picked up the virtual object");
        ros::Duration(5.0).sleep(); // sleep for 5 seconds

        /********************* Move to the desired dropoff zone *********************/
        // Define a position and orientation for the robot to reach
        goal.target_pose.pose.position.x = DROP_OFF_X;
        goal.target_pose.pose.position.y = DROP_OFF_Y;
        goal.target_pose.pose.orientation.z = -0.5; // -60 degree
        goal.target_pose.pose.orientation.w = 0.866; // -60 degree

        // Send the goal position and orientation for the robot to reach
        ROS_INFO("Robot is traveling to the drop off zone");
        ac.sendGoal(goal);    

        // Wait an infinite time for the results
        // wait for the goal to finish using the ac.waitForGoalToFinish call 
        // which will block until the move_base action is done processing 
        // the goal we sent it
        ac.waitForResult();

        // Check if the robot reached its goal
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Robot dropped the virtual object");
        else
            ROS_INFO("The base failed to move to dropoff zone for some reasons");
    }
    else
        ROS_INFO("The base failed to move to pickup zone for some reasons");

    return 0;
}