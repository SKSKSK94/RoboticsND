#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>

#define EPS 0.2

class SubscribeAndPublish
{
private:
    bool pickup_reached_ = false;

    ros::Subscriber odom_sub_;

    ros::Publisher marker_pub_;

    // Create a ROS NodeHandle
    ros::NodeHandle nh_;

    double PICK_UP_X;
    double PICK_UP_Y;
    double DROP_OFF_X;
    double DROP_OFF_Y;

    visualization_msgs::Marker marker_;

    // Set our shape type to be a cube
    uint32_t shape_ = visualization_msgs::Marker::CUBE;

    double cal_distance(double A_x, double A_y, double B_x, double B_y)
    {
        return sqrt(pow((A_x-B_x), 2) + pow((A_y-B_y), 2));
    }
    
public:
    SubscribeAndPublish();
    void odom_callback(const nav_msgs::Odometry odom);
}; // End of class SubscribeAndPublish

SubscribeAndPublish::SubscribeAndPublish()
{
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    
    odom_sub_ = nh_.subscribe("/odom", 10, &SubscribeAndPublish::odom_callback, this);    

    // Get node name
    std::string node_name = ros::this_node::getName();

    // Get joints min and max parameters
    nh_.getParam(node_name + "/PICK_UP_X", PICK_UP_X);
    nh_.getParam(node_name + "/PICK_UP_Y", PICK_UP_Y);
    nh_.getParam(node_name + "/DROP_OFF_X", DROP_OFF_X);
    nh_.getParam(node_name + "/DROP_OFF_Y", DROP_OFF_Y);

    // Set our shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CUBE;

    // Set the timestamp, frame ID.  See the TF tutorials for information on these.
    marker_.header.stamp = ros::Time::now();
    marker_.header.frame_id = "map";

    // Set the scale of the marker
    marker_.scale.x = 0.15;
    marker_.scale.y = 0.15;
    marker_.scale.z = 0.15;

    // Set the color -- be sure to set alpha to something non-zero!
    marker_.color.r = 0.0f;
    marker_.color.g = 1.0f;
    marker_.color.b = 0.0f;
    marker_.color.a = 1.0;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker_.pose.position.x = PICK_UP_X;
    marker_.pose.position.y = PICK_UP_Y;
    marker_.pose.position.z = 0;

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker_.ns = "add_markers";
    marker_.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker_.type = shape_;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker_.action = visualization_msgs::Marker::ADD;

    marker_.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub_.getNumSubscribers() < 1)
    {
        ROS_WARN_ONCE("Please create a subscriber to the marker");
        sleep(1);
    }

    // Publish the marker at the pickup zone
    marker_pub_.publish(marker_);

}

void SubscribeAndPublish::odom_callback(const nav_msgs::Odometry odom)
{
    double robot_pose_x = odom.pose.pose.position.x;
    double robot_pose_y = odom.pose.pose.position.y;

    double pickup_distance = cal_distance(robot_pose_x, robot_pose_y, PICK_UP_X, PICK_UP_Y);
    double dropoff_distance = cal_distance(robot_pose_x, robot_pose_y, DROP_OFF_X, DROP_OFF_Y);

    // If robot reached the pickup zone
    if (pickup_distance < EPS)
    {
        pickup_reached_ = true;

        // Set the timestamp.  See the TF tutorials for information on these.
        marker_.header.stamp = ros::Time::now();

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker_.pose.position.x = PICK_UP_X;
        marker_.pose.position.y = PICK_UP_Y;
        marker_.pose.position.z = 0;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker_.action = visualization_msgs::Marker::DELETE;

        marker_.lifetime = ros::Duration();

        // Publish the marker at the pickup zone
        marker_pub_.publish(marker_);
    }

    // If robot had reached the pickup zone before and now the drop off zone
    if (pickup_reached_ == true && dropoff_distance < EPS/2)
    {
        // Set the timestamp.  See the TF tutorials for information on these.
        marker_.header.stamp = ros::Time::now();

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker_.pose.position.x = DROP_OFF_X;
        marker_.pose.position.y = DROP_OFF_Y + 2*marker_.scale.y;
        marker_.pose.position.z = 0;

        // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
        marker_.action = visualization_msgs::Marker::ADD;

        marker_.lifetime = ros::Duration();

        // Publish the marker at the dropoff zone
        marker_pub_.publish(marker_);
    }    
}

int main( int argc, char** argv )
{
    // Initialize the add_markers node
    ros::init(argc, argv, "add_markers");

    // Create an object of class SubscribeAndPublish that will take care of everything
    SubscribeAndPublish SAP;

    // Handle ROS communication events
    ros::spin();

    return 0;
}