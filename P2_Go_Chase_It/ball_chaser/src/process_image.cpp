#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>
#define EPS 1e-3

class SubscribeAndRequest
{
private:
    float lin_x_;
    float ang_z_;

    // client that can request services
    ros::ServiceClient client_;

    ros::Subscriber sub1_;

    // Create a ROS NodeHandle
    ros::NodeHandle nh_;
    
public:
    SubscribeAndRequest();
    void process_image_callback(const sensor_msgs::Image img);
    void drive_robot(float lin_x, float ang_z);
}; // End of class SubscribeAndRequest

SubscribeAndRequest::SubscribeAndRequest()
{
    // Define a client service capable of requesting services from command_robot
    client_ = nh_.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    sub1_ = nh_.subscribe("/camera/rgb/image_raw", 10, &SubscribeAndRequest::process_image_callback, this);

    lin_x_ = 0.;
    ang_z_ = 0.;
}

void SubscribeAndRequest::drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    
    if (!client_.call(srv))
        ROS_ERROR("Failed to call service command_robot");
}

// This callback function continuously executes and reads the image data
void SubscribeAndRequest::process_image_callback(const sensor_msgs::Image img)
{
    // std_msgs/Header header
    //   uint32 seq
    //   time stamp
    //   string frame_id
    // uint32 height
    // uint32 width
    // string encoding
    // uint8 is_bigendian
    // uint32 step
    // uint8[] data

    uint white_pixel_threshold = 250;
    bool find_ball = false;

    // Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    int step;
    int step_sum = 0;
    int occupancy_num = 0;
    for (int h = 0; h < img.height; ++h)
    {
        for (step = 0; step < img.step; step+=3) // 3 because of RGB channels
        {
            int idx = step + h*img.step;
            if (img.data[idx] > white_pixel_threshold 
                && img.data[idx+1] > white_pixel_threshold 
                && img.data[idx+2] > white_pixel_threshold)
            {
                find_ball = true;
                step_sum += step;
                occupancy_num += 1;
            }
        }
    }
    
    step = step_sum / std::max(occupancy_num, 1);

    float lin_x = 0., ang_z = 0.;
    if (find_ball)
    {
        lin_x = 1.0;     
        if (step < img.step / 3.0) // ball is in left side of image
            ang_z = 2.0;
        else if (step < img.step * 2.0 / 3.0) // ball is in middle of image    
            ang_z = 0.;
        else  // ball is in right side of image
            ang_z = -2.0; 
    }

    if (occupancy_num > (img.step/3) * img.height * 0.05)
    {
        ROS_INFO("Too close!");
        lin_x = 0.;
    }

    // If current command is different from the previous command 
    // then request server to drive robot
    if (fabs(lin_x_ - lin_x) > EPS || fabs(ang_z_ - ang_z) > EPS)
    {
        drive_robot(lin_x, ang_z);
        lin_x_ = lin_x;
        ang_z_ = ang_z;
    }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");

    // Create an object of class SubscribeAndRequest that will take care of everything
    SubscribeAndRequest SAR;
    
    // Handle ROS communication events
    ros::spin();

    return 0;
}