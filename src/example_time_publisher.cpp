#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>

int main(int argc, char **argv)
{
    // Initialize ROS with the name "time_publisher"
    ros::init(argc, argv, "time_publisher");
    ros::NodeHandle nh;

    double publish_rate;
    // Try to get the 'publish_rate' parameter from the ROS parameter server.
    // Default to 1.0 if the parameter is not set.
    if (!nh.getParam("time_publisher/publish_rate", publish_rate))
    {
        ROS_WARN("Failed to get 'publish_rate' parameter. Defaulting to 1.0 Hz.");
        publish_rate = 1.0;
    }

    // Advertise the "current_time" topic with a message queue size of 1000
    ros::Publisher time_pub = nh.advertise<std_msgs::String>("current_time", 1000);

    // Set the rate at which to loop and publish messages
    ros::Rate loop_rate(publish_rate);

    // Main loop: runs while the ROS node is okay
    while (ros::ok())
    {
        std_msgs::String msg;
        
        std::stringstream ss;
        // Fetch the current time and prepare the message string
        ss << "Current Time: " << ros::Time::now();
        msg.data = ss.str();
        
        // Publish the message on the "current_time" topic
        time_pub.publish(msg);
        
        // Process any incoming messages in callback queues
        ros::spinOnce();
        // Sleep for the remainder of the loop to achieve the desired loop rate
        loop_rate.sleep();
    }

    return 0;
}
