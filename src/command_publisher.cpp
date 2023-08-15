#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>

int main (int argc, char **argv)
{ 
    ros::init(argc, argv,"command_publisher_node");
    ros::NodeHandle nh;
    ros::Publisher channel_pub = nh.advertise<mavros_msgs::RCIn>("/quadrotor/mavros/rc/in",1);

    ros::Rate loop_rate(5);
    int slope = 1;
    int i = 0;
    while(ros::ok())
    {
        mavros_msgs::RCIn msg;
        msg.header.frame_id = "body";
        msg.header.stamp = ros::Time::now();
        msg.channels.resize(8);
        msg.channels[0] = 1500;
        msg.channels[1] = 1500;
        msg.channels[2] = 1100 + i*50;
        msg.channels[3] = 1500;
        if (msg.channels[2] == 1100)
        {
            slope = 1;
        }
        else if (msg.channels[2] == 1800)
        {
            slope = 2;
    
        }
        switch (slope)
        {
        case 1:
            i++;
            break;
        case 2:
            i--;
            break;
        }
        msg.rssi = -50;
        channel_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();

    };
    return 0;
}