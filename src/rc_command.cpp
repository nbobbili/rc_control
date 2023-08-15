#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <mav_manager/Vec4.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <mavros_msgs/ParamGet.h>
#include <vector>

class command
{
    private:
        std::string quadrotor;
    public:
        //ROS Objects
        ros::NodeHandle nh;
        ros::Subscriber rc_in_cb;
        ros::ServiceClient vel_cmd,land_trigger,takeoff_trigger, rc_calibration;
        //Variables
        float dshot_normalised[4] = {0.0};
        uint16_t rc_channels[8] ={0};
        float rc_mins[4] = {0.0};
        float rc_max[4] = {0.0};

        //Callback function Prototypes
        void rc_in_cb_(const mavros_msgs::RCIn::ConstPtr& msg);
        bool velocity();
        bool land();
        bool takeoff();
        bool get_RC_Calibration();        

        //MAVROS Params to get
        
    command(){
            quadrotor =std::getenv("MAV_NAME"); //Uncomment when uploading to robot
	    std::cout << "MAV_NAME: /" << quadrotor << std::endl;
            rc_in_cb = nh.subscribe("/"+quadrotor+"/mavros/rc/in",1,&command::rc_in_cb_,this);
            vel_cmd = nh.serviceClient<mav_manager::Vec4>("/"+quadrotor+"/mav_services/setDesVelInBodyFrame");
            land_trigger = nh.serviceClient<std_srvs::Trigger>("/"+quadrotor+"/mav_services/land");
            takeoff_trigger = nh.serviceClient<std_srvs::Trigger>("/"+quadrotor+"/mav_services/takeoff");
            rc_calibration = nh.serviceClient<mavros_msgs::ParamGet>("/"+quadrotor+"/mavros/param/get");
            get_RC_Calibration();
	    // std::cout << "Calibration Value: " << rc_mins[0] << std::endl;
    }
};

void command::rc_in_cb_(const mavros_msgs::RCIn::ConstPtr& msg)
{
    //Main DSHOT Channels
    rc_channels[0] = msg->channels[0]; //Roll
    rc_channels[1] = msg->channels[1]; //Pitch
    rc_channels[2] = msg->channels[2]; //Throttle
    rc_channels[3] = msg->channels[3]; //Yaw
    
    //Switches
    rc_channels[4] = msg->channels[4]; //Arming
    rc_channels[5] = msg->channels[5]; //Mode_change
    rc_channels[6] = msg->channels[6]; //Land_Takeoff_Mode
    rc_channels[7] = msg->channels[7]; //Kill 

   // ROS_INFO("RC_CHANNEL[8}: %5i",static_cast<int>(msg->channels[8])); 
   // ROS_INFO("RC_CHANNEL[8}: %5i",rc_channels[8]); 
    
    //Normalising DSHOT
    for (int i {0}; i < 3; i++)
    {
        dshot_normalised[i] = (static_cast<float>(rc_channels[i]) -static_cast<float>(rc_mins[i]) )/( static_cast<float>(rc_max[i]) -static_cast<float>(rc_mins[i]) ); //Changes: Replace with min and max
    }
    std::cout << "Normalised Dshot: " << dshot_normalised[2] << std::endl;

    //Calling Vec4 Service
    if (rc_channels[5] > 1700) //To make sure Velocity command is not sent all the time in off-board.
    {
        command::velocity();
    }
    static int land_takeoff_trigger = 0;
    //Land-Takeoff
    if ((rc_channels[6] > 1350)&&(rc_channels[6] <1600)&&(land_takeoff_trigger==0))
    {
        command::takeoff();
	land_takeoff_trigger++;
    }
    else if ((rc_channels[6] > 1750)&&(land_takeoff_trigger==1))
    {
        command::land();
	land_takeoff_trigger++;
    }
   // ROS_WARN("Not Normalised Throttle: %f", dshot_normalised[2]);
   // ROS_INFO("Throttle: %f", (2.0*dshot_normalised[2]-1));
}

bool command::velocity()
{
    bool success = false;
    
    mav_manager::Vec4 main_channels;
    main_channels.request.goal[0] = dshot_normalised[1]; //Pitch
    main_channels.request.goal[1] = dshot_normalised[0]; //Roll
    main_channels.request.goal[2] =2.0*dshot_normalised[2]-1; //Throttle
    main_channels.request.goal[3] = dshot_normalised[3]; //Yaw
    
    vel_cmd.call(main_channels);

    return success;
    
}

bool command::land()
{
    bool success = false;
    ROS_INFO("Landing");
    std_srvs::Trigger trigger_land;
    try
    {
        land_trigger.call(trigger_land);
        success = true;
    }
    catch(const std::exception& e)
    {
        success = false;
    }
    return success;
    
}

bool command::takeoff()
{
    bool success = false;
    std_srvs::Trigger trigger_takeoff;
    ROS_INFO("Taking off");
    try
    {
        land_trigger.call(trigger_takeoff);
        success = true;
    }
    catch(const std::exception& e)
    {
        success = false;
    }
    return success;
}

bool command::get_RC_Calibration() 
{
    bool success = false;
    mavros_msgs::ParamGet values;
    //Add parameters as needed
    std::vector<std::string> params;
        params.push_back("RC1_MIN");
        params.push_back("RC2_MIN");
        params.push_back("RC3_MIN");
        params.push_back("RC4_MIN");
        params.push_back("RC1_MAX");
        params.push_back("RC2_MAX");
        params.push_back("RC3_MAX");
        params.push_back("RC4_MAX");

    for (int i = 0; i < 7; i++)
    {
        values.request.param_id = params[i];
	std::cout <<"Getting Param: " << params[i] << std::endl;
        try
        {   
            rc_calibration.call(values);
	    if (i<5)
	    {
            rc_mins[i] = values.response.value.real;
	    }
	    else
	    {
	    rc_max[i] = values.response.value.real;
	    }
        }
        catch(const std::exception& e)
        {
            success = false;
        }
    }
    return success;
}

int main (int argc, char **argv)
{
    ros::init(argc,argv,"rc_command_node");
    command command;
    ros::spin();
    return 0;
}
