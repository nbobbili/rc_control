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
	    command();
        //ROS Objects
        ros::NodeHandle nh;
        ros::Subscriber rc_in_cb;
        ros::ServiceClient vel_cmd,land_trigger,takeoff_trigger, rc_calibration, hover_trigger, Motors_on;
        //Variables
        float dshot_normalised[4];
        uint16_t rc_channels[8];
        double rc_mins[4];
        double rc_max[4];

        //Callback function Prototypes
        void rc_in_cb_(const mavros_msgs::RCIn::ConstPtr& msg);
        bool velocity();
        bool land();
        bool takeoff();
        bool get_RC_Calibration();
        bool hover();
        bool switch_on_motors();
};

command::command()
{
    quadrotor =std::getenv("MAV_NAME");
    rc_in_cb = nh.subscribe("/"+quadrotor+"/mavros/rc/in",1,&command::rc_in_cb_,this);
    vel_cmd = nh.serviceClient<mav_manager::Vec4>("/"+quadrotor+"/mav_services/setDesVelInBodyFrame");
    land_trigger = nh.serviceClient<std_srvs::Trigger>("/"+quadrotor+"/mav_services/land");
    takeoff_trigger = nh.serviceClient<std_srvs::Trigger>("/"+quadrotor+"/mav_services/takeoff");
    hover_trigger = nh.serviceClient<std_srvs::Trigger>("/"+quadrotor+"/mav_services/hover");
    rc_calibration = nh.serviceClient<mavros_msgs::ParamGet>("/"+quadrotor+"/mavros/param/get");
    Motors_on = nh.serviceClient<std_srvs::SetBool>("/"+quadrotor+"/mav_services/motors");
    dshot_normalised[4] = {0.0};
    rc_channels[8] ={0};
    rc_mins[4] = {0};
    rc_max[4] = {0};
    bool success = get_RC_Calibration();
    //motors = false;
}
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

    float scale = 1.8;
    
    //Normalising DSHOT
    for (int i {0}; i < 4; i++)
    {
        dshot_normalised[i] = scale*(static_cast<float>(rc_channels[i]) - 1500.00 )/(static_cast<float>(rc_max[i]) -static_cast<float>(rc_mins[i]));
    }
    
    static bool velocity_mode = false;
    static bool motors = false;
    //Motors_on Service (Using mode switch channel)
    if ((rc_channels[5] > 1350) && !motors) //Avoid calling motors_on when already on
    {
        command::switch_on_motors();
        motors = true;
    }

    //Velocity commands
    if (rc_channels[5] > 1750) //To make sure Velocity command is not sent all the time in off-board.
    {
        command::velocity();
	    velocity_mode = true;
    }
    else if ((rc_channels[5] < 1750) &&(rc_channels[5] > 1350) && velocity_mode) //Exit velocity line tracker to avoid using last known vel cmd
    {
	    command::hover();
	    velocity_mode = false;
    }
    
    static int land_takeoff_trigger = 0; //Trigger for making sure land and take-off is not called always

    //Land-Takeoff
    //Add if condition to check off-board
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
    if (rc_channels[6] < 1350 && (land_takeoff_trigger > 0)) //To be bale to call land multiple times if needed.
    {
	land_takeoff_trigger = 1;
    }

}

bool command::velocity()
{
    bool success = false;
    
    mav_manager::Vec4 main_channels;
    //Deadzone
   // for (int i =0; i<4; i++)
   // {
   //     if ((dshot_normalised[i] < 0.525) && (dshot_normalised[i]>0.475))
   //     {
  //          dshot_normalised[i] = 0.5;
//	    std::cout << "In the deadzone: " << std::endl;
//	    std::cout << "velocity setpoint: " << dshot_normalised[i] << std::endl;
 //       }
//    }
    main_channels.request.goal[0] = dshot_normalised[1]; //Pitch
    main_channels.request.goal[1] = dshot_normalised[0]; //Roll
    main_channels.request.goal[2] = dshot_normalised[2]; //Throttle
    main_channels.request.goal[3] = dshot_normalised[3]; //Yaw
    //std::cout << "velocity setpoint: " << dshot_normalised[2] << std::endl;
    vel_cmd.call(main_channels);

    return success;
    
}

bool command::switch_on_motors()
{
    bool status = false;
    std_srvs::SetBool motors_trigger;
    motors_trigger.request.data = true;
    try
    {
        Motors_on.call(motors_trigger);
        status = true;
    }
    catch(const std::exception& e)
    {
        status = false;
    }
    //motors = motors_trigger.response.success;
    return status;
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
        takeoff_trigger.call(trigger_takeoff);
        success = true;
    }
    catch(const std::exception& e)
    {
        success = false;
    }
    return success;
}

bool command::hover()
{
    bool success = false;
    std_srvs::Trigger trigger_hover;
    ROS_INFO("Hovering");
    try
    {
        hover_trigger.call(trigger_hover);
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

    for (int i = 0; i < 8; i++)
    {
        values.request.param_id = params[i];
        try
        {   
            rc_calibration.call(values);
	    if (i<4)
	    {
            rc_mins[i] = values.response.value.real;
	    }
	    else
	    {
	        rc_max[i-4] = values.response.value.real;
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
