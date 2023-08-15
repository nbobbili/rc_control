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
        float dshot_normalised[4];
        unsigned int rc_channels[16];
        float rc_mins[4], rc_max[4];

        //Callback function Prototypes
        void rc_in_cb_(const mavros_msgs::RCIn::ConstPtr& msg);
        bool velocity();
        bool land();
        bool takeoff();
        bool get_RC_Calibration();        

        //MAVROS Params to get
        
    command(){
            //nh.getParam("quadrotor",quadrotor); //Uncomment when uploading to robot
            rc_in_cb = nh.subscribe("quadrotor/mavros/rc/in",1,&command::rc_in_cb_,this);
            vel_cmd = nh.serviceClient<mav_manager::Vec4>("/quadrotor/mav_services/setDesVelInBodyFrame");
            land_trigger = nh.serviceClient<std_srvs::Trigger>("/quadrotor/mav_services/land");
            takeoff_trigger = nh.serviceClient<std_srvs::Trigger>("/quadrotor/mav_services/takeoff");
            rc_calibration = nh.serviceClient<mavros_msgs::ParamGet>("/quadrotor/mavros/param/get");
            bool get_RC_Calibration();
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
    rc_channels[5] = msg->channels[5]; //Kill-Switch
    rc_channels[6] = msg->channels[6]; //Mode_change
    rc_channels[7] = msg->channels[7]; //Land_takeoff_switch
    rc_channels[7] = msg->channels[8]; //Land_takeoff_trigger
    rc_channels[7] = msg->channels[9];
     
    
    //Normalising DSHOT
    for (int i {0}; i < 3; i++)
    {
        dshot_normalised[i] = (static_cast<float>(rc_channels[i]) - rc_mins[i])/(rc_max[i]-rc_mins[i]); //Changes: Replace with min and max
    }
    std::cout << "Normalised Dshot: " << dshot_normalised[2] << std::endl;

    //Calling Vec4 Service
    if (rc_channels[6] > 1700) //To make sure Velocity command is not sent all the time in off-board.
    {
        command::velocity();
    }

    if ((rc_channels[8] < 1350)&&(rc_channels[9] > 1750))
    {
        command::land();
    }
    else if ((rc_channels[8] > 1650)&&(rc_channels[9] > 1750))
    {
        command::takeoff();
    }
}

bool command::velocity()
{
    bool success = false;
    
    mav_manager::Vec4 main_channels;
    main_channels.request.goal[0] = dshot_normalised[1]; //Pitch
    main_channels.request.goal[1] = dshot_normalised[0]; //Roll
    main_channels.request.goal[2] = dshot_normalised[2]; //Throttle
    main_channels.request.goal[3] = dshot_normalised[3]; //Yaw
    
    vel_cmd.call(main_channels);

    return success;
    
}

bool command::land()
{
    bool success = false;
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
        params.push_back("{'param_id': 'RC1_MIN'");
        params.push_back("{'param_id': 'RC2_MIN'");
        params.push_back("{'param_id': 'RC3_MIN'");
        params.push_back("{'param_id': 'RC4_MIN'");
        params.push_back("{'param_id': 'RC1_MAX'");
        params.push_back("{'param_id': 'RC2_MAX'");
        params.push_back("{'param_id': 'RC3_MAX'");
        params.push_back("{'param_id': 'RC4_MAX'");
    
    for (int i = 0; i < (params.size()-4); ++i)
    {
        values.request.param_id = params[i];
        try
        {   
            rc_calibration.call(values);
            rc_mins[i] = values.response.value.real;
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