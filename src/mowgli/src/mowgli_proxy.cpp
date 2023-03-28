 /*
 * Mowgli to OpenMower Proxy V 1.1
 * (c) Georg Swoboda <cn@warp.at> 2022
 *
 * https://github.com/cloudn1ne/MowgliRover
 *
 *
 * v1.0: inital release
 * v1.1: DR experimentaly code added
 *
 */

#include "ros/ros.h"

/* OM */
#include <mower_msgs/Status.h>
#include <mower_msgs/ESCStatus.h>
// #include <mower_msgs/MowerControlSrv.h>
#include <mower_msgs/EmergencyStopSrv.h>

/* Mowgli */
#include "std_srvs/SetBool.h"
#include <mowgli/MowgliProxyConfig.h>
#include <mowgli/status.h>
#include <mowgli/Led.h>
#include <mowgli/wheelticks.h>

#include <dynamic_reconfigure/server.h>
#include <xbot_msgs/WheelTick.h>
#include <mower_msgs/HighLevelStatus.h>

#define WHEEL_DISTANCE_M 0.325

// #define MOWGLI_DEBUG  1
#define MOWGLI_DR_DEBUG 1

// Dynamic Reconfigure for MowgliProxy
mowgli::MowgliProxyConfig config;

// OM Status
// mower_msgs::Status om_mower_status;

// LED
ros::ServiceClient ledSetClient;
ros::ServiceClient ledClrClient;

// Pubs
ros::Publisher pubOMStatus;

// Service Clients
ros::ServiceClient mowClient;
bool mowEmergencyDisableFlag = false;

//Subs
ros::Subscriber subMowgliStatus;

 ros::Publisher wheel_tick_pub;

 ros::Time last_cmd_vel;

 int speed_l;

 int speed_r;

 bool setEmergencyStop(mower_msgs::EmergencyStopSrvRequest &req, mower_msgs::EmergencyStopSrvResponse &res) {
    ROS_ERROR_STREAM("mowgli_proxy: setEmergencyStop");
    if (!mowEmergencyDisableFlag)
    {
        mowEmergencyDisableFlag = true;
        std_srvs::SetBool mow_srv;
        mow_srv.request.data = false;
        mowClient.call(mow_srv);
        ROS_WARN_STREAM("mowgli_proxy: EMERGENCY Blade ENABLED -> DISABLED");
    }
    return true;
}

/*
* translate /mowgli/status to /mower/status
*/
void MowgliStatusCB(const mowgli::status::ConstPtr &msg) 
{
    mower_msgs::Status om_mower_status;
#ifdef MOWGLI_DEBUG    
    ROS_INFO_STREAM("mowgli_proxy: MowgliStatusCB");
#endif

     om_mower_status.rain_detected = msg->rain_detected;
     om_mower_status.emergency = msg->emergency_stopbutton_triggered | msg->emergency_tilt_mech_triggered | msg->emergency_tilt_accel_triggered |
                                 msg->emergency_right_wheel_lifted | msg->emergency_left_wheel_lifted | msg->emergency_left_stop | msg->emergency_right_stop;
     /* not used anymore*/
     om_mower_status.v_charge = msg->v_charge;
     om_mower_status.charge_current = msg->i_charge;
     om_mower_status.v_battery = msg->v_battery;
     om_mower_status.left_esc_status.current = msg->left_power;
     om_mower_status.right_esc_status.current = msg->right_power;
     om_mower_status.mow_esc_status.temperature_motor = msg->blade_temperature;
     om_mower_status.mow_esc_status.tacho = msg->blade_RPM;
     om_mower_status.mow_esc_status.current = msg->blade_power;
     om_mower_status.mow_esc_status.status = msg->blade_motor_enabled;
     om_mower_status.mow_esc_status.temperature_motor = msg->blade_temperature;
     om_mower_status.left_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
     om_mower_status.right_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;

//    dr_left_encoder_ticks = msg->left_encoder_ticks;
//    dr_right_encoder_ticks = msg->right_encoder_ticks;

    pubOMStatus.publish(om_mower_status);
}

 void MowgliWheelTicksCB(const mowgli::wheelticks::ConstPtr &msg) {
    xbot_msgs::WheelTick wheel_tick;
    wheel_tick.stamp = msg->stamp;
     wheel_tick.wheel_tick_factor = msg->wheel_tick_factor;
     wheel_tick.valid_wheels = msg->valid_wheels;
     wheel_tick.wheel_direction_fl = msg->wheel_direction_fl;
     wheel_tick.wheel_ticks_fl = msg->wheel_ticks_fl;
     wheel_tick.wheel_direction_fr = msg->wheel_direction_fr;
     wheel_tick.wheel_ticks_fr = msg->wheel_ticks_fr;
     wheel_tick.wheel_direction_rl = msg->wheel_direction_rl;
     wheel_tick.wheel_ticks_rl = msg->wheel_ticks_rl;
     wheel_tick.wheel_direction_rr = msg->wheel_direction_rr;
     wheel_tick.wheel_ticks_rr= msg->wheel_ticks_rr;
    wheel_tick_pub.publish(wheel_tick);
}

 void reconfigureCB(mowgli::MowgliProxyConfig &c, uint32_t level)
{
    ROS_INFO_STREAM("mowgli_proxy: setting new MowgliProxy Configuration");
    config = c;
}

 void highLevelStatusReceived(const mower_msgs::HighLevelStatus::ConstPtr &msg) {
       /* if (msg->gps_quality_percent < 0.8) {
            mowgli::Led ledsrv;

            // clear
            ledsrv.request.led = 139;
            ledClrClient.call(ledsrv);
            ROS_WARN("GPS Fix LED turned off");
        } else {
            mowgli::Led ledsrv;

            // set
            ledsrv.request.led = 139;
            ledSetClient.call(ledsrv);
            ROS_WARN("GPS Fix LED turned on");
        }*/
 }

 int main(int argc, char **argv)
{
    ros::init(argc, argv, "mowgli_proxy");


    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");

    ROS_INFO_STREAM("mowgli_proxy: Starting mowgli_proxy");

    // Dynamic reconfiguration server
    dynamic_reconfigure::Server<mowgli::MowgliProxyConfig> reconfig_server(paramNh);
    reconfig_server.setCallback(reconfigureCB);

    // Mowgli Topics
    subMowgliStatus = n.subscribe("mowgli/status", 50, MowgliStatusCB);    
    subMowgliStatus = n.subscribe("mowgli/wheelticks", 50, MowgliWheelTicksCB);
    // subMowgliOdom = n.subscribe("mowgli/odom", 50, MowgliOdomCB);

    // OpenMower Status
    pubOMStatus = n.advertise<mower_msgs::Status>("mower/status", 50);
    wheel_tick_pub = n.advertise<xbot_msgs::WheelTick>("mower/wheel_ticks", 1);

    // Services required by OpenMower
    ros::ServiceServer om_emergency_service = n.advertiseService("mower_service/emergency", setEmergencyStop);

    ros::Subscriber high_level_status_sub = n.subscribe("/mower_logic/current_state", 0, highLevelStatusReceived);

    // Led Control
    ledSetClient = n.serviceClient<mowgli::Led>("mowgli/SetLed");
    ROS_INFO("mowgli_proxy: Waiting for mowgli/SetLed server");
    if (!ledSetClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("mowgli_proxy: SetLed server service not found.");
        return 1;
    }
    ledClrClient = n.serviceClient<mowgli::Led>("mowgli/ClrLed");
    ROS_INFO("mowgli_proxy: Waiting for mowgli/ClrLed server");
    if (!ledClrClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("mowgli_proxy: ClrLed server service not found.");
        return 1;
    }
    // turn all LEDs off
    mowgli::Led ledsrv;
    ledsrv.request.led = 139;
    ledClrClient.call(ledsrv);

    // Mowgli Services
    mowClient = n.serviceClient<std_srvs::SetBool>("mowgli/EnableMowerMotor");

    ROS_INFO("mowgli_blade: Waiting for mowgli/EnableMowerMotor server");
    if (!mowClient.waitForExistence(ros::Duration(60.0, 0.0))) {
        ROS_ERROR("mowgli_blade: EnableMowerMotor server service not found.");
        return 1;
    }

    ros::spin();
    return 0;
}
