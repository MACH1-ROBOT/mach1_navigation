#include <algorithm>
#include <geometry_msgs/Twist.h>
#include <mach1_msgs/Collision.h>
#include <mach1_msgs/VelCmd.h>
#include <map>
#include <memory>
#include <mutex>
#include <ros/ros.h>

class VelSwitch
{
  public:
    enum class NavMode
    {
        MODE_0,
        MODE_AUTO,
        MODE_JOY,
        MODE_KEY,
    };
    VelSwitch();
    void joystick_vel_callback(const geometry_msgs::Twist::ConstPtr &vel);
    void keyboard_vel_callback(const geometry_msgs::Twist::ConstPtr &vel);
    void rear_collision_callback(const mach1_msgs::Collision::ConstPtr &data);
    void loop();

  private:
    ros::NodeHandle nh_;
    ros::Publisher vel_status_pub;
    ros::Subscriber joystick_vel_sub;
    ros::Subscriber keyboard_vel_sub;
    ros::Subscriber rear_collision_sub;
    ros::Rate loop_rate{20};
    std::mutex vel_msg_mutex;
    mach1_msgs::VelCmd vel_msg;

    std::map<NavMode, bool> navModes = {
        {NavMode::MODE_0, false},
        {NavMode::MODE_AUTO, false},
        {NavMode::MODE_JOY, false},
        {NavMode::MODE_KEY, false},
    };
};

VelSwitch::VelSwitch()
{
    vel_status_pub = nh_.advertise<mach1_msgs::VelCmd>("/vel_status", 1);
    joystick_vel_sub = nh_.subscribe("/joy_cmd_vel", 10, &VelSwitch::joystick_vel_callback, this);
    keyboard_vel_sub =
        nh_.subscribe("/keyboard_cmd_vel", 10, &VelSwitch::keyboard_vel_callback, this);
    rear_collision_sub =
        nh_.subscribe("/rear_collision", 10, &VelSwitch::rear_collision_callback, this);
};

void VelSwitch::joystick_vel_callback(const geometry_msgs::Twist::ConstPtr &vel)
{
    std::lock_guard<std::mutex> lock(vel_msg_mutex);
    vel_msg.twist_msg = *vel;
    vel_msg.teleop_enabled = true;
    navModes[NavMode::MODE_JOY] = true;
};   // release the lock when lock_guard goes out of scope

void VelSwitch::keyboard_vel_callback(const geometry_msgs::Twist::ConstPtr &vel)
{
    std::lock_guard<std::mutex> lock(vel_msg_mutex);
    vel_msg.twist_msg = *vel;
    vel_msg.teleop_enabled = true;
    navModes[NavMode::MODE_KEY] = true;
};   // release the lock when lock_guard goes out of scope

void VelSwitch::rear_collision_callback(const mach1_msgs::Collision::ConstPtr &data)
{
    if (data->distance < data->min_collision_dist)
    {
        std::lock_guard<std::mutex> lock(vel_msg_mutex);
        vel_msg.twist_msg.linear.x = 0;
        vel_msg.twist_msg.angular.z = 0;
        navModes[NavMode::MODE_0] = true;
    }   // release the lock when lock_guard goes out of scope
};

void VelSwitch::loop()
{
    while (ros::ok())
    {
        {
            std::lock_guard<std::mutex> lock(vel_msg_mutex);
            if (navModes[NavMode::MODE_0])
            {
                vel_msg.nav_type = "OFF";
                navModes[NavMode::MODE_0] = false;
            }
            else if (navModes[NavMode::MODE_JOY])
            {
                vel_msg.nav_type = "JOYSTICK";
                navModes[NavMode::MODE_JOY] = false;
            }
            else if (navModes[NavMode::MODE_KEY])
            {
                vel_msg.nav_type = "KEYBOARD";
                navModes[NavMode::MODE_KEY] = false;
            }
            else if (navModes[NavMode::MODE_AUTO])
            {
                vel_msg.nav_type = "AUTONOMOUS";
                navModes[NavMode::MODE_AUTO] = false;
            }
            else
            {
                vel_msg.nav_type = "NONE";
                vel_msg.teleop_enabled = false;
            }
        }   // release the lock when lock_guard goes out of scope
        loop_rate.sleep();
        ros::spinOnce();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vel_mux_node");
    VelSwitch VelSwitch;
    VelSwitch.loop();
    return 0;
}