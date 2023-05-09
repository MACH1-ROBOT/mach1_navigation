#include <geometry_msgs/Twist.h>
#include <mach1_msgs/VelStatus.h>
#include <map>
#include <mutex>
#include <ros/ros.h>

class VelSwitch
{
  public:
    enum class NavMode
    {
        MODE_AUTO,
        MODE_JOY,
        MODE_KEY,
    };
    VelSwitch();
    void joystick_vel_callback(const geometry_msgs::Twist::ConstPtr &vel);
    void keyboard_vel_callback(const geometry_msgs::Twist::ConstPtr &vel);
    void loop();

  private:
    ros::NodeHandle nh_;
    ros::Publisher vel_status_pub;
    ros::Subscriber joystick_vel_sub;
    ros::Subscriber keyboard_vel_sub;
    ros::Rate loop_rate{10};
    std::mutex vel_msg_mutex;
    mach1_msgs::VelStatus vel_msg;

    std::map<NavMode, bool> navModes = {
        {NavMode::MODE_AUTO, false},
        {NavMode::MODE_JOY, false},
        {NavMode::MODE_KEY, false},
    };
};

VelSwitch::VelSwitch()
{
    vel_status_pub = nh_.advertise<mach1_msgs::VelStatus>("/vel_status", 1);
    joystick_vel_sub = nh_.subscribe("/joy_cmd_vel", 10, &VelSwitch::joystick_vel_callback, this);
    keyboard_vel_sub =
        nh_.subscribe("/keyboard_cmd_vel", 10, &VelSwitch::keyboard_vel_callback, this);
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

void VelSwitch::loop()
{
    while (ros::ok())
    {
        {
            std::lock_guard<std::mutex> lock(vel_msg_mutex);
            if (navModes[NavMode::MODE_JOY])
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
                vel_msg.twist_msg.angular.z = 0;
                vel_msg.twist_msg.linear.x = 0;
                vel_msg.teleop_enabled = false;
                vel_msg.nav_type = "NONE";
            }
            vel_status_pub.publish(vel_msg);

        }   // release the lock when lock_guard goes out of scope
        loop_rate.sleep();
        ros::spinOnce();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "vel_switch_node");
    VelSwitch VelSwitch;
    VelSwitch.loop();
    return 0;
}