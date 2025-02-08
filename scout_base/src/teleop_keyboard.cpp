#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "scout_msgs/Scout_carrier.h"
#include <iostream>
#include <map>
#include <termios.h>
#include <unistd.h>

#define SCOUT_MAX_LIN_VEL 0.22
#define SCOUT_MAX_ANG_VEL 2.84

#define LIN_VEL_STEP_SIZE 0.01
#define ANG_VEL_STEP_SIZE 0.1

std::string Scout_MODEL = "scout";


int getch() {
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}


float constrain(float input, float low, float high) {
    if (input < low) return low;
    if (input > high) return high;
    return input;
}

float check_linear_limit_velocity(float velocity) {
    if (Scout_MODEL == "scout") {
        return constrain(velocity, -SCOUT_MAX_LIN_VEL, SCOUT_MAX_LIN_VEL);
    }
}

float check_angular_limit_velocity(float velocity) {
    if (Scout_MODEL == "scout") {
        return constrain(velocity, -SCOUT_MAX_ANG_VEL, SCOUT_MAX_ANG_VEL);
    }
}

void printInstructions() {
    std::cout << "\nControl Your TurtleBot3!\n"
              << "---------------------------\n"
              << "Moving around:\n"
              << "        w\n"
              << "   a    s    d\n"
              << "        x\n\n"
              << "w/x : increase/decrease linear velocity\n"
              << "a/d : increase/decrease angular velocity\n"
              << "space, s : stop\n"
              << "CTRL-C to quit\n\n";
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "teleop_keyboard");
    ros::NodeHandle nh;
    
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
    ros::Publisher carrier_pub = nh.advertise<scout_msgs::Scout_carrier>("test", 1);
    
    float target_linear_velocity = 0.0;
    float target_angular_velocity = 0.0;
    int key;
    int carrier_mode = 0;

    printInstructions();

    while (ros::ok()) {
        key = getch();
        
        if (key == 'w') {
            target_linear_velocity = check_linear_limit_velocity(target_linear_velocity + LIN_VEL_STEP_SIZE);
        } else if (key == 'x') {
            target_linear_velocity = check_linear_limit_velocity(target_linear_velocity - LIN_VEL_STEP_SIZE);
        } else if (key == 'a') {
            target_angular_velocity = check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE);
        } else if (key == 'd') {
            target_angular_velocity = check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE);
        } else if (key == ' ' || key == 's') {
            target_linear_velocity = 0.0;
            target_angular_velocity = 0.0;
        } else if (key == 't'){
            carrier_mode = 1;
            scout_msgs::Scout_carrier command;
            command.mode = carrier_mode;
            carrier_pub.publish(command);
        } else if (key == 'y'){
            carrier_mode = 2;
            scout_msgs::Scout_carrier command;
            command.mode = carrier_mode;
            carrier_pub.publish(command);
        }  //t = 올려져있는 상태, y = 내려져있는상태
        else if (key == '\x03') {  // CTRL+C
            std::cout << "\n\n                 .     .\n              .  |\\-^-/|  .    \n             /| } O.=.O { |\\\n\n                 CH3EERS\n\n";
            break;
        }

        // 속도 출력
        std::cout << "\rCurrently: linear velocity " << target_linear_velocity
                  << "\t      angular velocity " << target_angular_velocity
                  << "\t      carrier state " << carrier_mode 
                  << "   " << std::flush;

        geometry_msgs::Twist twist;
        twist.linear.x = target_linear_velocity;
        twist.angular.z = target_angular_velocity;
        pub.publish(twist);

        ros::spinOnce();
    }

    geometry_msgs::Twist twist;
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    pub.publish(twist);

    return 0;
}
