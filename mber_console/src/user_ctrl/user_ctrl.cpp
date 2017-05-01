#include "std_msgs/String.h"
#include <sstream>
#include <ros/ros.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#define KEY_CODE_R 0x43
#define KEY_CODE_L 0x44
#define KEY_CODE_U 0x41
#define KEY_CODE_D 0x42

class TeleopMBER {
  public:
    TeleopMBER();
    void keyLoop();
  private:
    ros::NodeHandle nh_;
    ros::Publisher mber_motor_ctrl;
};

TeleopMBER::TeleopMBER () {
  mber_motor_ctrl = nh_.advertise<std_msgs::String>("rc_motor_ctrl", 1);
}

int kfd = 0;
struct termios cooked;
struct termios raw;
/*
void quit(int sig) {
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}
*/
int main(int argc, char **argv) {
  ros::init(argc, argv, "teleop_mconsole");
  TeleopMBER teleopMBER;

  //signal(SIGINT, quit);
  teleopMBER.keyLoop();

  return(0);
}

void TeleopMBER::keyLoop(){
  char c;
  bool dirty = false;

  /* get the console in raw mode */
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);

  /* Setting a new line, then end of file */
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the MBER Bot.");

  for(;;) {
    std_msgs::String motor_msg;
    std::stringstream ss;

    /* get the next event from the keyboard */
    if(read(kfd, &c, 1) < 0) {
      perror("read():");
      exit(-1);
    }

    ROS_DEBUG("value: 0x%02X\n", c);

    switch(c) {
      case KEY_CODE_L:
        ROS_DEBUG("LEFT");
        ss << "L";
        dirty = true;
        break;
      case KEY_CODE_R:
        ROS_DEBUG("RIGHT");
        ss << "R";
        dirty = true;
        break;
      case KEY_CODE_U:
        ROS_DEBUG("UP");
        ss << "U";
        dirty = true;
        break;
      case KEY_CODE_D:
        ROS_DEBUG("DOWN");
        ss << "D";
        dirty = true;
        break;
    }
    if(dirty ==true) {
      motor_msg.data = ss.str();
      ROS_INFO("MBER CONSOLE: %s", motor_msg.data.c_str());
      mber_motor_ctrl.publish(motor_msg);    
      dirty=false;
    }
  }


  return;
}