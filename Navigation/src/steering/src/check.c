#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#define BUFFER_SIZE BUFSIZ
#include <termios.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <turtlesim/Pose.h>


static struct termios old, new;

/* Initialize new terminal i/o settings */
void initTermios(int echo)
{
  tcgetattr(0, &old); /* grab old terminal i/o settings */
  new = old; /* make new settings same as old settings */
  new.c_lflag &= ~ICANON; /* disable buffered i/o */
  new.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
  tcsetattr(0, TCSANOW, &new); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios(void)
{
  tcsetattr(0, TCSANOW, &old);
}

/* Read 1 character - echo defines echo mode */
char getch_(int echo)
{
  char ch;
  initTermios(echo);
  ch = getchar();
  resetTermios();
  return ch;
}

/* Read 1 character without echo */
char getch(void)
{
  return getch_(0);
}

/* Read 1 character with echo */
char getche(void)
{
  return getch_(1);
}

int main(int argc, const char **argv) {

    ros::init(argc, argv, "check");
    ros::NodeHandle check;
    ros::Publisher check_pub = check.advertise<std_msgs::String>("chatter",
    1000);
    geometry_msgs::Twist geo;
    
    struct timeval fi,se;

    char first;

    int otherCombination = -1;

    for (;;) {
      first = getch();
      if( first == 3) {
        printf("Exiting...");
        break;
      }
      if(first == 32) {
        printf("Stop\n");
      }
      else if((first == 'd' || first == 'D')) {
        printf("Right\n");
        geo.linear.x = 0.1;
        geo.linear.y = 0.0;
        geo.linear.z = 0.0;

        geo.angular.x = 0.0;
        geo.angular.y = 0.0;
        geo.angular.z = -0.1;
      }
      else if ((first == 'a'|| first == 'A')) {
        printf("Left\n");
        geo.linear.x = 0.1;
        geo.linear.y = 0.0;
        geo.linear.z = 0.0;

        geo.angular.x = 0.0;
        geo.angular.y = 0.0;
        geo.angular.z = 0.1;

      }
      else if ((first == 's' || first == 'S')) {
        printf("Backward\n");
        geo.linear.x = -2;
        geo.linear.y = 0.0;
        geo.linear.z = 0.0;

        geo.angular.x = 0.0;
        geo.angular.y = 0.0;
        geo.angular.z = 0.0;

      }
      else if ((first == 'e' || first == 'E')) {
        printf("Forward\n");
        geo.linear.x = 1.1;
        geo.linear.y = 0.0;
        geo.linear.z = 0.0;

        geo.angular.x = 0.0;
        geo.angular.y = 0.0;
        geo.angular.z = 0.0;

      }
      else {
        
        geo.linear.x = 0.0;
        geo.linear.y = 0.0;
        geo.linear.z = 0.0;

        geo.angular.x = 0.0;
        geo.angular.y = 0.0;
        geo.angular.z = 0.0;
        
        check_pub.publish(geo);
      }
    }
    return 0;
}

