#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <string.h>

#include "ros/ros.h"

#include "controller.h"
#include "locator.h"

void changemode(int dir)
{
  static struct termios oldt, newt;
  if ( dir == 1 )
  {
    tcgetattr( STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~( ICANON | ECHO );
    tcsetattr( STDIN_FILENO, TCSANOW, &newt);
  }
  else
    tcsetattr( STDIN_FILENO, TCSANOW, &oldt);
}

int kbhit (void)
{
  struct timeval tv;
  fd_set rdfs;

  tv.tv_sec = 0;
  tv.tv_usec = 0;

  FD_ZERO(&rdfs);
  FD_SET (STDIN_FILENO, &rdfs);

  select(STDIN_FILENO+1, &rdfs, NULL, NULL, &tv);
  return FD_ISSET(STDIN_FILENO, &rdfs);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vis_pose_test");

  ros::NodeHandle n;

  Controller controller(&n);

  Locator locator(&n, &controller);

  changemode(1);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    // get the next event from the keyboard
    while(kbhit())
    {
      int c = getchar();

      switch (c)
      {
        case 'h':
          printf("Help - Place Holder for keyboard input\n");
          break;
      }
    }

    ros::spinOnce();

    loop_rate.sleep();
  }

  changemode(0);

  return 0;
}

