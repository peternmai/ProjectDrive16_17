#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>
#define BUFFER_SIZE BUFSIZ
#include <termios.h>


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

    struct timeval fi,se;

    char first;

    int otherCombination = -1;
    int speed;
    int angle;

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
      }
      else if ((first == 'a'|| first == 'A')) {
        printf("Left\n");
      }
      else if ((first == 's' || first == 'S')) {
        printf("Backward\n");
      }
      else if ((first == 'e' || first == 'E')) {
        printf("Forward\n");
      }
      else {
        speed = 0;
        angle = 0;
        printf("Speed: %d  Angle: %d\n", speed, angle);
      }
    }
    return 0;
}

