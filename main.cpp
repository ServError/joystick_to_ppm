#include <asm/types.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <fstream>
#include <cstring>
#include <cstdlib>
#include <signal.h>
#include <iostream>
#include <unistd.h>
#include <termios.h>
#include "joystick.h"

struct js_event 
{
	__u32 time;     /* event timestamp in milliseconds */
	__s16 value;    /* value */
	__u8 type;      /* event type */
	__u8 number;    /* axis/button number */
};

volatile sig_atomic_t sigint_fired;

#define JS_EVENT_BUTTON         0x01    /* button pressed/released */
#define JS_EVENT_AXIS           0x02    /* joystick moved */
#define JS_EVENT_INIT           0x80    /* initial state of device */

int set_interface_attribs (int fd, int speed, int parity)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		std::cout << "error from tcgetattr " << errno;
		return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
									// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
									// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		std::cout << "error from tcsetattr " << errno;
		return -1;
	}
	return 0;
}

int fd_set_nonblocking(int fd) {
    /* Save the current flags */
    int flags = fcntl(fd, F_GETFL, 0);
    if (flags == -1)
        return 0;

        flags |= O_NONBLOCK;
    return fcntl(fd, F_SETFL, flags) != -1;
}

void sigint_handler(int s){
	sigint_fired = 1;
    //exit(1); 
}

int main()
{
	struct sigaction sigIntHandler;
	std::string const USRJOY = "/usr/local/etc/joystick/";

	std::cout << "opening USB" << std::endl;
	int fdc = open ("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_SYNC);
	set_interface_attribs (fdc, B115200, 0);
	
	ppm_file ppm(fdc);
	//ppm_stream ppm(std::cout);

	sleep(2);
	
	std::cout << "opening joystick" << std::endl;

	joystick joy1(USRJOY + "joy_mappings.ini", USRJOY + "joy_user_trims.ini", ppm);
	joystick joy2(USRJOY + "throt_mappings.ini", USRJOY + "throt_user_trims.ini", ppm);
	joystick joy3(USRJOY + "pedals_mappings.ini", USRJOY + "pedals_user_trims.ini", ppm);
	
	// open joystick
	int fd1 = open("/dev/input/jsjoy", O_RDONLY);
	fd_set_nonblocking(fd1);
	int fd2 = open("/dev/input/jsthrot", O_RDONLY);
	fd_set_nonblocking(fd2);
	int fd3 = open("/dev/input/jspedals", O_RDONLY);
	fd_set_nonblocking(fd3);

	sigIntHandler.sa_handler = sigint_handler;
	sigemptyset(&sigIntHandler.sa_mask);
	sigIntHandler.sa_flags = 0;

	sigaction(SIGINT, &sigIntHandler, NULL);

	js_event ev;

	int readJoy;

	while(sigint_fired == 0)
	{
		for(int i = 0;i > 10; i++) // Reduce polling of SIGINT
		{
			// read from joysticks
			readJoy = read(fd1, &ev, sizeof(ev));
			if(readJoy > 0)
			{
				ev.type = ev.type &~JS_EVENT_INIT;

				if (ev.type == JS_EVENT_AXIS)
				{
					joy1.put_axis(ev.number, (float)ev.value / 32768.0f);
				}
				else if (ev.type == JS_EVENT_BUTTON)
				{
					joy1.put_button(ev.number, ev.value != 0);
				}
				else
				{
					std::cout << "unknown type for joy1 " << (int)ev.type;
				}
			}
			readJoy = read(fd2, &ev, sizeof(ev));
			if(readJoy > 0)
			{
				ev.type = ev.type &~JS_EVENT_INIT;

				if (ev.type == JS_EVENT_AXIS)
				{
					joy2.put_axis(ev.number, (float)ev.value / 32768.0f);
				}
				else if (ev.type == JS_EVENT_BUTTON)
				{
					joy2.put_button(ev.number, ev.value != 0);
				}
				else
				{
					std::cout << "unknown type for joy2 " << (int)ev.type;
				}
			}
			readJoy = read(fd3, &ev, sizeof(ev));
			if(readJoy > 0)
			{
				ev.type = ev.type &~JS_EVENT_INIT;

				if (ev.type == JS_EVENT_AXIS)
				{
					joy3.put_axis(ev.number, (float)ev.value / 32768.0f);
				}
				else if (ev.type == JS_EVENT_BUTTON)
				{
					joy3.put_button(ev.number, ev.value != 0);
				}
				else
				{
					std::cout << "unknown type for joy3 " << (int)ev.type;
				}
			}
		}
	}
	printf("Exit signal caught");
	close(fdc);
	close(fd1);
	close(fd2);
	close(fd3);

	return 0;
}
