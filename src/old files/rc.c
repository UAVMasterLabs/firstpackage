/*
 * Main.cpp
 *
 *  Created on: 9-Jan-2009
 *      Author: root
 */


/////////////////////////////////////////////////
// Serial port interface program               //
/////////////////////////////////////////////////

#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions
#include <errno.h> // Error number definitions
#include <termios.h> // POSIX terminal control definitionss
#include <time.h>   // time calls
//#include <iostream>

void send_cmd(int, int);

int fd;
int thr_value=0;
int yaw_value=63;
int pitch_value=63;
int roll_value=63;
int mode_value=127;

char getch() {
        char buf = 0;
        struct termios old = {0};
        if (tcgetattr(0, &old) < 0)
                perror("tcsetattr()");
        old.c_lflag &= ~ICANON;
        old.c_lflag &= ~ECHO;
        old.c_cc[VMIN] = 1;
        old.c_cc[VTIME] = 0;
        if (tcsetattr(0, TCSANOW, &old) < 0)
                perror("tcsetattr ICANON");
        if (read(0, &buf, 1) < 0)
                perror ("read()");
        old.c_lflag |= ICANON;
        old.c_lflag |= ECHO;
        if (tcsetattr(0, TCSADRAIN, &old) < 0)
                perror ("tcsetattr ~ICANON");
        return (buf);
}

int open_port(void)
{
	int fd; // file description for the serial port
	
	fd = open("/dev/ttyUSB0", O_RDWR| O_NONBLOCK | O_NDELAY);
	
	if(fd == -1) // if open is unsucessful
	{
		//perror("open_port: Unable to open /dev/ttyS0 - ");
		printf("open_port: Unable to open /dev/ttyUSB0. \n");
		//return 0;
	}

	if ( fd < 0 )
    	{
        	//cout << "Error " << errno << " opening " << "/dev/ttyUSB0" << ": " << strerror (errno) 			<< endl;
        	perror("USB ");
    	}

	else
	{
		fcntl(fd, F_SETFL, 0);
		printf("port is open.\n");
	}
	
	return(fd);
} //open_port

int configure_port(int fd)      // configure the port
{
	struct termios port_settings;      // structure to store the port settings in

	cfsetispeed(&port_settings, B57600);    // set baud rates
	cfsetospeed(&port_settings, B57600);

	port_settings.c_cflag &= ~PARENB;    // set no parity, stop bits, data bits
	port_settings.c_cflag &= ~CSTOPB;
	port_settings.c_cflag &= ~CSIZE;
	port_settings.c_cflag |= CS8;
	
	tcsetattr(fd, TCSANOW, &port_settings);    // apply the settings to the port
	return(fd);

} //configure_port

int main(void)
{ 
	fd = open_port();
	configure_port(fd);
	char c;
	//while(!kbhit()); // wait for input
	do
	{	
		//c=toupper(getch()); 
c = getch();
		switch(c)
		{
			case 'Q':
			if(thr_value < 127) thr_value++;
			send_cmd(0, thr_value);
			printf("throttle=%u\n",thr_value);
			break;
			case 'A':
			if(thr_value > 0) thr_value--;
			send_cmd(0, thr_value);
			printf("throttle=%u\n",thr_value);
			break;
			case 'W':
			if(yaw_value < 127) yaw_value++;
			send_cmd(1, yaw_value);	
			printf("yaw=%u\n",yaw_value);
			break;
			case 'S':
			if(yaw_value > 0) yaw_value--;
			send_cmd(1, yaw_value);	
			printf("yaw=%u\n",yaw_value);
			break;
			case 'E':
			if(pitch_value < 127) pitch_value++;
			send_cmd(2, pitch_value);	
			printf("pitch=%u\n",pitch_value);
			break;
			case 'D':
			if(pitch_value > 0) pitch_value--;
			send_cmd(2, pitch_value);
			printf("pitch=%u\n",pitch_value);
			break;
			case 'R':
			if(roll_value < 127) roll_value++;
			send_cmd(3, roll_value);
			printf("roll=%u\n",roll_value);
			break;
			case 'F':
			if(roll_value > 0) roll_value--;
			send_cmd(3, roll_value);
			printf("roll=%u\n",roll_value);
			break;
			case 'T':
			mode_value=127;
			send_cmd(4, mode_value);
			printf("mode=%u\n",mode_value);
			break;
			case 'G':
			mode_value=0;
			send_cmd(4, mode_value);
			printf("mode=%u\n",mode_value);
		}
	}while(c != 27);	

	return(0);
	
} //main


// cmd=0-> throttle ...
void send_cmd(int cmd, int value)
{
	unsigned char send_bytes[4];
	// header byte
	send_bytes[0]=0x81;
	// throttle=0x82, yaw=0x83, pitch=0x84, roll=0x85, mode=0x86;
	send_bytes[1]=0x82 + cmd;
	// value bayte	
	send_bytes[2]=value;
	//CRC byte
	send_bytes[3]=send_bytes[0] ^ send_bytes[1] ^ send_bytes[2]; 
	write(fd, send_bytes,4);
	printf("done\n");
}



