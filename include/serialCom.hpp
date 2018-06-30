#include <stdio.h>
#include <string>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

class serialCom
{
	private:

		speed_t speed;
		
		int parity;

		const char *port;

		int fd;

		int set_interface_attribs (void)
		{
		        struct termios tty;
		        memset (&tty, 0, sizeof tty);
		        if (tcgetattr (fd, &tty) != 0)
		        {
		                printf ("error %d from tcgetattr", errno);
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
		        tty.c_cc[VTIME] = 1;            // 0.5 seconds read timeout

		        tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

		        tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
		                                        // enable reading
		        tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
		        tty.c_cflag |= parity;
		        tty.c_cflag &= ~CSTOPB;
		        tty.c_cflag &= ~CRTSCTS;

		        if (tcsetattr (fd, TCSANOW, &tty) != 0)
		        {
		                printf ("error %d from tcsetattr", errno);
		                return -1;
		        }

				//Wait for communication to be alive
				char tempMsg[1]; 
				for (int wait = 0; wait < 1000; wait++){
					if(read (fd, tempMsg, 1)) return 0;
				}
				
		        return -1;
		}

	public:

		serialCom(speed_t speedIn = B115200, 
			std::string portIn = "/dev/ttyUSB0",
			int parityIn = 0)
		{
			parity = parityIn;
			port = portIn.c_str();
			fd = open (port, O_RDWR| O_NOCTTY);
			//fd = open (port, O_RDWR | O_NOCTTY | O_SYNC);
			if (fd < 0)
			{
					printf ("error %d opening %s: %s", errno, portIn.c_str(), strerror (errno));
					return;
			}
			speed = speedIn;		

			set_interface_attribs ();
		}

		int readLine(char* buf){
			char inter[100];
			memset(inter,'\0',sizeof inter);
			int stat = read (fd, inter, sizeof inter);

			buf = inter;
			return stat;

		}
};
