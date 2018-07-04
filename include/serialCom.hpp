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
		cc_t block, timeout;
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
		        tty.c_cc[VMIN]  = block;            // read doesn't block
		        tty.c_cc[VTIME] = timeout;            // 0.5 seconds read timeout

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

				return 0;
		}

	public:

		serialCom(speed_t speedIn = B115200, 
			std::string portIn = "/dev/ttyUSB0",
			int parityIn = 0,
			cc_t blockIn = 0,
			cc_t timeoutIn = 1
			):
			speed(speedIn),
			parity(parityIn),
			port(portIn.c_str()),			
			block(blockIn),
			timeout(timeoutIn)
		{
			fd = open (port, O_RDWR| O_NOCTTY);
			//fd = open (port, O_RDWR | O_NOCTTY | O_SYNC);
			if (fd < 0)
			{
				printf ("error %d opening %s: %s", errno, port, strerror (errno));
				return;
			}					
			set_interface_attribs ();
		}

		//! This method is specific for arduino connection. Waiting to boot.
		/*!
		\param loop loops to read.
		\param sucessRead Threshold .
		\return True if read succeeds		
		*/	
		bool waitConnection(int loops, int sucessRead){
			int reads = 0;
			char buff[1];
			printf("Waiting serial to boot...\n");
			for (int n = 0; n < loops; n++){
				reads = reads + read (fd, buff, 1);
				if (reads >= sucessRead) return true;
			}
			return false;
		}

		int readLine(char* buf){
			char inter[100];
			memset(inter,'\0',sizeof inter);
			int stat = read (fd, inter, sizeof inter);

			buf = inter;
			return stat;

		}
};
