#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

class serialCom
{
	private:

		speed_t speed;		
		const char *port;
		cc_t block, timeout;
		int fd;
		const int maxLineSize = 4000;

	public:

		serialCom(speed_t speedIn = B115200, 
			std::string portIn = "/dev/ttyUSB0",
			int parityIn = 0,
			cc_t blockIn = 0,
			cc_t timeoutIn = 1
			):
			speed(speedIn),
			port(portIn.c_str()),			
			block(blockIn),
			timeout(timeoutIn)
		{
			fd = open (port, O_RDWR | O_NOCTTY | O_SYNC);
			if (fd < 0)
			{
				printf ("error %d opening %s: %s", errno, port, strerror (errno));
			}					
			
			struct termios tty;
			memset (&tty, 0, sizeof tty);
			if (tcgetattr (fd, &tty) != 0)
			{
					printf ("error %d from tcgetattr", errno);
			}

			cfsetospeed (&tty, speed);
			cfsetispeed (&tty, speed);

			/* 8 bits, no parity, no stop bits */
			tty.c_cflag &= ~PARENB;
			tty.c_cflag &= ~CSTOPB;
			tty.c_cflag &= ~CSIZE;
			tty.c_cflag |= CS8;
			/* no hardware flow control */
			tty.c_cflag &= ~CRTSCTS;
			/* enable receiver, ignore status lines */
			tty.c_cflag |= CREAD | CLOCAL;
			/* disable input/output flow control, disable restart chars */
			tty.c_iflag &= ~(IXON | IXOFF | IXANY);
			/* disable canonical input, disable echo,
			disable visually erase chars,
			disable terminal-generated signals */
			tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
			/* disable output processing */
			tty.c_oflag &= ~OPOST;

			tty.c_cc[VMIN]  = block;            // read doesn't block
			tty.c_cc[VTIME] = timeout;            // seconds read timeout
			
			if (tcsetattr (fd, TCSANOW, &tty) != 0)
			{
				printf ("error %d from tcsetattr", errno);
			}
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
			return read (fd, buf, maxLineSize);
		}
};
