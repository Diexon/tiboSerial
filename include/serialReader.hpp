#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

class serialReader
{
	private:

		int speed , parity;

		char *port;

		int fd;

		int set_interface_attribs (int fdSt, int speedSt, int paritySt)
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
		                printf ("error %d from tcsetattr", errno);
		                return -1;
		        }
		        return 0;
		}

	public:

		serialReader() : speed(0), parity(0), port('\0'), fd(0) { }

		serialReader(int speedIn, int parityIn, char *portIn)
		{
			fd = open (portIn, O_RDWR | O_NOCTTY | O_SYNC);
			speed = speedIn;
			parity = parityIn;
			port = portIn;

			set_interface_attribs (fd, speed, parity);
		}
};
