#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

class serialCom
{
	private:

		// Serial connection parameters
		speed_t speed;		
		const char *port;
		cc_t block, timeout;
		int fd;
		static const int maxLineSize = 4000;

		std::map<int, int> baudRates{{300, B300},
									{600, B600},
									{1200, B1200},
									{2400, B2400},
									{4800, B4800},
									{9600, B9600},
									{19200, B19200},
									{38400, B38400},
									{57600, B57600},
									{115200, B115200}};

		// Arduino booting parameters
		static const int loops = 1000, sucessRead = 6;

		// Serial control
		bool serialActive;
		char * currentImg;

	public:

		serialCom():serialActive(false){}

		bool init(int speedIn = 115200, 
			std::string portIn = "/dev/ttyUSB0",
			int parityIn = 0,
			cc_t blockIn = 0,
			cc_t timeoutIn = 1
			)
		{									
			auto it = baudRates.find(speedIn);

			if (it != baudRates.end())
			{
				speed = it->second;
			}else
			{	
				printf ("error: no valid baud rate");
				return 1;
			}
			
			port = portIn.c_str();
			block = blockIn;
			timeout = timeoutIn;

			fd = open (port, O_RDWR | O_NOCTTY | O_SYNC);
			if (fd < 0)
			{
				printf ("error %d opening %s: %s", errno, port, strerror (errno));
				return 1;
			}					
			
			struct termios tty;
			memset (&tty, 0, sizeof tty);
			if (tcgetattr (fd, &tty) != 0)
			{
				printf ("error %d from tcgetattr", errno);
				return 1;
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
			/* enable canonical input*/
			tty.c_lflag &= ICANON;
			/*disable echo, disable visually erase chars,
			disable terminal-generated signals */
			tty.c_lflag &= ~(ECHO | ECHOE | ISIG);
			/* disable output processing */
			tty.c_oflag &= ~OPOST;

			tty.c_cc[VMIN]  = block;            // read doesn't block
			tty.c_cc[VTIME] = timeout;            // seconds read timeout
			// For canonical
			tty.c_cc[VEOL] = '\n';

			if (tcsetattr (fd, TCSANOW, &tty) != 0)
			{
				printf ("error %d from tcsetattr", errno);
				return 1;
			}

			// Wait booting after initializing
			if (!waitConnection()){
				fprintf(stderr, "error: No boot from Arduino reached.\n");
				return 1;
			}
/*
			// Read current image to display
			if (readLine(currentImg) <= 0)
			{
				fprintf(stderr, "error: Not able to read line from serial.\n");
				return 1;
			}
*/
			return 0;

		}

		char * getImg(){
			return currentImg; 
		}

		//! This method is specific for arduino connection. Waiting to boot.
		/*!
		\param loop loops to read.
		\param sucessRead Threshold .
		\return True if read succeeds		
		*/	
		bool waitConnection(){
			int reads = 0;
			char buff[1];
			printf("Waiting serial to boot...\n");
			for (int n = 0; n < loops; n++){
				reads = reads + read (fd, buff, 1);
				if (reads >= sucessRead) return true;
			}
			return false;
		}
		//! Read one line present in the serial bus
		/*!
		\param buf Buffer to fill with serial line
		\return Read status		
		*/	
		int readLine(char* buf){
			int stat = read (fd, buf, maxLineSize);
			if (stat < 0) {
    			/* No content */
 			}
			else{
				buf[stat - 1] = '\0'; //erase the new line
			}
			return stat;
		}
};
