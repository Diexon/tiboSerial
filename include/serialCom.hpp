#include <stdio.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

static const int STRING_SIZE = 100;

class serialCom
{
	private:

		// Serial connection parameters
		static int fd;
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
		static const int loops = 1000;
		static const int sucessRead = 6;

		// Serial control		
		static bool serialActive;
		static const int imgValid = 5; // Size of image name to be valid
		static const int emptyRead = 5; // Limit for empty reads
		static constexpr const char* stopMsg = "stop";
		static char currentImg[];

	public:

		static bool changeImg;		
		static bool stopSerial;				

		bool init(int speedIn = 115200, 
			std::string port = "/dev/ttyUSB0",
			cc_t block = 0,
			cc_t timeout = 1
			){	
			speed_t speed;

			auto it = baudRates.find(speedIn);

			if (it != baudRates.end())
			{
				speed = it->second;
			}else
			{	
				printf ("error: no valid baud rate");
				return 1;
			}

			fd = open (port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
			if (fd < 0)
			{
				printf ("error %d opening %s: %s", errno, port.c_str(), strerror (errno));
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
			// Set activation to true if all steps were successful
			serialActive = true;

			return 0;

		}

		static void checkNextImg(){
			if(serialActive) return; 
			char nextImg[STRING_SIZE];
			static int emptyReadCnt = 0;
			if(readLine(nextImg) <= 0){
				changeImg = false;
				emptyReadCnt ++;
				if(emptyReadCnt > emptyRead){
					changeImg = true;
					stopSerial = true;
				}
				else{
					// Do nothing
					return;
				}
			}
			if (strcmp(nextImg, currentImg) == 0){
				changeImg = false;
				stopSerial = false;
				emptyReadCnt = 0;				
			}
			else if (strcmp(nextImg, stopMsg) == 0){				
				changeImg = true;
				stopSerial = true;
				emptyReadCnt = 0;
			}
			else{
				strcpy(currentImg, nextImg);
			}
			
		}

		char* getImg(){
			return currentImg; 
		}

		bool getSerialActive(){
			return serialActive;
		}

	private:
		//! This method is specific for arduino connection. Waiting to boot.
		/*!
		\param loop loops to read.
		\param sucessRead Threshold .
		\return True if read succeeds		
		*/	
		bool waitConnection(){
			int reads = 0;
			char buff[STRING_SIZE];
			printf("Waiting serial to boot...\n");
			for (int n = 0; n < loops; n++){
				reads = readLine(buff);
				tcflush(fd, TCIFLUSH);
				if (reads > imgValid) {
					strcpy(currentImg, buff); // Store first image received
					return true;
				}
			}
			return false;
		}
		//! Read one line present in the serial bus
		/*!
		\param buf Buffer to fill with serial line
		\return Read size of the string	
		*/	
		static int readLine(char* buf){
			int stat = read (fd, buf, maxLineSize);
			tcflush(fd, TCIFLUSH);
			if (stat <= imgValid) {
    			/* No content or content not valid */
				return 0;
 			}
			else{
				buf[stat - 1] = '\0'; //erase the new line
			}
			return stat;
		}
};

serialCom srCom;
bool serialCom::serialActive{false};
bool serialCom::changeImg{false};		
bool serialCom::stopSerial{false};		
char serialCom::currentImg[STRING_SIZE];
int serialCom::fd{0};
