#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <curses.h>

class XBee 
{
};

int main()
{
    int uart0_filestream = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY); // | O_NDELAY);

    if (uart0_filestream == -1)
    {
        printf("Error - Unable to open UART. Ensure it is not in use by another application\n");
	exit(-1);
    } 

    //fcntl(uart0_filestream, F_SETFL, 0); // FNDELAY

    struct termios options;
    tcgetattr(uart0_filestream, &options);

    cfsetispeed(&options, B19200);
    cfsetospeed(&options, B19200);

    // 8N1
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;

    // no ownership of port
    options.c_cflag |= (CLOCAL | CREAD);

    // raw input mode
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // or canonical input (instead of raw). This is line oriented.
    //options.c_lflag |= (ICANON | ECHO | ECHOE);

    // enable parity checking and strip parity bit.
    options.c_iflag |= (INPCK | ISTRIP); 

    // processed output, change newlines into CR-LF
    options.c_oflag |= OPOST;

    // Raw output
    options.c_oflag &= ~OPOST;

    //tcflush(uart0_filestream, TCIFLUSH);

    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0;

    tcsetattr(uart0_filestream, TCSANOW, &options);

    unsigned char rx_buffer[256];

    // ncurses init.
    initscr();
    timeout(0); // getch timeout
    cbreak();

    bool gotStartFrame = false;
    int remainingBytes = -1;
    int packetBytesRead = 0;
    int dataLen = 0;
    int sourceAddress = 0;
    unsigned char apiId = 0;
    unsigned char signalStrength = 0;
    unsigned char xbeeOptions = 0;

    while (getch() <= 0)
    {
        int rx_length = read(uart0_filestream, (void*)rx_buffer, 255);
        if (rx_length < 0)
        {
    	    printw("error\n");
	    refresh();
	    break;
    	}
    	else if (rx_length ==0)
    	{
	    //printw("no data\n");
    	}
	else
    	{
	    //printw("%i bytes read : ", rx_length);
	    for(int i = 0; i < rx_length; ++i)
	    {
		if (!gotStartFrame && rx_buffer[i] == 0x7e) 
		{
		    printw("\nstart msg: ");
                    gotStartFrame = true;
		    packetBytesRead = 1;
              	}
		// bytes 2/3 are the dataLength
		else if (gotStartFrame && packetBytesRead < 3)
		{
		    packetBytesRead++;

		    if (packetBytesRead == 2)
		    {
			dataLen = rx_buffer[i] << 8;
		    }
		    else
		    {
			dataLen |= rx_buffer[i];
			remainingBytes = dataLen + 1;  // +1 for checksum
			printw("data size: %d: ", dataLen);
		    }
		}
		// byte 4 is the API Identifier
		else if (gotStartFrame && packetBytesRead < 4)
		{
		    packetBytesRead++;
		    remainingBytes--;

		    apiId = rx_buffer[i];
		    printw("api-id: %d, ", apiId);
		}
		// bytes 5/6 are source address
		else if (gotStartFrame && packetBytesRead < 6)
		{
		    packetBytesRead++;
		    remainingBytes--;

		    if (packetBytesRead == 5)
		    {
			sourceAddress = rx_buffer[i] << 8;
		    }
		    else
		    {
		        sourceAddress |= rx_buffer[i];
		        printw("sourceAddress: %x, ", sourceAddress);
		    }

		}
		// byte 7 is the signal strength
		else if (gotStartFrame && packetBytesRead < 7)
		{
		    packetBytesRead++;
		    remainingBytes--;

		    signalStrength = rx_buffer[i];
		    printw("signal strength: %2x, ", signalStrength);
		}
		// byte 8 is the options
		else if (gotStartFrame && packetBytesRead < 8)
		{
		    packetBytesRead++;
		    remainingBytes--;

		    xbeeOptions = rx_buffer[i];
		    printw("options: %2x, ", xbeeOptions);
		}
		// remaining bytes are data
		else if (gotStartFrame)
		{
		    packetBytesRead++;
		    remainingBytes--;

		    if (remainingBytes == 0)
		    {
			// last byte is checksum
			gotStartFrame = false;
			remainingBytes = -1;
			dataLen = 0;
			sourceAddress = 0;
			apiId = 0;
			signalStrength = 0;
		    }			
		    else
		    {
		    	printw("%c", (unsigned)rx_buffer[i]);
		    }
		}
		else
		{
	    	    printw("Unexpected data: %2x\n", (unsigned)rx_buffer[i]);
		}
            }
	    refresh();
	}
    }

    close(uart0_filestream);
    endwin();
}