#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <curses.h>

struct XBeePacket
{
    unsigned short dataLen;
    unsigned char apiId;
    unsigned short sourceAddress;
    unsigned char signalStrength;
    unsigned char options;
    std::string data;
    
    XBeePacket()
    {
        dataLen = 0;
        apiId = 0;
        sourceAddress = 0;
        signalStrength = 0;
        options = 0;
    }
};

class XBeeSerial
{
    private:
        parsePacket();
        
        unsigned char rx_buffer[256];
        
        XBeePacket currentPacket;
        
        int uart0_filestream;
        bool gotStartFrame = false;
        int remainingBytes = -1;
        
        std::queue<XBeePacket> packets;
        
    public:
        XBeeSerial(const std::string& deviceName);
        ~XBeeSerial();
        void receivePacket();
        bool hasPackets();
};

XBeeSerial::XBeeSerial(const std::string& deviceName)
: uart0_filestream(-1)
, gotStartFrame(false)
, remainingBytes(-1)
{
    uart0_filestream = open(deviceName.c_str(), O_RDWR | O_NOCTTY); // | O_NDELAY);

    if (uart0_filestream == -1)
    {
        throw std::exception("Error - Unable to open UART. Ensure it is not in use by another application\n");
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
}

XBeeSerial::~XBeeSerial()
{
   close(uart0_filestream);
}

bool XBeeSerial::hasPackets()
{
    return !packets.empty();
}

XBeePacket XBeeSerial::getPacket()
{
    XBeePacket p = packets.front();
    packets.pop();
    
    return p;
}
void XBeeSerial::receivePacket()
{
    int rx_length = read(uart0_filestream, (void*)rx_buffer, 255);
    if (rx_length < 0)
    {
        throw std::exeception("Error reading from port");
    }
    else if (rx_length > 0)
    {
        //printw("%i bytes read : ", rx_length);
        for(int i = 0; i < rx_length; ++i)
        {
            if (!gotStartFrame && rx_buffer[i] == 0x7e) 
            {
                gotStartFrame = true;
                packetBytesRead = 1;
                currentPacket = XBeePacket();
            }
            // bytes 2/3 are the dataLength
            else if (gotStartFrame && packetBytesRead < 3)
            {
                packetBytesRead++;

                if (packetBytesRead == 2)
                {
                    currentPacket.dataLen = rx_buffer[i] << 8;
                }
                else
                {
                    currentPacket.dataLen |= rx_buffer[i];
                    remainingBytes = dataLen + 1;  // +1 for checksum
                }
            }
            // byte 4 is the API Identifier
            else if (gotStartFrame && packetBytesRead < 4)
            {
                packetBytesRead++;
                remainingBytes--;

                currentPacket.apiId = rx_buffer[i];
            }
            // bytes 5/6 are source address
            else if (gotStartFrame && packetBytesRead < 6)
            {
                packetBytesRead++;
                remainingBytes--;

                if (packetBytesRead == 5)
                {
                    currentPacket.sourceAddress = rx_buffer[i] << 8;
                }
                else
                {
                    currentPacket.sourceAddress |= rx_buffer[i];
                }
            }
            // byte 7 is the signal strength
            else if (gotStartFrame && packetBytesRead < 7)
            {
                packetBytesRead++;
                remainingBytes--;

                currentPacket.signalStrength = rx_buffer[i];
            }
            // byte 8 is the options
            else if (gotStartFrame && packetBytesRead < 8)
            {
                packetBytesRead++;
                remainingBytes--;

                currentPacket.xbeeOptions = rx_buffer[i];
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
                    packetBytesRead = 0;
                    packets.push(currentPacket);
                }			
                else
                {
                    currentPacket.data.append(rx_buffer[i]);
                }
            }
            else
            {
                throw std::exception("Unexpected data: %2x\n", (unsigned)rx_buffer[i]);
            }
        }
    }
}


class XBee 
{
    private:
    
        XBeeSerial serial;
        
    public:
        XBee();
        bool hasPacket()
        {
            serial.receivePacket();
            return serial.hasPackets();
        }
        
        XBeePacket getPacket()
        {
            return serial.getPacket();
        }
};

XBee::XBee()
: serial("/dev/ttyUSB0")
{
}

int main()
{
    XBee xbee;
    
    // ncurses init.
    initscr();
    timeout(0); // getch timeout
    cbreak();

    while (getch() <= 0)
    {
        if (xbee.hasPacket())
        {
            XBeePacket p = xbee.getPacket();
            printw("Got Packet\n");
        }
    }

    endwin();
}