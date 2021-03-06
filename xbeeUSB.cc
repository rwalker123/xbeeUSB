//pi@raspberrypi ~/git/xbeeUSB $ g++ -Wall -I/home/pi/git/azure-iot-sdks/c/iothub_client/inc -I/home/pi/git/azure-iot-sdks/c/common/inc xbeeUSB.cc -L/home/pi/git/azure-iot-sdks/c/iothub_client/build/linux -L/home/pi/git/azure-iot-sdks/c/common/build/linux -L/home/pi/git/azure-iot-sdks/c/serializer/build/linux -lncurses -liothub_client -liothub_amqp_transport -lcommon -lpthread -lqpid-proton -lserializer -o xbeeTest

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <curses.h>
#include <string>
#include <queue>
#include <exception>


class TextException : public std::exception
{
private:
    std::string errorText;

public:
    TextException(const char *e) : errorText(e) {}
    virtual ~TextException() _GLIBCXX_USE_NOEXCEPT {}
    const char* what() const _GLIBCXX_USE_NOEXCEPT { return errorText.c_str(); }
};

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
    void initVars();

    unsigned char rx_buffer[256];

    XBeePacket currentPacket;

    int uart0_filestream;
    bool gotStartFrame;
    int remainingBytes;
	int packetBytesRead;
  	unsigned char chksum;

    std::queue<XBeePacket> packets;

public:
    XBeeSerial(const std::string& deviceName);
    ~XBeeSerial();
    void receivePacket();
    bool hasPackets();
	XBeePacket getPacket();
};

void XBeeSerial::initVars()
{
    gotStartFrame = false;
    remainingBytes = -1;
    packetBytesRead = 0;
    chksum = 0;
}

XBeeSerial::XBeeSerial(const std::string& deviceName)
{
    initVars();

    uart0_filestream = open(deviceName.c_str(), O_RDWR | O_NOCTTY); // | O_NDELAY);

    if (uart0_filestream == -1)
    {
        throw TextException("Error - Unable to open UART. Ensure it is not in use by another application");
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
        throw TextException("Error reading from port");
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
                    remainingBytes = currentPacket.dataLen + 1;  // +1 for checksum
                }
            }
            // byte 4 is the API Identifier
            else if (gotStartFrame && packetBytesRead < 4)
            {
                packetBytesRead++;
                remainingBytes--;

        		chksum += rx_buffer[i];

                currentPacket.apiId = rx_buffer[i];
            }
            // bytes 5/6 are source address
            else if (gotStartFrame && packetBytesRead < 6)
            {
                packetBytesRead++;
                remainingBytes--;

		        chksum += rx_buffer[i];

                if (packetBytesRead == 5)
                {
                    currentPacket.sourceAddress = rx_buffer[i] << 8;
                }
                else
                {
                    currentPacket.sourceAddress |= rx_buffer[i];
                }
            }
            // byte 7 is signal strength
            else if (gotStartFrame && packetBytesRead < 7)
            {
            packetBytesRead++;
            remainingBytes--;
    
            chksum += rx_buffer[i];
    
            currentPacket.signalStrength = rx_buffer[i];
            }
            // byte 8 is the options
            else if (gotStartFrame && packetBytesRead < 8)
            {
                packetBytesRead++;
                remainingBytes--;

		        chksum += rx_buffer[i];

                currentPacket.options = rx_buffer[i];
            }
            // remaining bytes are data
            else if (gotStartFrame)
            {
                packetBytesRead++;
                remainingBytes--;

                if (remainingBytes == 0)
                {
                    // last byte is checksum
                    unsigned char recvCheckSum = rx_buffer[i];
                    chksum += rx_buffer[i];
              	    initVars();
                    packets.push(currentPacket);
                }
                else
                {
                    if (rx_buffer[i] == 0x7D) // next byte must be XOR'ed with 0x20
                    {
                        throw TextException("have to handle escaped data");
                    }
                    else
                    {
                        currentPacket.data += rx_buffer[i];
                    }
                }
            }
            else
            {
                //throw TextException("Unexpected data"); //: %2x\n", (unsigned)rx_buffer[i]);
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
: serial(std::string("/dev/ttyUSB0"))
{
}

#include <iothub_client.h>
#include <iothubtransportamqp.h>
#include <iothub_client_ll.h>

DEFINE_ENUM_STRINGS(IOTHUB_CLIENT_CONFIRMATION_RESULT, IOTHUB_CLIENT_CONFIRMATION_RESULT_VALUES);

class AzureIoT
{
private:
    IOTHUB_CLIENT_HANDLE iotHubClientHandle;

    //static IOTHUBMESSAGE_DISPOSITION_RESULT ReceiveMessageCallback(IOTHUB_MESSAGE_HANDLE message, void* userContextCallback)
    //{
    //    const char* buffer;
    //    size_t size;
    //    IoTHubMessage_GetByteArray(message, (const unsigned char**)&buffer, &size);
    //    (void)printf("Received Message with Data: <<<%.*s>>> & Size=%d\r\n", (int)size, buffer, (int)size);
    //    /* Some device specific action code goes here... */
    //    return IOTHUBMESSAGE_ACCEPTED;
    //}
    
    static void SendConfirmationCallback(IOTHUB_CLIENT_CONFIRMATION_RESULT result, void* userContextCallback)
    {
        int messageTrackingId = (intptr_t)userContextCallback;
        (void)printf("Confirmation received for message tracking id = %d with result = %s\r\n", messageTrackingId, ENUM_TO_STRING(IOTHUB_CLIENT_CONFIRMATION_RESULT, result));
        /* Some device specific action code goes here... */
    }

public:
    AzureIoT();
    ~AzureIoT();

    void sendMessage();
};

AzureIoT::AzureIoT()
{
    const char* connectionString = "HostName=rewIoTHub.azure-devices.net;DeviceId=MyFirstDevice;SharedAccessKey=BIX0M0v68y0kwnyrUrzN8tR7OtGsMCmlxR7xmG5NeuU=";

    /* Create IoT Hub Client instance */
    iotHubClientHandle = IoTHubClient_CreateFromConnectionString(connectionString, AMQP_Protocol);
    
    /* Setting Message call back, so we can receive Commands. */
    //IoTHubClient_SetMessageCallback(iotHubClientHandle, ReceiveMessageCallback, &receiveContext);    
}

void AzureIoT::sendMessage()
{
    static unsigned int messageTrackingId = 1;
    char msgText[1024];
    sprintf_s(msgText, sizeof(msgText), "{\"deviceId\":\"MyFirstDevice\",\"data\":%.2f}", rand()%4+2 );
    IOTHUB_MESSAGE_HANDLE messageHandle = IoTHubMessage_CreateFromByteArray((const unsigned char*)msgText, strlen(msgText));

    IoTHubClient_SendEventAsync(iotHubClientHandle, messageHandle, SendConfirmationCallback, (void*)(uintptr_t)messageTrackingId);
    IoTHubMessage_Destroy(messageHandle);
    messageTrackingId++;
}

AzureIoT::~AzureIoT()
{
    /* When everything is done and the app is closing, clean up resources */
    IoTHubClient_Destroy(iotHubClientHandle);    
}

int main()
{
    XBee xbee;
    AzureIoT azureIoT;
    
    // ncurses init.
    initscr();
    timeout(0); // getch timeout
    cbreak();

    while (getch() <= 0)
    {
        if (xbee.hasPacket())
        {
            XBeePacket p = xbee.getPacket();
            printw("Got Packet: dataLen: %d, API Id: %x, Source Address: %d, Signal Strength: %d, Options: %d, data: %s\n", 
	               	p.dataLen, p.apiId, p.sourceAddress, p.signalStrength, p.options, p.data.c_str());
            azureIoT.sendMessage();        
        }
    }

    endwin();
}
