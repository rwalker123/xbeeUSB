from xbee import XBee
import serial
#from azureEventHub import *
import socket
#from proton import *
import uuid

port = serial.Serial("/dev/ttyUSB0", 19200,
                     parity=serial.PARITY_NONE,
                     stopbits=serial.STOPBITS_ONE,
                     bytesize=serial.EIGHTBITS)

print(port.isOpen())

partition = 1

xbee = XBee(port)

while True:
    try:
        response = xbee.wait_read_frame()
        print(response)

        mng = Messenger()
        mng.start()

        msg = Message()

        id = uuid.getnode()
        msg.properties = dict()
        msg.properties[symbol("DeviceId")] = symbol(id)
        msg.properties[symbol("Temp")] = symbol("55")
        msg.properties[symbol("Humidity")] = symbol("100")
        print msg.properties
        
##        msg.address = 'amqps://RootManageSharedAccessKey:kYbxXgt6Q3VDYMYZ50eV+uH5W2XHZLdBpZmo4A0nYbI=@rewiotmsg-ns.servicebus.windows.net/iotevents/publishers/device1'
##        msg.address = 'amqps://RootManageSharedAccessKey:FM7PPefw3sVOxePp2CVikx%2FLx8IZ%2FcDHQhKnQRAtYhY=@rewiot-ns.servicebus.windows.net/rewiot/partitions/' + str(partition)
        msg.address = 'amqps://Send:TMWv%2F3ziVgyyrCUJSc6Mxc5DSUi9+da%2Fi+ykoUpSEh8=@rewiot-ns.servicebus.windows.net/rewiot/partitions/' + str(partition)
        msg.body = unicode('this is the body')

        partition = partition + 1
        if partition > 14:
            partition = 1
            
        mng.put(msg)
        mng.send()
        mng.stop()
        
##        hubClient = EventHubClient()
##        parser = EventDataParser()
##        hostname = socket.gethostname()
##        sensor = 'TestDevice'
##         
##        body = parser.getMessage('temperature:1,humidity:1',sensor)
##        hubStatus = hubClient.sendMessage(body,hostname)
##        ## return the HTTP status to the caller
##        print hubStatus

    except KeyboardInterrupt:
        break

port.close()

#    response=port.read(1)
#    print(hex(ord(response)))
