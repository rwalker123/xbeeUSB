#!/user/bin/python
import sys
import azure
import socket
 
class EventHubClient(object):
 
  def sendMessage(self,body,partition):
    #eventHubHost = "rewiot-ns.servicebus.windows.net"
 
    # change to use requests module
    #httpclient = _HTTPClient(service_instance=self)
 
    deviceId = "1" # "dev-01"
    sasKeyName = "send"
    sasKeyValue = "nvkLuDJx52RYdxBn+EyN7q9Jr/h5EIXEubRCHQD5/ds="
    serviceNamespace = "rewIoT-ns"
    hubName = "rewiot"

    #serviceNamespace = "iothub-ns-rewiothub-1424-55263ce08e"
    #hubName = "rewIoTHub"
    #sasKeyName = "iothubowner"
    #sasKeyValue = "7ez9wA3VTqt/Fv2oePmFb07Or5t6UV95haXfEDCNm2A="

    serviceBus = azure.servicebus.servicebusservice.ServiceBusService(service_namespace = serviceNamespace, shared_access_key_name = sasKeyName, shared_access_key_value = sasKeyValue)


    #eventHub = serviceBus.get_event_hub("iotevents")

    serviceBus.send_event(hubName, '{ "Temperature":"37.0" }', device_id = deviceId)

    #authentication = ServiceBusSASAuthentication(sasKeyName,sasKeyValue)
 
    #request = HTTPRequest()
    #request.method = "POST"
    #request.host = eventHubHost
    #request.protocol_override = "https"
    #request.path = "/iotevents/publishers/" + partition + "/messages?api-version=2014-05"
    #request.body = body
    #request.headers.append(('Content-Type', 'application/atom+xml;type=entry;charset=utf-8'))
 
    #authentication.sign_request(request, httpclient)
 
    #request.headers.append(('Content-Length', str(len(request.body))))
 
    #status = 0

    #try:
    #    resp = httpclient.perform_request(request)
    #    status = resp.status
    #except HTTPError as ex:
    #    status = ex.status
 
    #return status
 
class EventDataParser(object):
 
  def getMessage(self,payload,sensorId):
    #host = socket.gethostname()
    #body = "{ \"DeviceId\" : \"" + host + "\",\"SensorData\": [ "
 
    msgs = payload.split(",")
    first = True
 
    #for msg in msgs:
    #  sensorType = msg.split(":")[0]
    #  sensorValue = msg.split(":")[1]
    #  if first == True:
    #    first = False
    #  else:
    #    body += ","
 
    # body += "{ \"SensorId\" : \"" + sensorId + "\", \"SensorType\" : \"" + sensorType + "\", \"SensorValue\" : " + sensorValue + " }"
    body += "]}"
 
    #return body
 
