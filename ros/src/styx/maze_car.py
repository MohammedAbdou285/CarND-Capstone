from SimpleWebSocketServer import SimpleWebSocketServer, WebSocket
import json

class Bridge(object):
        iteration = 0
        def __init__(self):
                self.client = None
        def reg(self, client):
                self.client = client
        def throttle(self,value):
                if self.client is not None:
                        self.client.sendMessage('slam:'+str(value))
                        
bridge = Bridge()
class SimpleEcho(WebSocket):
    def handleMessage(self):
        s = str(self.data)
        print(self.data)
        print(type(self.data))

        my_bytes_value = self.data

        # Decode UTF-8 bytes to Unicode, and convert single quotes 
        # to double quotes to make it valid JSON
        my_json = my_bytes_value.decode('utf8').replace("'", '"')
        data = json.loads(my_json)
        
        x = float(data["telemetry"]["x"])
        y = float(data["telemetry"]["y"])

        if(y > 3):
            self.sendMessage('{slam:{"in_x":0.1,"in_y":0}}')
        else:
            self.sendMessage('{slam:{"in_x":0,"in_y":0.1}}')
    def handleConnected(self):
        bridge.reg(self)
        print("CONNECTED")
        #self.sendMessage('{slam:{}}')
        print(self.address, 'connected')
    def handleClose(self):
        print(self.address, 'closed')

server = SimpleWebSocketServer('127.0.0.1',4567, SimpleEcho)

import threading
import time


t = threading.Thread(target=server.serveforever)
t.start()