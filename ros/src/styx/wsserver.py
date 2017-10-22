from SimpleWebSocketServer import SimpleWebSocketServer, WebSocket
from functools import wraps

import json

clients = []
routes = {}

def on(topic):
    def decorator(func):
        @wraps(func)
        def wrapper(*args, **kwargs):
            return func(*args, **kwargs)
        routes[topic] = wrapper
        return wrapper
    return decorator

def set_inst(i):
    global inst
    inst = i

class Client(WebSocket):
    def handleMessage(self):
        data = json.loads(self.data.decode('utf8'))
        topic = data.keys()[0]
        if topic in routes:
            routes[topic](inst, data[topic])
        #self.sendMessage(json.dumps({
        #    "topic": "throttle",
        #    "data": {
        #        "throttle": 0.1
        #    }
        #}))

    def handleConnected(self):
        clients.append(self)
        print self.address, 'connected'

    def handleClose(self):
        clients.remove(self)
        print self.address, 'closed'



def get_server(host, port):
    return SimpleWebSocketServer(host, port, Client)

def get_clients():
    return clients

