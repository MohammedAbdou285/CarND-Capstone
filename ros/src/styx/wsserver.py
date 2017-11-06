from SimpleWebSocketServer import SimpleWebSocketServer, WebSocket
from functools import wraps

import json

clients = []
routes = {}

def on(topic):
    def decorator(func):
        routes[topic] = func
        @wraps(func)
        def wrapper(*args, **kwargs):
            return func(*args, **kwargs)
        return wrapper
    return decorator


class Client(WebSocket):
    def handleMessage(self):
        data = json.loads(self.data.decode('utf8'))
        topic = data.keys()[0]
        print data
        if topic in routes:
            print topic
            print data[topic]
            routes[topic](data[topic])
        self.sendMessage(json.dumps({
            "topic": "throttle",
            "data": {
                "throttle": 0.1
            }
        }))

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

s = SimpleWebSocketServer('127.0.0.1', 4567, Client)
s.serveforever()