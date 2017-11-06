#!/usr/bin/env python
from wsserver import get_server, get_clients, on
from bridge import Bridge
from conf import conf
import json
wss = get_server('127.0.0.1', 4567)

class StyxServer(object):

    def __init__(self, wss):
        self.wss = wss
        self.bridge = Bridge(conf, self)
        self.dbw_enable = False
        self.clients = get_clients()

    def start_server(self):
        self.wss.serveforever()

    def send(self, topic, data):
        for client in self.clients:
            client.sendMessage(json.dumps({'topic': topic, 'data': data}))

    @on('telemetry')
    def telemetry(self, data):
        print data
        return
        if data["dbw_enable"] != self.dbw_enable:
            self.dbw_enable = data["dbw_enable"]
            self.bridge.set_connected(True)
            self.bridge.publish_dbw_status(self.dbw_enable)
        self.bridge.publish_odometry(data)

    @on('control')
    def control(self, data):
        print data
        return
        self.bridge.publish_controls(data)

    @on('obstacle')
    def obstacle(self, data):
        print data
        return
        self.bridge.publish_obstacles(data)

    @on('lidar')
    def obstacle(self, data):
        print data
        return
        self.bridge.publish_lidar(data)

    @on('trafficlights')
    def trafficlights(self, data):
        print data
        return
        self.bridge.publish_traffic(data)

    @on('image')
    def image(self, data):
        return
        self.bridge.publish_camera(data)


s = StyxServer(wss)
s.start_server()
