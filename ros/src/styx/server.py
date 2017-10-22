#!/usr/bin/env python
from wsserver import get_server, get_clients, on, set_inst
from bridge import Bridge
from conf import conf
import json
wss = get_server('0.0.0.0', 4567)
class StyxServer(object):

    def __init__(self, wss):
        self.wss = wss
        self.bridge = Bridge(conf, self)
        self.dbw_enable = False
        self.clients = get_clients()
	self.steer_data = {}
	self.throttle_data = {}
	self.brake_data = {}
	self.path_data = {}
	self.path_init = False
	self.brake_init = False
	self.throttle_init = False
	self.steer_init = False

    def start_server(self):
        self.wss.serveforever()

    def send(self, topic, data):
        for client in self.clients:
            client.sendMessage(json.dumps({'topic': topic, 'data': data}))

    def setPathData(self, data):
	self.path_data = data
	self.path_init = True

    def setBrakeData(self, data):
	self.brake_data = data
	self.brake_init = True

    def setThrottleData(self, data):
	self.throttle_data = data
	self.throttle_init = True

    def setSteerData(self, data):
	self.steer_data = data
	self.steer_init = True

    @on('telemetry')
    def telemetry(self, data):
        if data["dbw_enable"] != self.dbw_enable:
            self.dbw_enable = data["dbw_enable"]
            self.bridge.publish_dbw_status(self.dbw_enable)
        self.bridge.publish_odometry(data)
	if self.path_init:
	    for client in self.clients:
	        client.sendMessage(json.dumps({'topic': 'drawline', 'data': self.path_data}))
	else:
	    for client in self.clients:
		client.sendMessage(json.dumps({'topic': 'manual', 'data': {} }))
  	if self.throttle_init:
	    for client in self.clients:
		client.sendMessage(json.dumps({'topic': 'throttle', 'data' : self.throttle_data}))
	if self.steer_init:
	    for client in self.clients:
		client.sendMessage(json.dumps({'topic': 'steer', 'data' : self.steer_data}))
	if self.brake_init:
	    for client in self.clients:
		client.sendMessage(json.dumps({'topic': 'brake', 'data' : self.brake_data}))	

    @on('control')
    def control(self, data):
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

print "Listening for Server"
s = StyxServer(wss)
set_inst(s)
s.start_server()
