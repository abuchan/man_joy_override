#!/usr/bin/env python

import rospy
from man_joy_override.srv import *
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

from threading import Condition
from Queue import Queue
from collections import OrderedDict

# Roach Imports
import command
import time, sys, os, traceback
import serial
from velociroach import *
import shared_multi as shared

#DEFAULT_ADDRS = ['\x20\x72','\x20\x73']
DEFAULT_ADDRS = ['\x20\x72']
#DEFAULT_ADDRS = ['\x30\x22']

MIN_VEL = 0.5

class RoachBridge():
  def __init__(self, xb, robot_addrs=DEFAULT_ADDRS, robot_offset=0):
    self.xb = xb
    self.robots = [Velociroach(addr,self.xb) for addr in robot_addrs]
    self.robot_offset = robot_offset

    for r in self.robots:
      r.running = False
      r.VERBOSE = False

    self.n_robots = len(self.robots)
    self.states = [[0.0,0.0]]*self.n_robots
    
    rospy.init_node('roach_bridge')
    
    shared.imu_queues = OrderedDict()
    self.imu_pubs = []

    for i in range(self.n_robots):
      shared.imu_queues[self.robots[i].DEST_ADDR_int] = Queue()
      self.imu_pubs.append(rospy.Publisher(
        'robot%d/imu' % (i+self.robot_offset),Imu,queue_size=10
      ))
      rospy.Subscriber(
        'robot%d/cmd_vel' % (i+self.robot_offset),
        Twist, 
        self.curried_callback(i)
      )
      rospy.Service(
        'robot%d/stream_telemetry' % (i+self.robot_offset), 
        StreamTelemetry, 
        self.curried_service(i)
      )

    
    self.rate = rospy.Rate(5.0)

    self.lock = Condition()

    shared.ROBOTS = self.robots
    shared.xb = self.xb

    print 'RoachBridge init complete'

  def command_to_state_ams(self, command):
    lv = 3.0
    la = 2.0
    v = command.linear.x
    a = command.angular.z
    r = lv*v + la*(abs(a) + a)/2
    l = lv*v + la*(abs(a) - a)/2
    return [l,r]

  def command_to_state_bemf(self, command):
    lv = 10.0
    la = 10.0
    v = command.linear.x
    a = command.angular.z
    r = int(lv*v + (la * a))
    l = -int(lv*v - (la * a))
    return [l,r]
  
  def command_to_state_open(self, command):
    lv = 200
    la = 200
    v = command.linear.x
    a = command.angular.z
    r = int(lv*v + la*(abs(a) + a)/2)
    l = -int(lv*v + la*(abs(a) - a)/2)
    return [l,r]
  
  def command_callback(self, command, robot_id):
    #state = self.command_to_state_ams(command)
    #state = self.command_to_state_open(command)
    state = self.command_to_state_bemf(command)
    
    self.lock.acquire()
    self.states[robot_id] = state
    self.lock.release()

  def curried_callback(self, robot_id):
    return lambda m: self.command_callback(m, robot_id)

  def stream_telemetry(self, req, robot_id):
    self.lock.acquire()
    self.robots[robot_id].streamTelemetry(req.stream)
    self.lock.release()
    return StreamTelemetryResponse()

  def curried_service(self, robot_id):
    return lambda m: self.stream_telemetry(m, robot_id)

  def init_ams(self, robot):
    motorgains = [1800,100,100,0,0, 1800,100,100,0,0]
    # Parameters can be passed into object upon construction, as done here.
    simpleAltTripod = GaitConfig(motorgains, rightFreq=2, leftFreq=2)
    simpleAltTripod.phase = PHASE_180_DEG
    # Or set individually, as here
    simpleAltTripod.deltasLeft = [0.25, 0.25, 0.25]
    simpleAltTripod.deltasRight = [0.25, 0.25, 0.25]
    
    START_ROBOT(robot)
    robot.setGait(simpleAltTripod)
    
    time.sleep(0.5)
    
  def set_ams(self, state, robot):
    #robot.setThrustOpenLoop(state[0],state[1],200)
    if state[0] < MIN_VEL and state[1] < MIN_VEL:
      state = [0.0,0.0]
      if robot.running:
        STOP_ROBOT(robot)
    else:
      if not robot.running:
        START_ROBOT(robot)
      else:
        if state[0] < MIN_VEL: state[0] = MIN_VEL
        if state[1] < MIN_VEL: state[1] = MIN_VEL
        SET_VELOCITIES(robot, state[0], state[1], simpleAltTripod)
    
  def init_bemf(self, robot):
    print "Resetting robot at 0x%02X..." % robot.DEST_ADDR_int
    robot.reset()
    time.sleep(0.5)

    # Send robot a WHO_AM_I command, verify communications
    robot.query(retries = 1)

    motorgains = [20000,200,100,0,0,    20000,200,100,0,0]
    #motorgains = [25000,50,0,0,25,    25000,50,0,0,25]

    robot.setMotorGains(motorgains, retries = 1)

  def set_bemf(self, state, robot):
    print 'Setting robot 0x%02X to %s' % (robot.DEST_ADDR_int,state)
    robot.setMotorSpeeds(state[0],state[1])
  
  def init_open(self, robot):
    robot.reset()
    time.sleep(0.5)
    
    # Send robot a WHO_AM_I command, verify communications
    robot.query(retries = 1)

  def set_open(self, state, robot):
    robot.setThrustOpenLoop(state[0],state[1])

  def run(self):
    print 'RoachBridge running'
    
    for robot in self.robots:
      #self.init_ams(robot)
      #self.init_open(robot)
      self.init_bemf(robot)

    print 'Robots started'

    while not rospy.is_shutdown():
      self.lock.acquire()
      for state, robot in zip(self.states, self.robots):
        #self.set_ams(state,robot)
        #self.set_open(state,robot)
        self.set_bemf(state,robot)

      self.lock.release()
      
      for q,p in zip(shared.imu_queues.values(),self.imu_pubs):
        while not q.empty():
          d = q.get()
          i = Imu()
          i.header.seq = d[0]
          i.header.stamp = rospy.Time.now()
          i.angular_velocity.x = d[4]
          i.angular_velocity.y = d[5]
          i.angular_velocity.z = d[6]
          i.linear_acceleration.x = d[1]
          i.linear_acceleration.y = d[2]
          i.linear_acceleration.z = d[3]
          p.publish(i)
      self.rate.sleep()
    
    print 'Stopping robots'
    
    for r in self.robots:
      STOP_ROBOT(r)
    
    print 'RoachBridge exiting...'

def SET_VELOCITIES(r, left, right, gaitcfg):
  print left,right
  gaitcfg.leftFreq = left
  gaitcfg.rightFreq = right
  r.setVelProfile(gaitcfg)

def START_ROBOT(r):
  #r.startTimedRun(10000)
  r.PIDStartMotors()
  r.running = True

def STOP_ROBOT(r):
  #r.startTimedRun(1)
  r.PIDStopMotors()
  r.running = False

if __name__ == '__main__':
  xb = None
  try:
    xb = setupSerial('/dev/ttyUSB0',57600)
  except:
    print 'Failed to set up serial, exiting'

  if xb is not None:
    try:
      rb = RoachBridge(xb)
      rb.run()
    except KeyboardInterrupt:
      print "\nRecieved Ctrl+C, exiting."
    except Exception as args:
      print "\nGeneral exception from main:\n",args,'\n'
      print "\n    ******    TRACEBACK    ******    "
      traceback.print_exc()
      print "    *****************************    \n"
      print "Attempting to exit cleanly..."
    finally:
      xb_safe_exit(xb)
