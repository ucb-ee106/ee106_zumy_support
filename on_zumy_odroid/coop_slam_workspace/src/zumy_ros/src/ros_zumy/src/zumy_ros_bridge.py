#!/usr/bin/python

import rospy

from geometry_msgs.msg import Twist
from threading import Condition
from zumy import Zumy
from std_msgs.msg import String,Header,Int32,Float32,Bool
from sensor_msgs.msg import Imu

import socket,time

class ZumyROS:	
  def __init__(self):
    self.zumy = Zumy()
    rospy.init_node('zumy_ros')
    self.cmd = (0,0)
    rospy.Subscriber('cmd_vel', Twist, self.cmd_callback,queue_size=1)
    rospy.Subscriber('enable', Bool, self.enable_callback,queue_size=1)

    rospy.Subscriber('/base_computer',String,self.watchdog_callback,queue_size=1) #/base_computer topic, the global watchdog.  May want to investigate what happens when there moer than one computer and more than one zumy

    self.lock = Condition()
    self.rate = rospy.Rate(30.0)
    self.name = socket.gethostname()


    if rospy.has_param("~timeout"):
        self.timeout = rospy.get_param('~timeout') #private value
    else:
      self.timeout = 2 #Length of watchdog timer in seconds, defaults to 2 sec.
    
    #publishers
    self.heartBeat = rospy.Publisher('heartBeat', String, queue_size=5)
    self.imu_pub = rospy.Publisher('imu', Imu, queue_size = 1)
    self.r_enc_pub = rospy.Publisher('r_enc', Int32, queue_size = 5)
    self.l_enc_pub = rospy.Publisher('l_enc', Int32, queue_size = 5)
    self.imu_count = 0

    self.batt_pub = rospy.Publisher('Batt',Float32,queue_size = 5)

    self.last_message_at = time.time()
    self.watchdog = True 

  def cmd_callback(self, msg):
    lv = 0.6
    la = 0.4
    v = msg.linear.x
    a = msg.angular.z
    r = lv*v + la*a
    l = lv*v - la*a
    self.lock.acquire()
    self.cmd = (l,r)
    self.lock.release()

  def enable_callback(self,msg):
    #enable or disable myself based on the content of the message.   
    if msg.data and self.watchdog:
      self.zumy.enable()
    else:
      self.zumy.disable()
    #do NOT update the watchdog, since the watchdog should do it itself.
    #If i'm told to go but the host's watchdog is down, something's very wrong, and i won't be doing much

  def watchdog_callback(self,msg):
    #update the last time i got a messag!
    self.last_message_at = time.time()
    self.watchdog = True



  def run(self):
    while not rospy.is_shutdown():
      self.lock.acquire()
      self.zumy.cmd(*self.cmd)
      imu_data = self.zumy.read_imu()
      enc_data = self.zumy.read_enc()
      self.lock.release()
      
      imu_msg = Imu()
      imu_msg.header = Header(self.imu_count,rospy.Time.now(),self.name)
      imu_msg.linear_acceleration.x = 9.81 * imu_data[0]
      imu_msg.linear_acceleration.y = 9.81 * imu_data[1]
      imu_msg.linear_acceleration.z = 9.81 * imu_data[2]
      imu_msg.angular_velocity.x = 3.14 / 180.0 * imu_data[3]
      imu_msg.angular_velocity.y = 3.14 / 180.0 * imu_data[4]
      imu_msg.angular_velocity.z = 3.14 / 180.0 * imu_data[5]
      self.imu_pub.publish(imu_msg)
      
      enc_msg = Int32()
      enc_msg.data = enc_data[0]
      self.r_enc_pub.publish(enc_msg)
      enc_msg.data = enc_data[1]
      self.l_enc_pub.publish(enc_msg)

      v_bat = self.zumy.read_voltage()
      self.batt_pub.publish(v_bat)
      self.heartBeat.publish("Alive, enabled is: " + str(self.zumy.enabled) + " Battery unsafe is: " + str(self.zumy.battery_unsafe()))
      self.rate.sleep()

      if time.time() > (self.last_message_at + self.timeout): #i've gone too long without seeing the watchdog.
        self.watchdog = False
        self.zumy.disable()
      self.zumy.battery_protection() # a function that needs to be called with some regularity.


    # If shutdown, turn off motors & disable anything else.
    self.zumy.disable()

if __name__ == '__main__':
  zr = ZumyROS()
  zr.run()
