#!/usr/bin/python

import rospy

from geometry_msgs.msg import Twist
from threading import Condition
from zumy import Zumy
from std_msgs.msg import String,Header,Int16
from sensor_msgs.msg import Imu

import socket,time

class ZumyROS:	
  def __init__(self):
    self.zumy = Zumy()
    rospy.init_node('zumy_ros')
    self.cmd = (0,0)
    rospy.Subscriber('cmd_vel', Twist, self.cmd_callback,queue_size=1)
    self.lock = Condition()
    self.rate = rospy.Rate(30.0)
    self.name = socket.gethostname()
    self.heartBeat = rospy.Publisher('heartBeat', String, queue_size=5)
    self.imu_pub = rospy.Publisher('imu', Imu, queue_size = 1)
    self.r_enc_pub = rospy.Publisher('r_enc', Int16, queue_size = 5)
    self.l_enc_pub = rospy.Publisher('l_enc', Int16, queue_size = 5)
    self.imu_count = 0

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
      
      enc_msg = Int16()
      enc_msg.data = enc_data[0]
      self.r_enc_pub.publish(enc_msg)
      enc_msg.data = enc_data[1]
      self.l_enc_pub.publish(enc_msg)

      self.heartBeat.publish("I am alive")
      self.rate.sleep()

    # If shutdown, turn off motors
    self.zumy.cmd(0,0)

if __name__ == '__main__':
  zr = ZumyROS()
  zr.run()
