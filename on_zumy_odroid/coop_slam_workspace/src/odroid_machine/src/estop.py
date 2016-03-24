#!/usr/bin/python

from tkinter import Tk, Label, Button

import rospy
import time
from std_msgs.msg import String,Bool,Float32

class GUI:
    def __init__(self, master):

        #GUI Stuff
        self.master = master
        master.title("A simple GUI")

        self.label = Label(master, text="Zumy Control GUI \n Press the 'x' to stop\n")
        self.label.pack()

        self.robot_state_label = Label(master,text='Robot is enabled')
        self.robot_state_label.pack()

        self.voltage = 0
        self.voltage_label = Label(master,text = "VBatt = " + str(self.voltage))
        self.voltage_label.pack()

        print("Warning: Robot starts enabled")
        self.enable_button = Button(master, text="Disable", command=self.change_enable_state)
        self.enabled = True
        self.enable_button.pack()

        self.close_button = Button(master, text="ESTOP", command=self.estop,bg='red')
        self.close_button.pack()

        #ROS stuff
        rospy.init_node('base_GUI')

        self.heartbeat_pub = rospy.Publisher('/base_computer',String,queue_size=1) #/base_computer topic, the global watchdog.  May want to investigate what happens when there moer than one computer and more than one zumy
        self.zumy_heatbeat = rospy.Subscriber('/zumy7a/heartBeat',String,self.callback,queue_size = 1)
        self.zumy_enable = rospy.Publisher('zumy7a/enable',Bool,queue_size = 1) #The GUI actuates the publish topic.
        self.zumy_voltage = rospy.Subscriber('/zumy7a/Batt',Float32,self.voltage_callback,queue_size = 1)

        self.last_heard = time.time()

    def change_enable_state(self):
        if self.enabled:
            self.enabled = False
            #Robot is disabled, the button is to enable
            self.enable_button["text"] = "Enable"
            self.robot_state_label.config(text = "Robot is disabled")
        else:
            self.enabled = True
            #Robot is enabled, the button will disable
            self.enable_button["text"] = "Disable"
            self.robot_state_label.config(text = "Robot is enabled")
        self.zumy_enable.publish(Bool(self.enabled)) #Change enabled state

    def estop(self):
        self.zumy_enable.publish(Bool(False)) #Bool is the ROS bool.  Disable the robot, please.
        self.enable_button["text"] = "Enable"
        self.robot_state_label.config(text = "Robot is disabled")
        self.enabled = False
        pass

    def callback(self,msg):
        self.last_heard = time.time()

    def voltage_callback(self,msg):
        self.voltage = float(msg.data)
        self.voltage_label.config(text = "VBatt = " + ("%.2f" % round(self.voltage,2)) + " V" )




def check():
    #force Tkinter to do something every 250ms... which is useful so it'll notice ctr-c in a reasonable timeframe
    root.after(250, check) # 250 stands for 250 ms.
    my_gui.heartbeat_pub.publish(String("Foo")) #publish the computer's watchdog
    #


if __name__ == '__main__':
    



    root = Tk()
    my_gui = GUI(root)
    root.after(250, check)
    root.mainloop()

#root = Tk()
#my_gui = MyFirstGUI(root)
#root.mainloop()