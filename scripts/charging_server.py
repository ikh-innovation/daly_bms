#! /usr/bin/env python

import rospy

import actionlib

import actionlib_tutorials.msg

from ikh_ros_msgs.msg import BatteryStatus, Labjack_dout
from daly_bms.msg import EnableChargeAction, EnableChargeFeedback, EnableChargeResult
from std_msgs.msg import Bool

ENABLE_CHARGE = 0

class EnableChargingServer(object):
    def __init__(self):
        # Initialize Subscriber to BMS data
        self._sub = rospy.Subscriber("daly_bms/data", BatteryStatus, self.sub_callback)

        # Initialize Subscriber to Charging Limit Switch
        self._sub_limit_switch = rospy.Subscriber("/aristos/charging_limit_switch", Labjack_dout, self.sub_callback_limit_switch)

        # Initialize Publisher to Chargin relay
        self._pub_charging_relay = rospy.Publisher('charging_switch', Bool, queue_size=10)

        # Initialize Server Result and Feedback
        self._feedback = EnableChargeFeedback()
        self._result = EnableChargeResult()

        # Create Action
        self.server = actionlib.SimpleActionServer('enable_charge', EnableChargeAction, self.execute, False)

        # When Initialization Complete Start Action Server
        self.server.start()

    def sub_callback(self, data):
        self.bat_total_voltage = data.voltage
        self.bat_level = data.level
        self.bat_capacity = data.rated_capacity
        self.bat_status = data.status
        self.bat_go_to_charge = data.go_to_charge
        self.bat_stop_charge = data.stop_charge
        self.bat_diff_volt = data.diff_volt

    def sub_callback_limit_switch(self, data):
        self.limit_switch = data.state
        
    def execute(self, goal):
        r = rospy.Rate(1)
        stop = False
        charging = False
        charged_flag = False
        balancing_delay = 300 #5 mins
        charging_delay = 20 #20 sec
        bat_high_voltage = 51.5 #Volt

        # Condition to exit if the charging limit switch is not triggered
        if not self.limit_switch:
            self._feedback.STATUS = "Robot's charging limit switch is not triggered yet. Please retry docking."
            rospy.logwarn(self._feedback.STATUS)
            rospy.loginfo("Exiting")
            self._result.CHARGING_SUCCESS = False
            self._result.BATTERY_LEVEL = self.bat_level
            self.server.set_aborted(result = self._result.CHARGING_SUCCESS, text = self._feedback.STATUS)
            self._pub_charging_relay.publish(False)
            return

        if goal.action_id == ENABLE_CHARGE:
            rospy.loginfo("Action call with ENABLE_CHARGING status")
            self._feedback.STATUS = "Awaiting to begin charging"

        # Publish on labjack to enable charging switch
        self._pub_charging_relay.publish(True)

        start = rospy.Time.now().to_sec()

        while not stop:
            self._feedback.BATTERY_LEVEL = self.bat_level

            # feedback: charging (if self.bat_status == "Charging")
            if self.bat_status == "Charging":
                self._feedback.STATUS = "Charging"
                charging = True

            # feedback: stop charging 
            if self.bat_status == "Charging" and self.bat_total_voltage > bat_high_voltage and charging:
                self._pub_charging_relay.publish(False)
                self._feedback.STATUS = "Charging stoped successfully"
                # rospy.loginfo(self._feedback.STATUS)
                charged_flag = True
                start_balancing = rospy.Time.now().to_sec()
                

            # check for ballancing cell status
            if charged_flag:
                self._feedback.STATUS = "Charged. Balancing cells remaining!"
                # rospy.loginfo(self._feedback.STATUS)

                # wait for ballancing cell
                if self.bat_diff_volt < 0.080 and (rospy.Time.now().to_sec() - start_balancing) > balancing_delay:
                    self._feedback.STATUS = "Robot is fully charged and ready to start"
                    rospy.loginfo(self._feedback.STATUS)
                    self._result.CHARGING_SUCCESS = True
                    self._result.BATTERY_LEVEL = self.bat_level
                    self.server.set_succeeded(self._result)
                    stop = True
            
            # Condition to exit if the charging has never started
            if (rospy.Time.now().to_sec() - start) > (charging_delay-1) and not charging:
                self._feedback.STATUS = "Robot has not accomplish to begin charging."
                rospy.logwarn(self._feedback.STATUS)
                rospy.loginfo("Exiting")
                self._result.CHARGING_SUCCESS = False
                self._result.BATTERY_LEVEL = self.bat_level
                self.server.set_aborted(result = self._result.CHARGING_SUCCESS, text = self._feedback.STATUS)
                self._pub_charging_relay.publish(False)
                stop = True

            # Condition to exit if the charging has never started with low volage detection
            if (rospy.Time.now().to_sec() - start) > charging_delay and self.bat_total_voltage < 41.0 and not charging:
                # rospy.logerr("set abort")
                self._feedback.STATUS = "Robot has not accomplish to begin charging for {} minutes. Caution Low Voltage!".format(int(charging_delay/60))
                rospy.logwarn(self._feedback.STATUS)
                rospy.loginfo("Exiting")
                self._result.CHARGING_SUCCESS = False
                self._result.BATTERY_LEVEL = self.bat_level
                self.server.set_aborted(result = self._result.CHARGING_SUCCESS, text = self._feedback.STATUS)
                self._pub_charging_relay.publish(False)
                stop = True
            
            # Condition to exit if the charging limit switch is not untriggered while charging
            if not self.limit_switch and charging:
                self._feedback.STATUS = "Robot's charging limit switch is untriggered. Please retry docking."
                rospy.logwarn(self._feedback.STATUS)
                rospy.loginfo("Exiting")
                self._result.CHARGING_SUCCESS = False
                self._result.BATTERY_LEVEL = self.bat_level
                self.server.set_aborted(result = self._result.CHARGING_SUCCESS, text = self._feedback.STATUS)
                self._pub_charging_relay.publish(False)
                stop = True

            self.server.publish_feedback(self._feedback)
            r.sleep()  

if __name__ == '__main__':
    rospy.init_node('enable_charging_server')
    server = EnableChargingServer()
    rospy.spin()