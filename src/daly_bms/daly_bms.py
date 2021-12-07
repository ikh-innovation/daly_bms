from sys import float_repr_style
import threading
from time import sleep
import rospy

from rcomponent.rcomponent import RComponent
from ikh_ros_msgs.msg import BatteryStatus

from daly_bms_driver import DalyBMSDriver

class DalyBMS(RComponent):
    def __init__(self):
        RComponent.__init__(self)

        self._driver = DalyBMSDriver()
        self._battery_status = BatteryStatus()
        self._last_battery_state = 'Unknown'
        self._time_init_charging = rospy.Time.now()
        self._last_discharge_value = 3.0

        self._go_to_charge = False
        self._stop_charge = False
        
        #read params
        self._port = rospy.get_param('~serial_port', "/dev/ttyUSB0")
        self._max_total_voltage = rospy.get_param('~max_total_voltage')
        self._min_total_voltage = rospy.get_param('~min_total_voltage')
        self._max_cell_voltage = rospy.get_param('~max_cell_voltage')
        self._min_cell_voltage = rospy.get_param('~min_cell_voltage')
        self._max_temperature = rospy.get_param('~max_temperature', 40)
        self._min_temperature = rospy.get_param('~min_temperature', 5)
        self._battery_capacity = rospy.get_param('~battery_capacity', 30)
    
    def ros_read_params(self):
        RComponent.ros_read_params(self)
        
        # self._port = rospy.get_param('~serial_port', "/dev/ttyUSB_BMS")
    
    def ros_setup(self):
        self._battery_status_pub = rospy.Publisher("~data", BatteryStatus, queue_size=10)
        self._reading_timer = threading.Timer(self._publish_state_timer, self.read)
        
        RComponent.ros_setup(self)
    
    def setup(self):
        self._driver.connect(self._port)
        self._reading_timer.start()

        RComponent.setup(self)
    
    def shutdown(self):
        self._reading_timer.cancel()

        RComponent.shutdown(self)

    def ros_shutdown(self):
        self._battery_status_pub.unregister()

        RComponent.ros_shutdown(self)
        
    def read(self):

        # Get SOC
        data = self._driver.get_soc()        
        self._battery_status.voltage = data['total_voltage']
        self._battery_status.current = data['current']

        # self._battery_status.level = int(round(((self._battery_status.voltage - self._min_total_voltage)*100)/(self._max_total_voltage-self._min_total_voltage)))
        # self._battery_status.level = data['soc_percent']

        # Get Temperature
        data = self._driver.get_temperatures()
        self._battery_status.temperature = data[1]

        # Get MOSFET Status
        data = self._driver.get_mosfet_status()
        # print(data)
        
        if data['mode'] == 'discharging':
            self._battery_status.status = "Discharging"
        elif data['mode'] == 'charging':
            self._battery_status.status = "Charging"
        else:
            self._battery_status.status = "Stationary"

        self._battery_status.rated_capacity = data['capacity_ah']

        self._battery_status.level = int(round( (data['capacity_ah']*100) / self._battery_capacity ))

        # if data['mode'] == 'discharging':
        #     self._battery_status.is_charging = False
        #     self._battery_status.time_charging = 0
        #     self._last_discharge_value = self._battery_status.current
        
        # elif data['mode'] == 'charging':
        
        #     if self._last_battery_state == 'Unknown' or self._last_battery_state == 'discharging':
        #         self._time_init_charging = rospy.Time.now().secs


        #     self._battery_status.is_charging = True
        #     elapsed_time = (rospy.Time.now().secs - self._time_init_charging)/60
        #     elapsed_time = int(elapsed_time)

        #     self._battery_status.time_charging = elapsed_time

        # remaining_hours = round(data['capacity_ah']/self._last_discharge_value, 0)
        # self._battery_status.time_remaining = int(remaining_hours)*60
        # self._last_battery_state = data['mode']

        # Get Cell Voltage Range
        data = self._driver.get_cell_voltage_range()
        self._battery_status.diff_volt = round(float(data['highest_voltage'] - data['lowest_voltage']),3)

        # print(data['lowest_voltage'],self._min_cell_voltage)

        if data['lowest_voltage'] <= self._min_cell_voltage:
            self._go_to_charge = True
        else:
            self._go_to_charge = False

        self._battery_status.go_to_charge = self._go_to_charge

        if data['highest_voltage'] >= self._max_cell_voltage:
            self._stop_charge = True
        else:
            self._stop_charge = False
        
        self._battery_status.stop_charge = self._stop_charge

        # if data['highest_voltage'] >= self._max_cell_voltage:
        #     print("cut charging")
        # elif data['highest_voltage'] >= self._max_cell_voltage:
        #     print("cut charging")

        data = None
        self._reading_timer = threading.Timer(self._publish_state_timer, self.read)
        self._reading_timer.start()

    def ros_publish(self):
        self._battery_status_pub.publish(self._battery_status)



