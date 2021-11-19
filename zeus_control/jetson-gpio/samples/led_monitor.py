
#!/usr/bin/env python 

import rospy
from std_msgs.msg import Int32MultiArray
import RPi.GPIO as GPIO
import time


LIGHT_MODES = { 'OFF':    0,
                'FLASH':    1,
                'ON':   2}

LIGHT_PIN = {   'GREEN' :   26,
                'YELLOW' :  20,
                'RED' :     21}

LIGHT_MESSAGE = {   'GREEN' :   0,
                    'YELLOW' :  1,
                    'RED' :     2}


class LedMonitor():
    def __init__(self):
        '''
        Node to control led tower
        '''
        rospy.init_node('led_monitor', anonymous=False)
        rospy.on_shutdown(self.on_shutdown)


        # Subscriber
        rospy.Subscriber("/zeus_monitor/light_control", Int32MultiArray, self.callback)

        rospy.Timer(rospy.Duration(1.0/2), self.led_callback)

        self.light_msg = [0, 0, 0]
        self.light_current = [0, 0, 0]
        GPIO.setmode(GPIO.BCM) 
        GPIO.setup(LIGHT_PIN['GREEN'], GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(LIGHT_PIN['YELLOW'], GPIO.OUT, initial=GPIO.HIGH)
        GPIO.setup(LIGHT_PIN['RED'], GPIO.OUT, initial=GPIO.HIGH)

        


    def callback(self, msg):
        self.light_msg = msg.data


    def led_callback(self, evt):
        color = 'GREEN'
        for color in LIGHT_MESSAGE.keys():
            
            if self.light_msg[LIGHT_MESSAGE[color]] == LIGHT_MODES['ON']:
                GPIO.output(LIGHT_PIN[color], GPIO.LOW)
                self.light_current[LIGHT_MESSAGE[color]] = GPIO.LOW
            elif self.light_msg[LIGHT_MESSAGE[color]] == LIGHT_MODES['FLASH']:
                self.light_current[LIGHT_MESSAGE[color]] ^= GPIO.HIGH
                GPIO.output(LIGHT_PIN[color], self.light_current[LIGHT_MESSAGE[color]])
            elif self.light_msg[LIGHT_MESSAGE[color]] == LIGHT_MODES['OFF']:
                GPIO.output(LIGHT_PIN[color], GPIO.HIGH)
                self.light_current[LIGHT_MESSAGE[color]] = GPIO.HIGH
            

    def on_shutdown(self):
        GPIO.cleanup()




if __name__ == '__main__':
    try:
        node = LedMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

