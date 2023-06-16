import os
from smbus2 import SMBus
os.environ['DISPLAY']=':0.0'
import time

import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Pose
 

#nom nom nom

class ATTinyI2C(Node):

    def __init__(self):
        super().__init__('attiny_i2c')
        self.subscription = self.create_subscription(
            Pose,
            'ATTinyinfo',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        #self.done_ACK = self.create_publisher(Bool,'arm_ACK',10)

    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)

        
        angle1 = msg.position.x
        angle2 = msg.position.y
        angle3 = msg.position.z
        angle4 = msg.orientation.x
        angle5 = msg.orientation.y
        angle6 = msg.orientation.z

        gripper = msg.orientation.w

        steps1 = int(round(angle1*20.0*200.0/360.0))
        steps2 = int(round(angle2*46.656*200.0/360.0))
        steps3 = int(round(angle3*46.656*200.0/360.0))
        steps4 = int(round(angle4*19.203*200.0/360.0))
        steps5 = int(round(angle5*20.0*200.0/360.0))
        steps6 = int(round(angle6*19.203*200.0/360.0))

# Motor 1 y 2

        if(steps1>0 and steps2 == 0):
          Direccion1=0b00010001

        elif(steps1<0 and steps2 == 0):
          Direccion1=0b00110001

        elif(steps2>0 and steps1 == 0):
          Direccion1=0b00010001

        elif(steps2<0 and steps1 == 0):
          Direccion1=0b00010011

        elif(steps2>0 and steps1 > 0):
          Direccion1=0b00010001

        elif(steps2<0 and steps1 > 0):
          Direccion1=0b00010011

        elif(steps2>0 and steps1 < 0):
          Direccion1=0b00110001

        elif(steps2<0 and steps1 < 0):
          Direccion1=0b00110011

# Motor 3 y 4

        if(steps3>0 and steps4 == 0):
          Direccion2=0b00010001

        elif(steps3<0 and steps4 == 0):
          Direccion2=0b00110001

        elif(steps4>0 and steps3 == 0):
          Direccion2=0b00010001

        elif(steps4<0 and steps3 == 0):
          Direccion2=0b00010011

        elif(steps4>0 and steps3 > 0):
          Direccion2=0b00010001

        elif(steps4<0 and steps3 > 0):
          Direccion2=0b00010011

        elif(steps4>0 and steps3 < 0):
          Direccion2=0b00110001

        elif(steps4<0 and steps3 < 0):
          Direccion2=0b00110011

# Motor 5 y 6

        if(steps5>0 and steps6 == 0):
          Direccion3=0b00010001

        elif(steps5<0 and steps6 == 0):
          Direccion3=0b00110001

        elif(steps6>0 and steps5 == 0):
          Direccion3=0b00010001

        elif(steps6<0 and steps5 == 0):
          Direccion3=0b00010011

        elif(steps6>0 and steps5 > 0):
          Direccion3=0b00010001

        elif(steps6<0 and steps5 > 0):
          Direccion3=0b00010011

        elif(steps6>0 and steps5 < 0):
          Direccion3=0b00110001

        elif(steps6<0 and steps5 < 0):
          Direccion3=0b00110011

# Motor 1 y 2

        address = 0x23

        lista_strings = {
            "direccion": Direccion1,
            "pasos1": steps1,
            "pasos2": steps2
          }

        if(steps1 ~= 0 and steps2 ~= 0):
          try:
              bus = SMBus(3)
              bus.write_byte(address, lista_strings["direccion"] & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              print("1: ")
              print(lista_strings["direccion"])
              
              bus.write_byte(address, lista_strings["pasos1"] >> 8 & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              print("2: ")
              print(lista_strings["pasos1"] >> 8 & 0xff)
              
              bus.write_byte(address, lista_strings["pasos1"] & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              print("3: ")
              print(lista_strings["pasos1"] & 0xff)
              
              bus.write_byte(address, lista_strings["pasos2"] >> 8 & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              print("4: ")
              print(lista_strings["pasos2"] >> 8 & 0xff)
              
              bus.write_byte(address, lista_strings["pasos2"] & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              print("5: ")
              print(lista_strings["pasos2"] & 0xff)
              
              
          except Exception as e:
              pass
        
# Motor 3 y 4

        address = 0x24

        lista_strings = {
            "direccion": Direccion2,
            "pasos3": steps3,
            "pasos4": steps4
          }

        if(steps3 ~= 0 and steps4 ~= 0):
          try:
              bus = SMBus(3)
              bus.write_byte(address, lista_strings["direccion"] & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              print("1: ")
              print(lista_strings["direccion"])
              
              bus.write_byte(address, lista_strings["pasos3"] >> 8 & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              print("2: ")
              print(lista_strings["pasos3"] >> 8 & 0xff)
              
              bus.write_byte(address, lista_strings["pasos3"] & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              print("3: ")
              print(lista_strings["pasos3"] & 0xff)
              
              bus.write_byte(address, lista_strings["pasos4"] >> 8 & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              print("4: ")
              print(lista_strings["pasos4"] >> 8 & 0xff)
              
              bus.write_byte(address, lista_strings["pasos4"] & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              print("5: ")
              print(lista_strings["pasos4"] & 0xff)
              
              
          except Exception as e:
              pass


# Motor 5 y 6

        address = 0x25

        lista_strings = {
            "direccion": Direccion3,
            "pasos5": steps5,
            "pasos6": steps6
          }

        if(steps5 ~= 0 and steps6 ~= 0):
          try:
              bus = SMBus(3)
              bus.write_byte(address, lista_strings["direccion"] & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              print("1: ")
              print(lista_strings["direccion"])
              
              bus.write_byte(address, lista_strings["pasos5"] >> 8 & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              print("2: ")
              print(lista_strings["pasos5"] >> 8 & 0xff)
              
              bus.write_byte(address, lista_strings["pasos5"] & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              print("3: ")
              print(lista_strings["pasos5"] & 0xff)
              
              bus.write_byte(address, lista_strings["pasos6"] >> 8 & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              print("4: ")
              print(lista_strings["pasos6"] >> 8 & 0xff)
              
              bus.write_byte(address, lista_strings["pasos6"] & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              print("5: ")
              print(lista_strings["pasos6"] & 0xff)
              
              
          except Exception as e:
              pass

        #if(gripper == 1.0):
          #cosaparaqueswitcheeellaser()

        #elif(gripper == 2.0):
          #cosaparaqueswitcheeelgripper()


def main(args=None):
    rclpy.init(args=args)

    attiny_i2c = ATTinyI2C()

    rclpy.spin(attiny_i2c)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    attiny_i2c.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()