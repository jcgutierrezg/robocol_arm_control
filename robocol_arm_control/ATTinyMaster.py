import os
from smbus2 import SMBus
os.environ['DISPLAY']=':0.0'
import time
import serial
import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
 
gripperState = 1.0
laserState = 0.0

waitingACK1 = False
waitingACK2 = False
waitingACK3 = False

#nom nom nom

class ATTinyI2C(Node):

    def __init__(self):

        global waitingACK1, waitingACK2, waitingACK3

        super().__init__('attiny_i2c')
        #gripperSer = serial.Serial("/dev/ttyUSB0", baudrate=9600) #Modificar el puerto serie de ser necesario
        self.ACKflagPub = self.create_publisher(Bool,'robocol/arm/next_position',10)
        self.subscription = self.create_subscription(
            Pose,
            'ATTinyinfo',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        #self.get_logger().info('I heard: "%s"' % msg.data)

        global gripperState, laserState, waitingACK1, waitingACK2, waitingACK3

        #ACK = True

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
        steps5 = int(round(angle5*46.656*200.0/360.0))
        steps6 = int(round(angle6*19.203*200.0/360.0))

        print(steps1)
        print(steps2)
        print(steps3)
        print(steps4)
        print(steps5)
        print(steps6)

        if((steps1 & 0xff) == 0xFF):
          steps1 = steps1+1

        if((steps2 & 0xff) == 0xFF):
          steps2 = steps2+1

        if((steps3 & 0xff) == 0xFF):
          steps3 = steps3+1

        if((steps4 & 0xff) == 0xFF):
          steps4 = steps4+1

        if((steps5 & 0xff) == 0xFF):
          steps5 = steps5+1

        if((steps6 & 0xff) == 0xFF):
          steps6 = steps6+1

        Direccion1 = 0b00010001
        Direccion2 = 0b00010001
        Direccion3 = 0b00010001

# Motor 1 y 2

        if(steps1>0 and steps2 == 0):
          Direccion1=0b00010001

        elif(steps1<0 and steps2 == 0):
          Direccion1=0b00110001

        elif(steps2>0 and steps1 == 0):
          Direccion1=0b00010011

        elif(steps2<0 and steps1 == 0):
          Direccion1=0b00010001

        elif(steps2>0 and steps1 > 0):
          Direccion1=0b00010011

        elif(steps2<0 and steps1 > 0):
          Direccion1=0b00010001

        elif(steps2>0 and steps1 < 0):
          Direccion1=0b00110011

        elif(steps2<0 and steps1 < 0):
          Direccion1=0b00110001

        print(Direccion1)

# Motor 3 y 4

        if(steps3>0 and steps4 == 0):
          Direccion2=0b00010001

        elif(steps3<0 and steps4 == 0):
          Direccion2=0b00110001

        elif(steps4>0 and steps3 == 0):
          Direccion2=0b00010011

        elif(steps4<0 and steps3 == 0):
          Direccion2=0b00010001

        elif(steps4>0 and steps3 > 0):
          Direccion2=0b00010011

        elif(steps4<0 and steps3 > 0):
          Direccion2=0b00010001

        elif(steps4>0 and steps3 < 0):
          Direccion2=0b00110011

        elif(steps4<0 and steps3 < 0):
          Direccion2=0b00110001

# Motor 5 y 6

        if(steps5>0 and steps6 == 0):
          Direccion3=0b00110001

        elif(steps5<0 and steps6 == 0):
          Direccion3=0b00010001

        elif(steps6>0 and steps5 == 0):
          Direccion3=0b00110011

        elif(steps6<0 and steps5 == 0):
          Direccion3=0b00110001

        elif(steps6>0 and steps5 > 0):
          Direccion3=0b00110011

        elif(steps6<0 and steps5 > 0):
          Direccion3=0b00110001

        elif(steps6>0 and steps5 < 0):
          Direccion3=0b00010011

        elif(steps6<0 and steps5 < 0):
          Direccion3=0b00010001

        steps1 = abs(steps1)
        steps2 = abs(steps2)
        steps3 = abs(steps3)
        steps4 = abs(steps4)
        steps5 = abs(steps5)
        steps6 = abs(steps6)

# Motor 1 y 2

        address = 0x23

        lista_strings = {
            "direccion": Direccion1,
            "pasos1": steps1,
            "pasos2": steps2
          }

        if(steps1 != 0 or steps2 != 0):
          print("Steps1: " + str(steps1))
          print("Steps2: " + str(steps2))
          print(Direccion1)

          bus = SMBus(1)

          complete = False

          while(not complete):

            sentCount = 0
            corruptData = False

            #------------BYTE1-------------

            sentFlag = False
            while(not sentFlag and not corruptData):
              bus.write_byte(address, lista_strings["direccion"] & 0xff)
              #print("1: ")
              #print(lista_strings["direccion"])
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              if(reading == 0b00100001):
                sentFlag = True
                sentCount = sentCount+1
              else:
                print("ATTiny data corrupted, resending all bytes on 0x23")
                corruptData = True
                bus.write_byte(address, 0b11111111 & 0xff)

            #------------BYTE2-------------

            sentFlag = False
            while(not sentFlag and not corruptData):
              bus.write_byte(address, lista_strings["pasos1"] >> 8 & 0xff)
              #print("2: ")
              #print(lista_strings["pasos1"] >> 8 & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              if(reading == 0b00100010):
                sentFlag = True
                sentCount = sentCount+1
              else:
                print("ATTiny data corrupted, resending all bytes on 0x23")
                corruptData = True
                bus.write_byte(address, 0b11111111 & 0xff)

            #------------BYTE3-------------

            sentFlag = False
            while(not sentFlag and not corruptData):
              bus.write_byte(address, lista_strings["pasos1"] & 0xff)
              #print("3: ")
              #print(lista_strings["pasos1"] & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              if(reading == 0b00100100):
                sentFlag = True
                sentCount = sentCount+1
              else:
                print("ATTiny data corrupted, resending all bytes on 0x23")
                corruptData = True
                bus.write_byte(address, 0b11111111 & 0xff)

             #------------BYTE4-------------

            sentFlag = False
            while(not sentFlag and not corruptData):
              bus.write_byte(address, lista_strings["pasos2"] >> 8 & 0xff)
              #print("4: ")
              #print(lista_strings["pasos2"] >> 8 & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              if(reading == 0b00101000):
                sentFlag = True
                sentCount = sentCount+1
              else:
                print("ATTiny data corrupted, resending all bytes on 0x23")
                corruptData = True
                bus.write_byte(address, 0b11111111 & 0xff)

              #------------BYTE5-------------

            sentFlag = False
            while(not sentFlag and not corruptData):
              bus.write_byte(address, lista_strings["pasos2"] & 0xff)
              #print("5: ")
              #print(lista_strings["pasos2"] & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              if(reading == 0b00110000):
                sentFlag = True
                sentCount = sentCount+1
              else:
                print("ATTiny data corrupted, resending all bytes on 0x23")
                corruptData = True
                bus.write_byte(address, 0b11111111 & 0xff)

              if(sentCount == 5):
                complete = True

          waitingACK1 = True
          
          print("Bytes sent to 0x23!")
        
# Motor 3 y 4

        address = 0x24

        lista_strings = {
            "direccion": Direccion2,
            "pasos3": steps3,
            "pasos4": steps4
          }

        if(steps3 != 0 or steps4 != 0):
          print("Steps3: " + str(steps3))
          print("Steps4: " + str(steps4))
          print(Direccion2)
          bus = SMBus(1)

          complete = False

          while(not complete):

            sentCount = 0
            corruptData = False

            #------------BYTE1-------------

            sentFlag = False
            while(not sentFlag and not corruptData):
              bus.write_byte(address, lista_strings["direccion"] & 0xff)
              #print("1: ")
              #print(lista_strings["direccion"])
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              if(reading == 0b01000001):
                sentFlag = True
                sentCount = sentCount+1
              else:
                print("ATTiny data corrupted, resending all bytes on 0x24")
                corruptData = True
                bus.write_byte(address, 0b11111111 & 0xff)

            #------------BYTE2-------------

            sentFlag = False
            while(not sentFlag and not corruptData):
              bus.write_byte(address, lista_strings["pasos3"] >> 8 & 0xff)
              #print("2: ")
              #print(lista_strings["pasos1"] >> 8 & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              if(reading == 0b01000010):
                sentFlag = True
                sentCount = sentCount+1
              else:
                print("ATTiny data corrupted, resending all bytes on 0x24")
                corruptData = True
                bus.write_byte(address, 0b11111111 & 0xff)

            #------------BYTE3-------------

            sentFlag = False
            while(not sentFlag and not corruptData):
              bus.write_byte(address, lista_strings["pasos3"] & 0xff)
              #print("3: ")
              #print(lista_strings["pasos1"] & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              if(reading == 0b01000100):
                sentFlag = True
                sentCount = sentCount+1
              else:
                print("ATTiny data corrupted, resending all bytes on 0x24")
                corruptData = True
                bus.write_byte(address, 0b11111111 & 0xff)

             #------------BYTE4-------------

            sentFlag = False
            while(not sentFlag and not corruptData):
              bus.write_byte(address, lista_strings["pasos4"] >> 8 & 0xff)
              #print("4: ")
              #print(lista_strings["pasos2"] >> 8 & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              if(reading == 0b01001000):
                sentFlag = True
                sentCount = sentCount+1
              else:
                print("ATTiny data corrupted, resending all bytes on 0x24")
                corruptData = True
                bus.write_byte(address, 0b11111111 & 0xff)

              #------------BYTE5-------------

            sentFlag = False
            while(not sentFlag and not corruptData):
              bus.write_byte(address, lista_strings["pasos4"] & 0xff)
              #print("5: ")
              #print(lista_strings["pasos2"] & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              if(reading == 0b01010000):
                sentFlag = True
                sentCount = sentCount+1
              else:
                print("ATTiny data corrupted, resending all bytes on 0x24")
                corruptData = True
                bus.write_byte(address, 0b11111111 & 0xff)

              if(sentCount == 5):
                complete = True

          waitingACK2 = True
          
          print("Bytes sent to 0x24!")

# Motor 5 y 6

        address = 0x25

        lista_strings = {
            "direccion": Direccion3,
            "pasos5": steps5,
            "pasos6": steps6
          }

        if(steps5 != 0 or steps6 != 0):
          print("Steps5: " + str(steps5))
          print("Steps6: " + str(steps6))
          print(Direccion3)
          bus = SMBus(1)

          complete = False

          while(not complete):

            sentCount = 0
            corruptData = False

            #------------BYTE1-------------

            sentFlag = False
            while(not sentFlag and not corruptData):
              bus.write_byte(address, lista_strings["direccion"] & 0xff)
              #print("1: ")
              #print(lista_strings["direccion"])
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              if(reading == 0b10000001):
                sentFlag = True
                sentCount = sentCount+1
              else:
                print("ATTiny data corrupted, resending all bytes on 0x25")
                corruptData = True
                bus.write_byte(address, 0b11111111 & 0xff)

            #------------BYTE2-------------

            sentFlag = False
            while(not sentFlag and not corruptData):
              bus.write_byte(address, lista_strings["pasos5"] >> 8 & 0xff)
              #print("2: ")
              #print(lista_strings["pasos1"] >> 8 & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              if(reading == 0b10000010):
                sentFlag = True
                sentCount = sentCount+1
              else:
                print("ATTiny data corrupted, resending all bytes on 0x25")
                corruptData = True
                bus.write_byte(address, 0b11111111 & 0xff)

            #------------BYTE3-------------

            sentFlag = False
            while(not sentFlag and not corruptData):
              bus.write_byte(address, lista_strings["pasos5"] & 0xff)
              #print("3: ")
              #print(lista_strings["pasos1"] & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              if(reading == 0b10000100):
                sentFlag = True
                sentCount = sentCount+1
              else:
                print("ATTiny data corrupted, resending all bytes on 0x25")
                corruptData = True
                bus.write_byte(address, 0b11111111 & 0xff)

             #------------BYTE4-------------

            sentFlag = False
            while(not sentFlag and not corruptData):
              bus.write_byte(address, lista_strings["pasos6"] >> 8 & 0xff)
              #print("4: ")
              #print(lista_strings["pasos2"] >> 8 & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              if(reading == 0b10001000):
                sentFlag = True
                sentCount = sentCount+1
              else:
                print("ATTiny data corrupted, resending all bytes on 0x25")
                corruptData = True
                bus.write_byte(address, 0b11111111 & 0xff)

              #------------BYTE5-------------

            sentFlag = False
            while(not sentFlag and not corruptData):
              bus.write_byte(address, lista_strings["pasos6"] & 0xff)
              #print("5: ")
              #print(lista_strings["pasos2"] & 0xff)
              #time.sleep(0.1)
              reading = bus.read_byte(address)
              if(reading == 0b10010000):
                sentFlag = True
                sentCount = sentCount+1
              else:
                print("ATTiny data corrupted, resending all bytes on 0x25")
                corruptData = True
                bus.write_byte(address, 0b11111111 & 0xff)

              if(sentCount == 5):
                complete = True

          waitingACK3 = True
          
          print("Bytes sent to 0x25!")


        if(gripper == 1.0):
          #cosaparaqueswitcheeellaser()
          if(laserState == 1.0):

            comando = "off"

            comandoBytes = comando.encode()
            #print("\n")
            #gripperSer.write(comandoBytes)
            #time.sleep(0.1)

          elif(laserState == 0.0):

            comando = "on"

            comandoBytes = comando.encode()
            #print("\n")
            #gripperSer.write(comandoBytes)
            #time.sleep(0.1)
          pass

        elif(gripper == 2.0):

          if(gripperState == 1.0):

            comando = "c"

            comandoBytes = comando.encode()
            #print("\n")
            #gripperSer.write(comandoBytes)
            #time.sleep(0.1)

          elif(gripperState == 0.0):

            comando = "a"

            comandoBytes = comando.encode()
            #print("\n")
            #gripperSer.write(comandoBytes)
            #time.sleep(0.1)

        sentNextMsg = False

        while(not sentNextMsg): 

          if(waitingACK1):

            complete1 = False

          else:

            complete1 = True
            print("0x23 didnt recieve steps")

          while(not complete1):

            reading1 = bus.read_byte(0x23)

            if(reading1 == 0b00111111):
               complete1 = True
               print("0x23 done!")
            else:
              time.sleep(0.3)


          if(waitingACK2):

            complete2 = False

          else:

            complete2 = True
            print("0x24 didnt recieve steps")

          while(not complete2):

            reading2 = bus.read_byte(0x24)

            if(reading2 == 0b01011111):
               complete2 = True
               print("0x24 done!")
            else:
              time.sleep(0.3)


          if(waitingACK3):

            complete3 = False

          else:

            complete3 = True
            print("0x25 done!")

          while(not complete3):

            reading3 = bus.read_byte(0x25)

            if(reading3 == 0b10011111):
               complete3 = True
               print("0x25 done!")
            else:
              time.sleep(0.3)


          if(complete1 and complete2 and complete3):

            msg = Bool()

            msg.data = True

            self.ACKflagPub.publish(msg)

            print("Next trajectory point please!")

            sentNextMsg = True
            waitingACK1 = False
            waitingACK2 = False
            waitingACK3 = False

        # if(ACK):

        #   time.sleep(3.0)

        #   msg = Bool()

        #   msg.data = True

        #   self.ACKflagPub.publish(msg)




def main(args=None):
    rclpy.init(args=args)

    attiny_i2c = ATTinyI2C()

    rclpy.spin(attiny_i2c)

    while(rclpy.ok()):

      if(waitingACK1 or waitingACK2 or waitingACK3):
        attiny_i2c.ACK_check()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    attiny_i2c.destroy_node()
    rclpy.shutdown()
    gripperSer.close()


if __name__ == '__main__':
    main()

