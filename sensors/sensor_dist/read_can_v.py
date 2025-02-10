#vehicle can
import struct
from sensors.sensor_dist.can_id_get_v import CAN_data

Candata=CAN_data()

class ReadCAN:

    def __init__(self):

        self.init_flg = 1
        self.read_flg = 0
        self.counter  = 0
        self.data     = [0]*22
        self.data_Packet= b'\x00'

        self.timestamp = 0
        self.speed     = 0
        self.yawrate   = 0



# -------------------------------------------------------- #
    def CANOut(self,d):

        if self.read_flg == 1:
            self.read_flg = 0

        if self.init_flg == 1:
            self.counter += 1
            if self.counter == 32:        # can.bin header area
                self.init_flg = 0
                self.counter = 0
        else:
            self.counter += 1

            self.data[self.counter-1] = ord(d)

            if self.counter == 22:        # can.din data packet area
                self.read_flg = 1
                self.counter = 0
                
                self.data_Packet=struct.pack("B"* len(self.data),*self.data)
                
                self.read_timestamp()
                canID = self.read_CANID()

                speed,yawrate,Object_Info1,Object_Info2,Object_Info3,Distance_Info=Candata.ADAS_CAN_rear(self.data_Packet[14:22],canID)
    
                
   
                self.speed=speed
                self.yawrate = yawrate
     
        return self.timestamp, self.read_flg, self.speed,self.yawrate 


# -------------------------------------------------------- #
    def read_CANID(self):
        

        canID = self.data_Packet[8]+self.data_Packet[9]*256
        return canID


# -------------------------------------------------------- #
    def read_timestamp(self):
        
        #print( self.data_Packet[0:8])
        timestamp  = struct.unpack("<Q", self.data_Packet[0:8])

        self.timestamp = timestamp[0]


# -------------------------------------------------------- #

