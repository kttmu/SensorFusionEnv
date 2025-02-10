import sensors.sensor_dist.decoder_can as decoder_can

class CAN_data:
    def __init__(self):
        
        self.speed = 0
        self.yawrate   = 0
        self.PSM_Object_Info1 = []
        self.PSM_Object_Info2 = []
        self.PSM_Object_Info3 = []
        self.PSM_Distance_Info = []

        self.PSM_Object_Info1_pop = []
        self.PSM_Object_Info2_pop = []
        self.PSM_Object_Info3_pop = []
        self.PSM_Distance_Info_pop = []

    def ADAS_CAN_rear(self,pktData,canID):

    
        if canID == 0x340:
            
            decoder_can.decode_PSM_Unit_Info(pktData)

        if canID == 0x217:
            speed=decoder_can.decode_wheel_Data(pktData)

            self.speed=speed

        if canID == 0x78:
            yawrate=decoder_can.decode_yawrate(pktData)
   
            self.yawrate=yawrate
        return self.speed,self.yawrate,self.PSM_Object_Info1,self.PSM_Object_Info2,self.PSM_Object_Info3,self.PSM_Distance_Info

