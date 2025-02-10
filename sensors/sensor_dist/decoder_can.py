#import dpkt, socket

def decode_PSM_Object_Info1(pktData):
    
    ObjID_PSM1      = (pktData[0]&0b11110000) >> 4
    ObjReli_PSM1    = (pktData[0]&0b00001110) >> 1
    ObjNearX_PSM1   = (pktData[0]&0b00000001) << 11 | \
                      (pktData[1]&0b11111111) << 3  | \
                      (pktData[2]&0b11100000) >> 5
    ObjNearY_PSM1   = (pktData[2]&0b00011111) << 7  | \
                      (pktData[3]&0b11111110) >> 1
    ObjRelVlcX_PSM1 = (pktData[3]&0b00000001) << 10 | \
                      (pktData[4]&0b11111111) << 2  | \
                      (pktData[5]&0b11000000) >> 6
    ObjRelVlcY_PSM1 = (pktData[5]&0b00111111) << 5  | \
                      (pktData[6]&0b11111000) >> 3
    Reserved        = (pktData[6]&0b00000111) << 2  | \
                      (pktData[7]&0b11000000) >> 6
    ObjUpdtCnt_PSM1 = (pktData[7]&0b00110000) >> 4
    ObjCnt_PSM1     = (pktData[7]&0b00001111) >> 0
   
    ObjNearX_PSM1   = ObjNearX_PSM1   * 0.01 - 20
    ObjNearY_PSM1   = ObjNearY_PSM1   * 0.01 - 20
    ObjRelVlcX_PSM1 = ObjRelVlcX_PSM1 * 0.01 - 10
    ObjRelVlcY_PSM1 = ObjRelVlcY_PSM1 * 0.01 - 10
    #print("ID1=",ObjRelVlcX_PSM1)

    return ObjNearX_PSM1,ObjNearY_PSM1,ObjRelVlcX_PSM1,ObjRelVlcY_PSM1,ObjReli_PSM1,ObjID_PSM1

def decode_PSM_Object_Info2(pktData):

    ObjID_PSM2      = (pktData[0]&0b11110000) >> 4
    ObjLdX_PSM2     = (pktData[0]&0b00001111) << 4 | \
                      (pktData[1]&0b11110000) >> 4
    ObjLdY_PSM2     = (pktData[1]&0b00001111) << 4 | \
                      (pktData[2]&0b11110000) >> 4
    ObjRdX_PSM2     = (pktData[2]&0b00001111) << 4 | \
                      (pktData[3]&0b11110000) >> 4
    ObjRdY_PSM2     = (pktData[3]&0b00001111) << 4 | \
                      (pktData[4]&0b11110000) >> 4
    ObjZT_PSM2      = (pktData[4]&0b00001111) << 4 | \
                      (pktData[5]&0b11110000) >> 4
    ObjZB_PSM2      = (pktData[5]&0b00001111) << 4 | \
                      (pktData[6]&0b11110000) >> 4
    ObjNewCnt_PSM2  = (pktData[6]&0b00001100) >> 2
    ObjExtpCnt_PSM2 = (pktData[6]&0b00000011) >> 0
    Reserved        = (pktData[7]&0b11000000) >> 6
    ObjUpdtCnt_PSM2 = (pktData[7]&0b00110000) >> 4
    ObjCnt_PSM2     = (pktData[7]&0b00001111) >> 0

    ObjLdX_PSM2 = ObjLdX_PSM2 * 0.01 -1.25
    ObjLdY_PSM2 = ObjLdY_PSM2 * 0.01 -1.25
    ObjRdX_PSM2 = ObjRdX_PSM2 * 0.01 -1.25
    ObjRdY_PSM2 = ObjRdY_PSM2 * 0.01 -1.25
    ObjZT_PSM2  = ObjZT_PSM2  * 0.01
    ObjZB_PSM2  = ObjZB_PSM2  * 0.01

    return ObjLdX_PSM2,ObjLdY_PSM2,ObjRdX_PSM2,ObjRdY_PSM2,ObjZT_PSM2,ObjZB_PSM2,ObjID_PSM2
#    print(pktData, ObjID_PSM2, ObjLdX_PSM2, ObjLdY_PSM2, ObjRdX_PSM2, ObjRdY_PSM2, ObjZT_PSM2, ObjZB_PSM2)

def decode_PSM_Object_Info3(pktData):

    ObjID_PSM3      = (pktData[0]&0b11110000) >> 4
    ObjRelAccX_PSM3 = (pktData[0]&0b00001111) << 5 | \
                      (pktData[1]&0b11111000) >> 3
    ObjRelAccY_PSM3 = (pktData[1]&0b00000111) << 6 | \
                      (pktData[2]&0b11111100) >> 2
    TTC_PSM3        = (pktData[2]&0b00000011) << 7 | \
                      (pktData[3]&0b11111110) >> 1
    ObjCls_PSM3     = (pktData[3]&0b00000001) << 3 | \
                      (pktData[4]&0b11100000) >> 5
    ObjVec_PSM3     = (pktData[4]&0b00011110) >> 1
    Reserved        = (pktData[4]&0b00000001) << 18 | \
                      (pktData[5]&0b11111111) << 10 | \
                      (pktData[6]&0b11111111) << 2  | \
                      (pktData[7]&0b11000000) >> 6
    ObjUpdtCnt_PSM3 = (pktData[7]&0b00110000) >> 4
    ObjCnt_PSM3     = (pktData[7]&0b00001111) >> 0

    ObjRelAccX_PSM3 = ObjRelAccX_PSM3 * 0.05 -12.5
    ObjRelAccY_PSM3 = ObjRelAccY_PSM3 * 0.05 -12.5
    TTC_PSM3        = TTC_PSM3        * 0.01
    #print("ID3=",ObjID_PSM3)

    return ObjRelAccX_PSM3,ObjRelAccY_PSM3,TTC_PSM3,ObjCls_PSM3,ObjID_PSM3
#    print(pktData,ObjID_PSM3,ObjRelAccX_PSM3,ObjRelAccY_PSM3,TTC_PSM3,ObjCls_PSM3,ObjVec_PSM3,Reserved,ObjUpdtCnt_PSM3,ObjCnt_PSM3)

def decode_PSM_Distance_Info(pktData):
    
    State_S01_PSM      = (pktData[0]&0b11000000) >> 6
    Reli_S01_PSM       = (pktData[0]&0b00111000) >> 3
    Dst_S01_PSM        = (pktData[0]&0b00000111) << 8 | \
                         (pktData[1]&0b11111111) >> 0
    Hght_S01_PSM       = (pktData[2]&0b11100000) >> 5
    State_S02_PSM      = (pktData[2]&0b00011000) >> 3
    Reli_S02_PSM       = (pktData[2]&0b00000111) >> 0
    Dst_S02_PSM        = (pktData[3]&0b11111111) << 3 | \
                         (pktData[4]&0b11100000) >> 5
    Hght_S02_PSM       = (pktData[4]&0b00011100) >> 2
    ObjNewCnt_S01_PSM  = (pktData[4]&0b00000011) >> 0
    ObjNewCnt_S02_PSM  = (pktData[5]&0b11000000) >> 6
    ObjExtpCnt_S01_PSM = (pktData[5]&0b00110000) >> 4
    ObjExtpCnt_S02_PSM = (pktData[5]&0b00001100) >> 2
    Reserved           = (pktData[5]&0b00000011) << 10 | \
                         (pktData[6]&0b11111111) << 2  | \
                         (pktData[7]&0b11000000) >> 6
    ObjUpdtCnt_PSM     = (pktData[7]&0b00110000) >> 4
    ObjCnt_PSM         = (pktData[7]&0b00001111) >> 0

    Dst_S01_PSM = Dst_S01_PSM * 0.01
    Dst_S02_PSM = Dst_S02_PSM * 0.01

    return Dst_S01_PSM,Dst_S02_PSM

def decode_PSM_Unit_Info(pktData):
    
    SIO_state_PSM                  = (pktData[0]&0b11110000) >> 4
    PAS_state_PSM                  = (pktData[0]&0b00001111)
    Reserved                       = (pktData[1]&0b10000000) >> 7
    PrkSns_AreaInfo_FRCenter_PSM   = (pktData[1]&0b01110000) >> 4
    PrkSns_AreaInfo_FRCorner_L_PSM = (pktData[1]&0b00001110) >> 1
    PrkSns_AreaInfo_FRCorner_R_PSM = (pktData[1]&0b00000001) << 2 | \
                                     (pktData[2]&0b11000000) >> 6
    PrkSns_AreaInfo_RRCenter_PSM   = (pktData[2]&0b00111000) >> 3
    PrkSns_AreaInfo_RRCorner_L_PSM = (pktData[2]&0b00000111) >> 0
    PrkSns_AreaInfo_RRCorner_R_PSM = (pktData[3]&0b11100000) >> 5
    PrkSns_AreaInfo_SideL_PSM      = (pktData[3]&0b00011100) >> 2
    PrkSns_AreaInfo_Side_R_PSM     = (pktData[3]&0b00000011) << 1 | \
                                     (pktData[4]&0b10000000) >> 7
    Reserved                       = (pktData[4]&0b01111111) << 20 | \
                                     (pktData[5]&0b11111111) << 12 | \
                                     (pktData[6]&0b11111111) << 4  | \
                                     (pktData[7]&0b11110000) >> 4
    PSMCnt_PSM                     = (pktData[7]&0b00001111)

def decode_wheel_Data(pktData):
    Veh_V_ActlBrk   = ( (pktData[4]&0b11111111)<< 8 )| (pktData[5]&0b11111111)
    
    Veh_V_ActlBrk *=  0.01

    return Veh_V_ActlBrk

def decode_yawrate(pktData):
    Veh_Yaw   = ((pktData[1]&0b00000001)<< 11 )| ((pktData[2]&0b11111111)<<3) | ((pktData[3]&0b11100000)>>5)
    
    Veh_Yaw *=  0.03663
    Veh_Yaw -= 75

    return Veh_Yaw
    

if __name__ == '__main__':
    main()
