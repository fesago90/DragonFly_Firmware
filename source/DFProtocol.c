#include "Common.h"
#include "DFProtocol.h"
#include "RFComm.h"
#include "VMeas.h"
#include "Motors.h"
#include "FlightControl.h"

//extern TJ_PID RollPID, PitchPID, YawPID;
//extern TJ_PID xPID, yPID, zPID;
extern TJ_PID PIDPitch, PIDRoll,PIDYaw;

DFPacket GlobalPacket;
uint8_t TXSeq = 123;

void DFP_Process_Command(DFCommand *cmd);
void DFP_Send_Packet();
void DFP_Build_State_Packet();
void DFP_Build_Camera_Packet();
void DFP_Build_Camera_Start_Frame_Packet();
void DFP_Build_Camera_End_Frame_Packet();
inline uint8_t DFP_Next_Seq();
inline void DFP_Set_Default_Toggle_States();

void DFP_Process_Set_Gains_Command(DFSetGainsCmd* cmd);


/*
 * This function is to be called to allow the DF to emit packets or
 * process any packets from the base station.
 */
void DFP_Continue()
{
    if (RFDataReady) {
        RF_Receive_Data();
        DFP_Process_Command((DFCommand*)RF_Get_RX_Payload());
    } else {
        if (RFReadyForTX && (PacketState == STATE_PACKET)) {
            GlobalPacket.head.Seq = DFP_Next_Seq();
            DFP_Build_State_Packet();
        }
        else if (RFReadyForTX && (PacketState == CAMERA_START_FRAME_PACKET)) {
            GlobalPacket.head.Seq = DFP_Next_Seq();
            DFP_Build_Camera_Start_Frame_Packet();
        }
        else if (RFReadyForTX && (PacketState == CAMERA_PACKET)) {
            GlobalPacket.head.Seq = DFP_Next_Seq();
            DFP_Build_Camera_Packet();
        }
        else if (RFReadyForTX && (PacketState == CAMERA_END_FRAME_PACKET)) {
            GlobalPacket.head.Seq = DFP_Next_Seq();
            DFP_Build_Camera_End_Frame_Packet();
        }
    }
    if (RFReadyForTX) {
        DFP_Send_Packet();
    }
}

/*
 * This function parses a command packet and executes it. If the command requires
 * a response, a packet is assembled in the GlobalPacket variable for transmitting.
 */
void DFP_Process_Command(DFCommand *cmd)
{
    asm("push DSRPAG");
    asm("movpag #0x0001, DSRPAG");

    LED1_W ^= 1;

    switch(cmd->head.CID) {
        case DFMoveCmdID:
            FC_Set_Throttle_Reference(((DFMoveCmd*)cmd)->ThrottleRef);
            break;

        case DFSetGainsCmdID:
            DFP_Process_Set_Gains_Command((DFSetGainsCmd*)cmd);
            break;

        case DFTrimCmdID:
            FC_Set_Roll_Trim(((DFTrimCmd*)cmd)->RollTrim);
            FC_Set_Pitch_Trim(((DFTrimCmd*)cmd)->PitchTrim);
            FC_Set_Yaw_Trim(((DFTrimCmd*)cmd)->YawTrim);
            break;

        case DFSettingCmdID:
            FC_Set_Wing_Sequence(((DFSettingCmd*)cmd)->WingSeq);
            FC_Set_RC_Calibration(((DFSettingCmd*)cmd)->RCCal);
            break;

        case DFSaturationCmdID:
            FC_Set_Main_Motor_Power(((DFSaturationCmd*)cmd)->main_motor);
            FC_Set_Servo_Nominal(((DFSaturationCmd*)cmd)->servo_norm);
            FC_Set_Servo_Saturation(((DFSaturationCmd*)cmd)->servo_satu);
            FC_Set_Pitch_Saturation(((DFSaturationCmd*)cmd)->pitch_satu);
            FC_Set_Yaw_Saturation(((DFSaturationCmd*)cmd)->yaw_satu);
            break;

        case DFServoTrimCmdID:
            FC_Set_Servo_FLTrim(((DFServoTrimCmd*)cmd)->fl);
            FC_Set_Servo_FRTrim(((DFServoTrimCmd*)cmd)->fr);
            FC_Set_Servo_RLTrim(((DFServoTrimCmd*)cmd)->rl);
            FC_Set_Servo_RRTrim(((DFServoTrimCmd*)cmd)->rr);
            break;

        case DFServoMinCmdID:
            FC_Set_Servo_FLMin(((uint16_t)((DFServoMinCmd*)cmd)->fl_high << 8) | ((DFServoMinCmd*)cmd)->fl_low);
            FC_Set_Servo_FRMin(((uint16_t)((DFServoMinCmd*)cmd)->fr_high << 8) | ((DFServoMinCmd*)cmd)->fr_low);
            FC_Set_Servo_RLMin(((uint16_t)((DFServoMinCmd*)cmd)->rl_high << 8) | ((DFServoMinCmd*)cmd)->rl_low);
            FC_Set_Servo_RRMin(((uint16_t)((DFServoMinCmd*)cmd)->rr_high << 8) | ((DFServoMinCmd*)cmd)->rr_low);
            break;

        case DFServoMaxCmdID:
            FC_Set_Servo_FLMax(((uint16_t)((DFServoMaxCmd*)cmd)->fl_high << 8) | ((DFServoMaxCmd*)cmd)->fl_low);
            FC_Set_Servo_FRMax(((uint16_t)((DFServoMaxCmd*)cmd)->fr_high << 8) | ((DFServoMaxCmd*)cmd)->fr_low);
            FC_Set_Servo_RLMax(((uint16_t)((DFServoMaxCmd*)cmd)->rl_high << 8) | ((DFServoMaxCmd*)cmd)->rl_low);
            FC_Set_Servo_RRMax(((uint16_t)((DFServoMaxCmd*)cmd)->rr_high << 8) | ((DFServoMaxCmd*)cmd)->rr_low);
            break;

        case DFToggleFilterCmdID:
            FC_Toggle_Filter();
            break;

        case DFGetOneFrameCmdID:
            CAM_Send_One_Frame();
            break;

        case DFToggleCameraEnable:
            CAM_Toggle_Enable();
            break;

        case DFSetTogglesCmdID:
            DFP_Set_Default_Toggle_States();
            break;
            
        default:
            break;
    }

    asm("pop DSRPAG");
}

/*
 * This funtion provides the RF controller with the packet to be sent and attempts
 * to start the RF transmission
 */
void DFP_Send_Packet()
{
    static int i;

    RFPayload* rfp = RF_Get_TX_Payload();

    uint8_t *payload = rfp->bytes;
    uint8_t *packet = GlobalPacket.bytes;

    for (i = 0; i < 32; i++)
    {
        payload[i] = packet[i];
    }

    RF_Try_Start_Packet_TX();
}

inline uint8_t DFP_Next_Seq()
{
    return TXSeq += 2;
}

void DFP_Build_State_Packet()
{
    _DSRPAG = 0x1;
    VoltageData *vd = VMeas_Get_Current_Data();
    AccData* ac     = FC_Get_Current_Filtered_Accel();
    GyroData *gy    = FC_Get_Current_Filtered_Gyro();
    MotorPWM *mpwm  = MC_Get_Motor_PWM();
    DFStatePacket *st = (DFStatePacket*)&GlobalPacket;

    st->head.PID = DFStatePacketID;

    st->Acc_X = ac->Acc_X;
    st->Acc_Y = ac->Acc_Y;
    st->Acc_Z = ac->Acc_Z;

    st->Gy_X = gy->Gy_X;
    st->Gy_Y = gy->Gy_Y;
    st->Gy_Z = gy->Gy_Z;

    st->V_Batt   = vd->V_Batt;
    st->V_Servo  = vd->V_Servo;
    st->V_Supply = vd->V_Supply;

    st->M_FrontLeft  = (int)(mpwm->MC_FrontLeft/PWM_MULT);               //12000 max pwm value?
    st->M_FrontRight = (int)(mpwm->MC_FrontRight/PWM_MULT);
    st->M_RearLeft   = (int)(mpwm->MC_RearLeft/PWM_MULT);
    st->M_RearRight  = (int)(mpwm->MC_RearRight/PWM_MULT);

}

void DFP_Build_Camera_Start_Frame_Packet()
{
    _DSRPAG = 0x1;

    DFStartFramePacket *sf = (DFStartFramePacket*)&GlobalPacket;

    sf->head.PID = DFStartFramePacketID;

    sf->numCols = COLS;
    sf->numRows = ROWS;
    sf->SegmentLength = SEGMENT_LENGTH;

    PacketState = CAMERA_PACKET;
}

void DFP_Build_Camera_Packet()
{
    _DSRPAG = 0x1;

    DFCameraPacket *cam = (DFCameraPacket*)&GlobalPacket;

    cam->head.PID = DFCameraPacketID;

    cam->CAM_SegmentID = CAM_Get_Next_Segment_ID();

    CAM_Fill_TX_Segment(&(cam->CAM_PixelData));

    CAM_Change_DF_State();
}

void DFP_Build_Camera_End_Frame_Packet()
{
    _DSRPAG = 0x1;

    DFEndFramePacket *ef = (DFEndFramePacket*)&GlobalPacket;

    ef->head.PID = DFEndFramePacketID;

    PacketState = STATE_PACKET;
}

void DFP_Process_Set_Gains_Command(DFSetGainsCmd* cmd)
{
    if (cmd->ControlMask & ControlTypeRoll) {
        //FC_PID_Set_Gains(&RollPID, cmd->RollKp, cmd->RollKi, cmd->RollKd);
        //FC_PID_Set_Gains(&yPID, cmd->RollKp, cmd->RollKi, 0);
        //FC_PID_Set_Gains(&RollPID, cmd->RollKd, 0, 0);
        //FC_PID_Set_Gains(&PIDRoll, cmd->RollKp,cmd->RollKd,cmd->RollKi);
        FC_PID_Set_Roll_Gains(&PIDRoll, cmd->RollKp,cmd->RollKi,cmd->RollKd);
    }

    if (cmd->ControlMask & ControlTypePitch) {
        //FC_PID_Set_Gains(&PitchPID, cmd->PitchKp, cmd->PitchKi, cmd->PitchKd);
        //FC_PID_Set_Gains(&xPID, cmd->PitchKp, cmd->PitchKi, 0);
        //FC_PID_Set_Gains(&PitchPID, cmd->PitchKd, 0, 0);
        //FC_PID_Set_Gains(&PIDPitch, cmd->PitchKp,cmd->PitchKd,cmd->PitchKi);
        FC_PID_Set_Pitch_Gains(&PIDPitch, cmd->PitchKp,cmd->PitchKi,cmd->PitchKd);
    }

    if (cmd->ControlMask & ControlTypeYaw) {
        FC_PID_Set_Yaw_Gains(&PIDYaw, cmd->YawKp,cmd->YawKi,cmd->YawKd); 
        //FC_PID_Set_Gains(&YawPID, cmd->YawKp, cmd->YawKi, cmd->YawKd);
    }
}

inline void DFP_Set_Default_Toggle_States()
{
    CAMReadNextFrame = 0;
    FCFilterIsEnabled = 1;
}