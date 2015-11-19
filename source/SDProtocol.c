#include "Common.h"
#include "SDProtocol.h"
#include "WiRaComm.h"
#include "VMeas.h"
#include "Motors.h"
#include "FlightControl.h"
#include "Camera.h"

//extern TJ_PID RollPID, PitchPID, YawPID;
//extern TJ_PID xPID, yPID, zPID;
extern TJ_PID PIDPitch, PIDRoll,PIDYaw;

static SDPacket GlobalPacket;
static uint8_t TXSeq = 123;

void SDP_Process_Command(SDCommand *cmd);
void SDP_Send_Packet();
void SDP_Build_State_Packet();
inline uint8_t SDP_Next_Seq();
inline void SDP_Set_Default_Toggle_States();

void SDP_Process_Set_Gains_Command(SDSetGainsCmd* cmd);


/*
 * This function is to be called to allow the SD to emit packets or
 * process any packets from the base station.
 */

void SDP_Continue()
{
    if (wrflags.rx_packet_ready) {
        SDP_Process_Command((SDCommand*)WR_Get_RX_Payload());
        wrflags.rx_packet_ready = 0;
    } else {
        if (!wrflags.tx_packet_ready && (PacketState == STATE_PACKET)) {
            GlobalPacket.head.Seq = SDP_Next_Seq();
            SDP_Build_State_Packet();
        }
    }
    if (!wrflags.tx_packet_ready) {
        SDP_Send_Packet();
    }
}

/*
 * This function parses a command packet and executes it. If the command requires
 * a response, a packet is assembled in the GlobalPacket variable for transmitting.
 */
void SDP_Process_Command(SDCommand *cmd)
{
    asm("push DSRPAG");
    asm("movpag #0x0001, DSRPAG");

    LED1_W ^= 1;
    
    switch(cmd->head.CID) {
        case SDControlCmdID:
            FC_Set_Roll_Reference(((SDControlCmd*)cmd)->roll_set);
            FC_Set_Pitch_Reference(((SDControlCmd*)cmd)->pitch_set);
            FC_Set_Yaw_Reference(((SDControlCmd*)cmd)->yaw_set);
            FC_Set_Throttle_Reference(((SDControlCmd*)cmd)->throttle_set);
            break;
        case SDSetGainsCmdID:
            SDP_Process_Set_Gains_Command((SDSetGainsCmd*)cmd);
            break;

        case SDTrimCmdID:
            FC_Set_Roll_Trim(((SDTrimCmd*)cmd)->RollTrim);
            FC_Set_Pitch_Trim(((SDTrimCmd*)cmd)->PitchTrim);
            FC_Set_Yaw_Trim(((SDTrimCmd*)cmd)->YawTrim);
            break;

        case SDSaturationCmdID:
            FC_Set_Main_Motor_Power(((SDSaturationCmd*)cmd)->main_motor);
            FC_Set_Servo_Nominal(((SDSaturationCmd*)cmd)->servo_norm);
            FC_Set_Servo_Saturation(((SDSaturationCmd*)cmd)->servo_satu);
            FC_Set_Pitch_Saturation(((SDSaturationCmd*)cmd)->pitch_satu);
            FC_Set_Yaw_Saturation(((SDSaturationCmd*)cmd)->yaw_satu);
            break;

        case SDServoTrimCmdID:
            FC_Set_Servo_FLTrim(((SDServoTrimCmd*)cmd)->fl);
            FC_Set_Servo_FRTrim(((SDServoTrimCmd*)cmd)->fr);
            FC_Set_Servo_RLTrim(((SDServoTrimCmd*)cmd)->rl);
            FC_Set_Servo_RRTrim(((SDServoTrimCmd*)cmd)->rr);
            break;

        case SDServoMinCmdID:
            FC_Set_Servo_FLMin(((uint16_t)((SDServoMinCmd*)cmd)->fl_high << 8) | ((SDServoMinCmd*)cmd)->fl_low);
            FC_Set_Servo_FRMin(((uint16_t)((SDServoMinCmd*)cmd)->fr_high << 8) | ((SDServoMinCmd*)cmd)->fr_low);
            FC_Set_Servo_RLMin(((uint16_t)((SDServoMinCmd*)cmd)->rl_high << 8) | ((SDServoMinCmd*)cmd)->rl_low);
            FC_Set_Servo_RRMin(((uint16_t)((SDServoMinCmd*)cmd)->rr_high << 8) | ((SDServoMinCmd*)cmd)->rr_low);
            break;

        case SDServoMaxCmdID:
            FC_Set_Servo_FLMax(((uint16_t)((SDServoMaxCmd*)cmd)->fl_high << 8) | ((SDServoMaxCmd*)cmd)->fl_low);
            FC_Set_Servo_FRMax(((uint16_t)((SDServoMaxCmd*)cmd)->fr_high << 8) | ((SDServoMaxCmd*)cmd)->fr_low);
            FC_Set_Servo_RLMax(((uint16_t)((SDServoMaxCmd*)cmd)->rl_high << 8) | ((SDServoMaxCmd*)cmd)->rl_low);
            FC_Set_Servo_RRMax(((uint16_t)((SDServoMaxCmd*)cmd)->rr_high << 8) | ((SDServoMaxCmd*)cmd)->rr_low);
            break;

        case SDToggleFilterCmdID:
            FC_Toggle_Filter();
            break;

        case SDGetOneFrameCmdID:
            CAM_Send_One_Frame();
            break;

        case SDToggleCameraEnable:
            CAM_Toggle_Enable();
            break;

        case SDSetTogglesCmdID:
            SDP_Set_Default_Toggle_States();
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
void SDP_Send_Packet()
{
    static int i;

    WRPacket* rfp = WR_Get_TX_Payload();

    uint8_t *payload = rfp->bytes;
    uint8_t *packet = GlobalPacket.bytes;

    for (i = 0; i < 32; i++)
    {
        payload[i] = packet[i];
    }

    wrflags.tx_packet_ready = 1;
}

inline uint8_t SDP_Next_Seq()
{
    return TXSeq += 2;
}

void SDP_Build_State_Packet()
{
    _DSRPAG = 0x1;
    VoltageData *vd = VMeas_Get_Current_Data();
    AccData* ac     = FC_Get_Current_Filtered_Accel();
    GyroData *gy    = FC_Get_Current_Filtered_Gyro();
    MotorPWM *mpwm  = MC_Get_Motor_PWM();
    SDStatePacket *st = (SDStatePacket*)&GlobalPacket;

    st->head.PID = SDStatePacketID;

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

void SDP_Process_Set_Gains_Command(SDSetGainsCmd* cmd)
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

inline void SDP_Set_Default_Toggle_States()
{
    CAMReadNextFrame = 0;
    FCFilterIsEnabled = 1;
}
