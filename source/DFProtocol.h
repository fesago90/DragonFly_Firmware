/* 
 * File:   DFProtocol.h
 * Author: Felipe
 *
 * Created on March 29, 2013, 4:31 PM
 */

#ifndef DFPROTOCOL_H
#define	DFPROTOCOL_H

#ifdef	__cplusplus
extern "C" {
#endif

#include "DFPackets.h"
#include "DFCommands.h"
#include "Camera.h"
    
    void DFP_Continue();
    void DFP_Set_Identity();

    void FC_Set_Wing_Sequence(int8_t wing);
    void FC_Set_RC_Calibration(int8_t rc);

#ifdef	__cplusplus
}
#endif

#endif	/* DFPROTOCOL_H */
