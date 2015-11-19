#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/MARC-basic-firmware-master.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/MARC-basic-firmware-master.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=source/traps.c source/Accel.c source/Gyro.c source/RCComm.c source/VMeas.c source/FlightControl.c source/LED.c source/Debug.c source/Motors.c source/FilterCoeffs.c source/DebugTiming.c source/I2C.c source/WiRaComm.c source/SDProtocol.c source/main.c source/Camera.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/source/traps.o ${OBJECTDIR}/source/Accel.o ${OBJECTDIR}/source/Gyro.o ${OBJECTDIR}/source/RCComm.o ${OBJECTDIR}/source/VMeas.o ${OBJECTDIR}/source/FlightControl.o ${OBJECTDIR}/source/LED.o ${OBJECTDIR}/source/Debug.o ${OBJECTDIR}/source/Motors.o ${OBJECTDIR}/source/FilterCoeffs.o ${OBJECTDIR}/source/DebugTiming.o ${OBJECTDIR}/source/I2C.o ${OBJECTDIR}/source/WiRaComm.o ${OBJECTDIR}/source/SDProtocol.o ${OBJECTDIR}/source/main.o ${OBJECTDIR}/source/Camera.o
POSSIBLE_DEPFILES=${OBJECTDIR}/source/traps.o.d ${OBJECTDIR}/source/Accel.o.d ${OBJECTDIR}/source/Gyro.o.d ${OBJECTDIR}/source/RCComm.o.d ${OBJECTDIR}/source/VMeas.o.d ${OBJECTDIR}/source/FlightControl.o.d ${OBJECTDIR}/source/LED.o.d ${OBJECTDIR}/source/Debug.o.d ${OBJECTDIR}/source/Motors.o.d ${OBJECTDIR}/source/FilterCoeffs.o.d ${OBJECTDIR}/source/DebugTiming.o.d ${OBJECTDIR}/source/I2C.o.d ${OBJECTDIR}/source/WiRaComm.o.d ${OBJECTDIR}/source/SDProtocol.o.d ${OBJECTDIR}/source/main.o.d ${OBJECTDIR}/source/Camera.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/source/traps.o ${OBJECTDIR}/source/Accel.o ${OBJECTDIR}/source/Gyro.o ${OBJECTDIR}/source/RCComm.o ${OBJECTDIR}/source/VMeas.o ${OBJECTDIR}/source/FlightControl.o ${OBJECTDIR}/source/LED.o ${OBJECTDIR}/source/Debug.o ${OBJECTDIR}/source/Motors.o ${OBJECTDIR}/source/FilterCoeffs.o ${OBJECTDIR}/source/DebugTiming.o ${OBJECTDIR}/source/I2C.o ${OBJECTDIR}/source/WiRaComm.o ${OBJECTDIR}/source/SDProtocol.o ${OBJECTDIR}/source/main.o ${OBJECTDIR}/source/Camera.o

# Source Files
SOURCEFILES=source/traps.c source/Accel.c source/Gyro.c source/RCComm.c source/VMeas.c source/FlightControl.c source/LED.c source/Debug.c source/Motors.c source/FilterCoeffs.c source/DebugTiming.c source/I2C.c source/WiRaComm.c source/SDProtocol.c source/main.c source/Camera.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/MARC-basic-firmware-master.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33EP512MC806
MP_LINKER_FILE_OPTION=,--script="p33EP512MC806_MBASIC.gld"
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/source/traps.o: source/traps.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/traps.o.d 
	@${RM} ${OBJECTDIR}/source/traps.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/traps.c  -o ${OBJECTDIR}/source/traps.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/traps.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/traps.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/Accel.o: source/Accel.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/Accel.o.d 
	@${RM} ${OBJECTDIR}/source/Accel.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/Accel.c  -o ${OBJECTDIR}/source/Accel.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/Accel.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/Accel.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/Gyro.o: source/Gyro.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/Gyro.o.d 
	@${RM} ${OBJECTDIR}/source/Gyro.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/Gyro.c  -o ${OBJECTDIR}/source/Gyro.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/Gyro.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/Gyro.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/RCComm.o: source/RCComm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/RCComm.o.d 
	@${RM} ${OBJECTDIR}/source/RCComm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/RCComm.c  -o ${OBJECTDIR}/source/RCComm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/RCComm.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/RCComm.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/VMeas.o: source/VMeas.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/VMeas.o.d 
	@${RM} ${OBJECTDIR}/source/VMeas.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/VMeas.c  -o ${OBJECTDIR}/source/VMeas.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/VMeas.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/VMeas.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/FlightControl.o: source/FlightControl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/FlightControl.o.d 
	@${RM} ${OBJECTDIR}/source/FlightControl.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/FlightControl.c  -o ${OBJECTDIR}/source/FlightControl.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/FlightControl.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/FlightControl.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/LED.o: source/LED.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/LED.o.d 
	@${RM} ${OBJECTDIR}/source/LED.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/LED.c  -o ${OBJECTDIR}/source/LED.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/LED.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/LED.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/Debug.o: source/Debug.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/Debug.o.d 
	@${RM} ${OBJECTDIR}/source/Debug.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/Debug.c  -o ${OBJECTDIR}/source/Debug.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/Debug.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/Debug.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/Motors.o: source/Motors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/Motors.o.d 
	@${RM} ${OBJECTDIR}/source/Motors.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/Motors.c  -o ${OBJECTDIR}/source/Motors.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/Motors.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/Motors.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/FilterCoeffs.o: source/FilterCoeffs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/FilterCoeffs.o.d 
	@${RM} ${OBJECTDIR}/source/FilterCoeffs.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/FilterCoeffs.c  -o ${OBJECTDIR}/source/FilterCoeffs.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/FilterCoeffs.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/FilterCoeffs.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/DebugTiming.o: source/DebugTiming.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/DebugTiming.o.d 
	@${RM} ${OBJECTDIR}/source/DebugTiming.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/DebugTiming.c  -o ${OBJECTDIR}/source/DebugTiming.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/DebugTiming.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/DebugTiming.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/I2C.o: source/I2C.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/I2C.o.d 
	@${RM} ${OBJECTDIR}/source/I2C.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/I2C.c  -o ${OBJECTDIR}/source/I2C.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/I2C.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/I2C.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/WiRaComm.o: source/WiRaComm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/WiRaComm.o.d 
	@${RM} ${OBJECTDIR}/source/WiRaComm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/WiRaComm.c  -o ${OBJECTDIR}/source/WiRaComm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/WiRaComm.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/WiRaComm.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/SDProtocol.o: source/SDProtocol.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/SDProtocol.o.d 
	@${RM} ${OBJECTDIR}/source/SDProtocol.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/SDProtocol.c  -o ${OBJECTDIR}/source/SDProtocol.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/SDProtocol.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/SDProtocol.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/main.o: source/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/main.o.d 
	@${RM} ${OBJECTDIR}/source/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/main.c  -o ${OBJECTDIR}/source/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/main.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/Camera.o: source/Camera.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/Camera.o.d 
	@${RM} ${OBJECTDIR}/source/Camera.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/Camera.c  -o ${OBJECTDIR}/source/Camera.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/Camera.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/Camera.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
else
${OBJECTDIR}/source/traps.o: source/traps.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/traps.o.d 
	@${RM} ${OBJECTDIR}/source/traps.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/traps.c  -o ${OBJECTDIR}/source/traps.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/traps.o.d"      -mno-eds-warn  -g -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/traps.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/Accel.o: source/Accel.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/Accel.o.d 
	@${RM} ${OBJECTDIR}/source/Accel.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/Accel.c  -o ${OBJECTDIR}/source/Accel.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/Accel.o.d"      -mno-eds-warn  -g -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/Accel.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/Gyro.o: source/Gyro.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/Gyro.o.d 
	@${RM} ${OBJECTDIR}/source/Gyro.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/Gyro.c  -o ${OBJECTDIR}/source/Gyro.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/Gyro.o.d"      -mno-eds-warn  -g -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/Gyro.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/RCComm.o: source/RCComm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/RCComm.o.d 
	@${RM} ${OBJECTDIR}/source/RCComm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/RCComm.c  -o ${OBJECTDIR}/source/RCComm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/RCComm.o.d"      -mno-eds-warn  -g -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/RCComm.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/VMeas.o: source/VMeas.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/VMeas.o.d 
	@${RM} ${OBJECTDIR}/source/VMeas.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/VMeas.c  -o ${OBJECTDIR}/source/VMeas.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/VMeas.o.d"      -mno-eds-warn  -g -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/VMeas.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/FlightControl.o: source/FlightControl.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/FlightControl.o.d 
	@${RM} ${OBJECTDIR}/source/FlightControl.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/FlightControl.c  -o ${OBJECTDIR}/source/FlightControl.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/FlightControl.o.d"      -mno-eds-warn  -g -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/FlightControl.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/LED.o: source/LED.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/LED.o.d 
	@${RM} ${OBJECTDIR}/source/LED.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/LED.c  -o ${OBJECTDIR}/source/LED.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/LED.o.d"      -mno-eds-warn  -g -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/LED.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/Debug.o: source/Debug.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/Debug.o.d 
	@${RM} ${OBJECTDIR}/source/Debug.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/Debug.c  -o ${OBJECTDIR}/source/Debug.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/Debug.o.d"      -mno-eds-warn  -g -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/Debug.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/Motors.o: source/Motors.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/Motors.o.d 
	@${RM} ${OBJECTDIR}/source/Motors.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/Motors.c  -o ${OBJECTDIR}/source/Motors.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/Motors.o.d"      -mno-eds-warn  -g -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/Motors.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/FilterCoeffs.o: source/FilterCoeffs.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/FilterCoeffs.o.d 
	@${RM} ${OBJECTDIR}/source/FilterCoeffs.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/FilterCoeffs.c  -o ${OBJECTDIR}/source/FilterCoeffs.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/FilterCoeffs.o.d"      -mno-eds-warn  -g -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/FilterCoeffs.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/DebugTiming.o: source/DebugTiming.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/DebugTiming.o.d 
	@${RM} ${OBJECTDIR}/source/DebugTiming.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/DebugTiming.c  -o ${OBJECTDIR}/source/DebugTiming.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/DebugTiming.o.d"      -mno-eds-warn  -g -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/DebugTiming.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/I2C.o: source/I2C.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/I2C.o.d 
	@${RM} ${OBJECTDIR}/source/I2C.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/I2C.c  -o ${OBJECTDIR}/source/I2C.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/I2C.o.d"      -mno-eds-warn  -g -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/I2C.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/WiRaComm.o: source/WiRaComm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/WiRaComm.o.d 
	@${RM} ${OBJECTDIR}/source/WiRaComm.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/WiRaComm.c  -o ${OBJECTDIR}/source/WiRaComm.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/WiRaComm.o.d"      -mno-eds-warn  -g -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/WiRaComm.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/SDProtocol.o: source/SDProtocol.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/SDProtocol.o.d 
	@${RM} ${OBJECTDIR}/source/SDProtocol.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/SDProtocol.c  -o ${OBJECTDIR}/source/SDProtocol.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/SDProtocol.o.d"      -mno-eds-warn  -g -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/SDProtocol.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/main.o: source/main.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/main.o.d 
	@${RM} ${OBJECTDIR}/source/main.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/main.c  -o ${OBJECTDIR}/source/main.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/main.o.d"      -mno-eds-warn  -g -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/main.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
${OBJECTDIR}/source/Camera.o: source/Camera.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} ${OBJECTDIR}/source 
	@${RM} ${OBJECTDIR}/source/Camera.o.d 
	@${RM} ${OBJECTDIR}/source/Camera.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  source/Camera.c  -o ${OBJECTDIR}/source/Camera.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MMD -MF "${OBJECTDIR}/source/Camera.o.d"      -mno-eds-warn  -g -omf=elf -O0 -I"../../../../../../../../Program Files (x86)/Microchip/xc16/v1.11/support/generic" -msmart-io=1 -Wall -msfr-warn=off
	@${FIXDEPS} "${OBJECTDIR}/source/Camera.o.d" $(SILENT)  -rsi ${MP_CC_DIR}../ 
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/MARC-basic-firmware-master.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    p33EP512MC806_MBASIC.gld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/MARC-basic-firmware-master.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -Wl,--local-stack,--defsym=__MPLAB_BUILD=1,--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_ICD3=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--library=dsp,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST) 
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/MARC-basic-firmware-master.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   p33EP512MC806_MBASIC.gld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/MARC-basic-firmware-master.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -Wl,--local-stack,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--library=dsp,--no-force-link,--smart-io,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map",--report-mem$(MP_EXTRA_LD_POST) 
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/MARC-basic-firmware-master.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf  
	
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif
