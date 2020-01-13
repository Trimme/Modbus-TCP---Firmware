/*
===============================================================================
 Name        : modbus_tcp.h
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : Modbus TCP Implementation for NXP LPC176xx and Wiznet W5500
===============================================================================
*/

#ifndef _MODBUS_TCP_H_
#define	_MODBUS_TCP_H_

/* ------------------------ Defines --------------------------------------- */
#define REG_DISCRETE_START    10001         // Start address of discrete inputs
#define REG_DISCRETE_SIZE     15            // Number of discrete inputs

#define REG_COILS_START       00001         // Coil start address
#define REG_COILS_SIZE        9             // Number of coils

#define REG_INPUT_START       30001         // Input register start address
#define REG_INPUT_NREGS       10            // Number of input registers

#define REG_HOLDING_START     40001         // Holding register start address
#define REG_HOLDING_NREGS     1             // Number of holding registers

/* Coils */
#define COIL_START             00001
#define COIL_QUICKSTOP         00002
#define COIL_REVERSE           00003
#define COIL_EN_SWASHREG       00004
#define COIL_EN_PRESSREG       00005
#define COIL_DIG1              00006
#define COIL_DIG2              00007
#define COIL_DIG3              00008
#define COIL_DIG4              00009

/* Discrete Inputs */
#define DISCRETE_WARN          10001
#define DISCRETE_ALARM         10002
#define DISCRETE_TEMP_WARN     10003
#define DISCRETE_DIG1          10004
#define DISCRETE_DIG2          10005
#define DISCRETE_DIG3          10006
#define DISCRETE_DIG4          10007
#define DISCRETE_DIG5          10008
#define DISCRETE_DIG6          10009
#define DISCRETE_DIG7          10010
#define DISCRETE_DIG8          10011
#define DISCRETE_DIG9          10012
#define DISCRETE_DIG10         10013
#define DISCRETE_DIG11         10014
#define DISCRETE_DIG12         10015

/* Input Registers */
#define INPUT_REG_PUMPCRNT     30001
#define INPUT_REG_SWASHANGLE   30002
#define INPUT_REG_HIPRESS      30003
#define INPUT_REG_PT100        30004
#define INPUT_REG_ANALOG1      30005
#define INPUT_REG_ANALOG2      30006
#define INPUT_REG_ANALOG3      30007
#define INPUT_REG_ANALOG4      30008
#define INPUT_REG_ANALOG5      30009
#define INPUT_REG_ANALOG6      30010

/* Holding Registers */
#define HOLD_SETPOINT          40001

#endif /* _MODBUS_TCP_H_ */
