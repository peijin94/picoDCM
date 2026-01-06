/* DCM.c
   Version 1

In order to compile this program a few steps are required:

1)  You have to disable the watchdog timers for this firmware to work.  To do
this just comment out all lines of VdInit(); in PROGRAM.lib

2) You need to change "User Defined Lib Directory File" under project options,
compiler, Advanced to:
J:\Firmware\EOVSA\Libraries\LIB.DIR

3)  Replace C:\DCRABBIT_10.70\Lib\Rabbit4000\RS232.lib  with
J:\Firmware\EOVSA\Libraries\RS232.lib

Modifications:

Ver4 9/25/2013:
Added support for interrupt driven table selection.

This is a MODBUS slave device specifically for the the EOVSA Downconverter Module
*/

#define  Firmware_Rev 6

#define 	MODBUS_DEBUG_PRINT 0 // 0xFC
#define 	MODBUS_SLAVE_DEBUG nodebug
#define 	USE_MODBUS_CRC

//#define 	MY_MODBUS_ADDRESS 17
#define  MODBUS_PORT    E
#define  CINBUFSIZE     255
#define  COUTBUFSIZE    255
#define  DINBUFSIZE     255
#define  DOUTBUFSIZE    255
#define  EINBUFSIZE     255
#define  EOUTBUFSIZE    255
#define	BUFFER_LEN	 	132

#define  MODBUS_BAUD    1843200
#define 	INPUT_ONE    LOW   // sets whether a logic one is high or low
#define 	OUTPUT_ONE   LOW   // sets whether a logic one is high or low

#define 	SER_DMA_ONLY
#define 	SERE_TXPORT PEDR
#define 	SERE_RXPORT PEDR

#define 	SER_NO_FLOWCONTROL
#define 	USE_MODBUS_CRC

#use "DCM_lib.LIB"
#use "Modbus_Slave_DCM.lib"
#use "low_level_modbus_slave.lib"
#use ADC_ADS7870.LIB

main ()
{
   int i;
   clockDoublerOff();
   brdInit();
   MODBUS_Serial_Init();
   while (1){
         MODBUS_Serial_tick();
   }
}