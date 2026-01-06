/* START LIBRARY DESCRIPTION *********************************************

DESCRIPTION:	Dynamic C MODBus Slave user functions for a LODM.

Modbus_Slave_LODM					Ver 1.00

Modifications:


This library contains functions specific to the targeted device.  The user
must complete each of the functions in this library based on the hardware
in the system.  These functions are independent of the communications
media being used: RS485, TCP/IP, etc.

*************************************************************************
IMPORTANT:
(1) The user program must execute the necessary functions to set up the board
I/O before starting the Modbus engine:
(2) This library does not support the "special" functions of the RIO:
		Input Capture, Quadrature Decoder, PWM, etc.

The function mbsbrdInit will handle board initialization.
*************************************************************************

The following functions are defined:

mbsbrdInit	initialize the I/O of the board

mbsStart		pre-packet processing - usually an empty function
mbsDone		post-packet processing - usually an empty function

			Single Bit functions
mbsDigOutRd	return the state of a single output [0x01]
mbsDigIn		return the state of a single input [0x02]
mbsDigOut	set the state of a single output [0x05]

			Byte/Word functions
mbsRegOutRd	return the value of a holding register [0x03]
mbsRegIn		return the value of an input register [0x04]
mbsRegOut	set the state of a holding register [0x06]

The following describes the Modbus function codes and how they relate
to the BL4S1xx I/O:

NOTE: The Dynamic C functions which program the digital outputs use a value
of 0 to turn on the output.  The easiest way to remember this is to think in
terms of the voltage level of the output when you need to program it.

mbsDigOutRd:	ModBus function code 0x01 - return the state of an output bit
						 OutputNbr 0..7

mbsDigIn:		MODBUS function code 0x02 - read the state of an input bit
						InputNbr 0..11

mbsRegOutRd:	ModBus function code 0x03 - return the state of output register
						OutputNbr 0..7 = register 0

mbsRegIn:		ModBus function code 0x04 - read an input register
						InputNbr 0..11 = register 0

mbsDigOut:		MODBUS function code 0x05 - set the state of an output bit
						function code 0x0F - set the state of multiple outputs
						OutputNbr 0..7

mbsRegOut:		ModBus function code 0x06 - write to an I/O register
						function code 0x10 - write to multiple I/O registers
						OutputNbr 0..7 = register 0

These are the defined error return values:
MB_SUCCESS		// success
MB_BADFUNC		//	Illegal Function
MB_BADADDR		//	Illegal Data Address
MB_BADDATA		//	Illegal Data Value
MB_BUSY			//	Target is busy with another request
MB_NORESP		//	No response from target
MB_DEVNOTSET	// device not properly set up


These three functions will only be needed if this device is connected to
'downstream' MODBUS devices via a serial interface - probably RS485.

MODBUS_Serial_Init initialize the serial port for 'downstream' devices
MODBUS_Serial_Tx	send a message to a 'downstream' device
MODBUS_Serial_Rx	receive a message from a 'downstream' device

END DESCRIPTION **********************************************************/


/*** BeginHeader */

#ifndef MODBUS_SLAVE_DEBUG
#define MODBUS_SLAVE_DEBUG __nodebug
#endif
/*
#ifndef BYTE_TIME
// maximum number of byte times to wait between received bytes
	#define BYTE_TIME		5
#endif*/

int	tempint;
int	DigOutValue;

#ifndef MODBUS_PORT
	#define MODBUS_PORT	D
#endif

#ifndef MODBUS_BAUD
	#define MODBUS_BAUD	19200
#endif

#define serOpen					CONCAT ( CONCAT(ser,MODBUS_PORT), open )
#define serClose					CONCAT ( CONCAT(ser,MODBUS_PORT), close )
#define serRead					CONCAT ( CONCAT(MB_ser,MODBUS_PORT), read )
#define serRdUsed					CONCAT ( CONCAT(ser,MODBUS_PORT), rdUsed )
#define serWrUsed					CONCAT ( CONCAT(ser,MODBUS_PORT), wrUsed )
#define serWrite					CONCAT ( CONCAT(ser,MODBUS_PORT), write )
#define serPeek					CONCAT ( CONCAT(ser,MODBUS_PORT), peek )
#define serRdFlush				CONCAT ( CONCAT(ser,MODBUS_PORT), rdFlush )
#define serWrFlush				CONCAT ( CONCAT(ser,MODBUS_PORT), wrFlush )
#define serFlowCtrlOn			CONCAT ( CONCAT(ser,MODBUS_PORT), flowCtrlOn )
#define serFlowCtrlOff			CONCAT ( CONCAT(ser,MODBUS_PORT), flowCtrlOff )
#define serPutc					CONCAT ( CONCAT(ser,MODBUS_PORT), putc )

#define serStatusReg				CONCAT ( CONCAT(S,MODBUS_PORT), SR )
#define serParity					CONCAT ( CONCAT(ser,MODBUS_PORT), parity )
#define serdmaOn					CONCAT ( CONCAT(ser,MODBUS_PORT), dmaOn )
#define serdmaOff					CONCAT ( CONCAT(ser,MODBUS_PORT), dmaOff )


/*** EndHeader */


/**********************************************************************
The following are the Function Descriptions for the functions which
MUST be implemented by the customer and in this library.
**********************************************************************/


/* START FUNCTION DESCRIPTION *****************************************
mbsStart						<Modbus_Slave_LODM.LIB>

NOTE: Modbus_Slave_LODM.LIB functions are generally not reentrant.

SYNTAX: void mbsStart()

DESCRIPTION: Called just before a received Modbus packet is processed,
this function is primarily intended to be used to lock resources so
that data returned in one Modbus response packet are atomic. Locking
resources may or may not be required, depending on how the Modbus functions
are implemented in a particular Modbus slave application. Note that Modbus
command handler functions in Modbus_slave_tcp.LIB may make multiple calls
to those functions while responding to a single Modbus command.

RETURN VALUE: None.
END DESCRIPTION ******************************************************/

/*** BeginHeader mbsStart */
void mbsStart	(	void );
/*** EndHeader */

MODBUS_SLAVE_DEBUG
void mbsStart ( void )
{
}


/* START FUNCTION DESCRIPTION *****************************************
mbsDone						<Modbus_Slave_LODM.LIB>

NOTE: Modbus_Slave_LODM.LIB functions are generally not reentrant.

SYNTAX: void mbsDone()

DESCRIPTION: Called just after a received Modbus command has been
processed and just before the reply is sent. This function is intended
to be used to unlock resources that were locked by msStart().  See msStart
for more details.

RETURN VALUE: None.
END DESCRIPTION ******************************************************/

/*** BeginHeader mbsDone */
void mbsDone	(	void );
/*** EndHeader */

MODBUS_SLAVE_DEBUG
void mbsDone ( void )
{
}

/* START FUNCTION DESCRIPTION ********************************************
mbsbrdInit					<Modbus_Slave_LODM.LIB>

SYNTAX:			void mbsbrdInit (void);

DESCRIPTION:	Initialize the I/O of the board

PARAMETER:		none

RETURN VALUE:	none

END DESCRIPTION **********************************************************/

/*** BeginHeader mbsbrdInit*/
void mbsbrdInit (void);
/*** EndHeader */

void mbsbrdInit (void)
{	int i;

	brdInit();						// initialize the board

// you may add any other initialization required by the application here
// or insert it into your main program after executing this function.

}

/* START FUNCTION DESCRIPTION *****************************************
mbsDigOutRd					<Modbus_Slave_LODM.LIB>

NOTE: Modbus_Slave_LODM.LIB functions are generally not reentrant.

ModBus function code = 0x01

SYNTAX: 			int mbsDigOutRd ( unsigned OutputNbr, int *pnState )

DESCRIPTION:   returns the last value written to OutputNbr
						0 = output on - load grounded
                  1 = output off

PARAMETER1:		output number: 0..7

PARAMETER2:		pointer to destination variable

RETURN VALUE:	MB_SUCCESS = success
               MB_BADADDR = illegal Output Nbr
               MB_DEVNOTSET = I/O not set as output

END DESCRIPTION ******************************************************/

/*** BeginHeader mbsDigOutRd */
int mbsDigOutRd ( unsigned OutputNbr, int *pnState );
/*** EndHeader */

MODBUS_SLAVE_DEBUG
int mbsDigOutRd ( unsigned OutputNbr, int *pnState )
{
	if ( OutputNbr > 7 )
	{
		return MB_BADADDR;
	}

	*pnState = (DigOutValue>>OutputNbr)&0x01;
	return MB_SUCCESS;
} // mbsDigOutRd


/* START FUNCTION DESCRIPTION *****************************************
mbsDigIn						<Modbus_Slave_LODM.LIB>

ModBus function code = 0x02

NOTE: Modbus_Slave_LODM.LIB functions are generally not reentrant.

SYNTAX:			int mbsDigIn ( unsigned InputNbr, int *pnState )

DESCRIPTION:	read the specified input

PARAMETER1:		input number: 0..11

PARAMETER2:		pointer to destination variable
						a '1' is returned if the input is high

RETURN VALUE:	MB_SUCCESS = success
					MB_BADADDR = illegal channel
               MB_DEVNOTSET = I/O not set as output

END DESCRIPTION ******************************************************/

/*** BeginHeader mbsDigIn */
int mbsDigIn ( unsigned InputNbr, int *pnState );
/*** EndHeader */

MODBUS_SLAVE_DEBUG
int mbsDigIn ( unsigned InputNbr, int *pnState )
{
	if ( InputNbr != 0)
	{
		return MB_BADADDR;
	}

	*pnState = digIn ( InputNbr );
	return MB_SUCCESS;
} // mbsDigIn


/* START FUNCTION DESCRIPTION *****************************************
mbsRegOutRd					<Modbus_Slave_LODM.LIB>

NOTE: Modbus_Slave_LODM.LIB functions are generally not reentrant.

ModBus function code = 0x03

SYNTAX:			int mbsRegOutRd ( unsigned OutRegNbr, unsigned *pwValue )

DESCRIPTION:	read an 8 bit output register

PARAMETER1:		register number
						0 = value last written of all the outputs

PARAMETER2:		pointer to destination variable
						for each bit: 0 = output is on, 1 = output is off

RETURN VALUE:	MB_SUCCESS = success
					MB_BADADDR = illegal register
               MB_DEVNOTSET = I/O not set as output
END DESCRIPTION ******************************************************/

/*** BeginHeader mbsRegOutRd */
int mbsRegOutRd ( unsigned OutRegNbr, unsigned *pwValue );
/*** EndHeader */

MODBUS_SLAVE_DEBUG
int mbsRegOutRd ( unsigned OutRegNbr, unsigned *pwValue )
{	auto int i, RegValue;

	if ( OutRegNbr > BUFFER_LEN + 52 ) return MB_BADADDR;

  	*pwValue = rd_holding_reg(OutRegNbr);

	return MB_SUCCESS;
} // mbsRegOutRd

/* START FUNCTION DESCRIPTION *****************************************
mbsRegIn						<Modbus_Slave_LODM.LIB>

NOTE: Modbus_Slave_LODM.LIB functions are generally not reentrant.

ModBus function code = 0x04

SYNTAX: 			int mbsRegIn ( unsigned InRegNbr, unsigned *pwValue )

DESCRIPTION:	read an input register: read state of pins (0V = 0)

PARAMETER1:		register number
						0 =  Inputs 0..11
	               Special Registers:
						3nnx = Analog Input where nn = A/D channel (0..7)
                  	3nn0 = execute an A/D read and
                     	returns least significant word of floating point value
                     3nn1 = returns most significant word of floating point value
                     		note: DIFF_MODE and mAMP_MODE only valid for floating point
                     3nn2 = integer millivolts
                     3003 = integer raw value
	               see mbsRegOut for setting up the A/D channels

PARAMETER2:		pointer to destination variable

RETURN VALUE:	MB_SUCCESS = success
					MB_BADADDR = illegal channel or function
               MB_DEVNOTSET = board not initialized
               MB_BADFUNC = invalid function
END DESCRIPTION ******************************************************/

/*** BeginHeader mbsRegIn */
int mbsRegIn ( unsigned InRegNbr, unsigned *pwValue );
/*** EndHeader */

MODBUS_SLAVE_DEBUG
int mbsRegIn ( unsigned InRegNbr, unsigned *pwValue )
{	//auto int channel, function, temp, DiffFlag, maFlag;

	if ( InRegNbr > 1 && InRegNbr < 65535) return MB_BADADDR;

   *pwValue = digInBank( InRegNbr );

	return MB_SUCCESS;
} // mbsRegIn


/* START FUNCTION DESCRIPTION ********************************************
mbsDigOut						<Modbus_Slave_LODM.LIB>

NOTE: Modbus_Slave_LODM.LIB functions are generally not reentrant.

MODBUS command = 0x05, 0x0F

SYNTAX:     	int mbsdigOut ( unsigned OutputNbr, int state );

DESCRIPTION:	turn the specified output on or off

PARAMETER1:		output channel number
						0 <= channel <= 7

PARAMETER2:		output state
						0 = turn output on - connects load to ground
						1 = turn output off - high impedance state

RETURN VALUE:	MB_SUCCESS = success
					MB_BADADDR = illegal channel
               MB_BADDATA = illegal data value
               MB_DEVNOTSET = I/O not set as output

SEE ALSO: 		brdInit, digOutConfig

END DESCRIPTION **********************************************************/

/*** BeginHeader mbsDigOut */
int mbsDigOut ( unsigned OutputNbr, int state );
/*** EndHeader */

MODBUS_SLAVE_DEBUG
int mbsDigOut ( unsigned OutputNbr, int state )
{	auto mask;

	if ( OutputNbr > 1 )
	{
		return MB_BADADDR;
	}

   // added the below line to make sure if not 0 set to 1
   state = (state!=0?1:0);

	digOut ( OutputNbr, state );

   mask = (1<<OutputNbr);
   if (state) DigOutValue |= mask;
   else DigOutValue &= ~mask;

   return MB_SUCCESS;
} // mbsDigOut


/* START FUNCTION DESCRIPTION *****************************************
mbsRegOut						<Modbus_Slave_LODM.LIB>

NOTE: Modbus_Slave_LODM.LIB functions are generally not reentrant.

ModBus function codes = 0x06, 0x10, 0x16 and 0x17,

SYNTAX: 			int mbsRegOut ( unsigned OutRegNbr, unsigned wValue )

DESCRIPTION: 	write to an I/O register

PARAMETER1:		register number
						0 = Digital outputs 0..7
                  writes all outputs at one time

PARAMETER2:		register value
					for DIO: each bit (0..7)
						0 = turn output off
                  1 = turn output on

RETURN VALUE:	MB_SUCCESS = success
					MB_BADADDR = illegal channel
               MB_DEVNOTSET = I/O not set as output
               MB_BADFUNC = invalid function
END DESCRIPTION ******************************************************/

/*** BeginHeader mbsRegOut */
int mbsRegOut ( unsigned OutRegNbr, unsigned wValue );
/*** EndHeader */

MODBUS_SLAVE_DEBUG
int mbsRegOut ( unsigned OutRegNbr, unsigned wValue )
{  //unsigned int data;

   // A union split up int to char's to see if data is valid
	union {
	   unsigned char chr_val[2];
	   unsigned int int_val;
	} conv_int;

   conv_int.int_val = wValue;

	if (OutRegNbr > 52) return MB_BADADDR;

	if (OutRegNbr == 0 || OutRegNbr > 2){ // digital attenuator value checks
   	if (conv_int.chr_val[0] > 15 || conv_int.chr_val[1] > 15) return MB_BADDATA;
   } else if (OutRegNbr == 2){
       //GN wValue may have an nonzero upper byte
		//if (wValue > 49) return MB_BADDATA;
   }
	digOutBank ( OutRegNbr, wValue );

	return MB_SUCCESS;
} // mbsRegOut


/* START FUNCTION DESCRIPTION ********************************************
MODBUS_Serial_Init		<Modbus_Slave_LODM.LIB>

SYNTAX:        int MODBUS_Serial_Init ();

DESCRIPTION:	Initialize the serial port
					Requires the following macros:
               	MODBUS_PORT
               	MODBUS_BAUD		desired baud rate

					Calculate Serial_timeout, used by MODBUS_Serial_Rx,
					as the timeout between bytes once a byte has been
					received: 5 byte times or 2msec, whichever is greater.

PARAMETER1:		none

RETURN VALUE:	MB_SUCCESS
					MB_BADDATA if illegal SERIAL_MODE

END DESCRIPTION **********************************************************/

/*** BeginHeader MODBUS_Serial_Init, Serial_timeout */
int MODBUS_Serial_Init (void);
extern int Serial_timeout;
/*** EndHeader */

int Serial_timeout;

MODBUS_SLAVE_DEBUG
int MODBUS_Serial_Init ( void )
{
   serOpen ( MODBUS_BAUD );
   serdmaOn (DMA_CHANNEL_ANY,DMA_CHANNEL_ANY);
   serParity (PARAM_EPARITY);
   serRdFlush(); 							// clear the read FIFO

	return MB_SUCCESS;
} // MODBUS_Serial_Init

/* START FUNCTION DESCRIPTION ********************************************
MODBUS_Serial_Tx			<Modbus_Slave_LODM.LIB>

SYNTAX:			int MODBUS_Serial_Tx ( char *Packet, int ByteCount );

DESCRIPTION:	Transmit a Modbus packet to a "downstream" device.
					Calculate the CRC and append to the packet.

PARAMETER1:		address of packet - must have two byte pad at end for
					inclusion of CRC word

PARAMETER2:		number of bytes in the packet

RETURN VALUE:	MB_SUCCESS

END DESCRIPTION **********************************************************/

/*** BeginHeader MODBUS_Serial_Tx */
int MODBUS_Serial_Tx ( char *Packet, int ByteCount );
/*** EndHeader */

MODBUS_SLAVE_DEBUG
int MODBUS_Serial_Tx ( char *Packet, int ByteCount )
{	auto int CalcCRC,i;

// insert CRC
#ifndef USE_MODBUS_CRC
	CalcCRC = getcrc ( Packet, ByteCount, 0xFFFF );
	Packet[ByteCount] = CalcCRC;			// store low byte
	Packet[ByteCount+1] = CalcCRC>>8;	// store high byte
#else
	CalcCRC = MODBUS_CRC(Packet, ByteCount);
	Packet[ByteCount+1] = CalcCRC;		// store low byte
	Packet[ByteCount] = CalcCRC>>8;		// store high byte
#endif
	ByteCount += 2;							// adjust for CRC

#if MODBUS_DEBUG_PRINT & 8
	printf ( "Ser Tx:" );
	for ( i=0; i<ByteCount; i++ )
   {
   	printf ( " %02X", Packet[i] );
   }
	printf ( "\n\r" );
#endif

   BitWrPortI ( PEDR, &PEDRShadow, 1, 2);   // set DRV_EN

	i=serWrite ( Packet, ByteCount ); // send the data

   // wait until bytes have all been written
	#asm
   Wait_trans:
	ioi	ld		a, (serStatusReg)	  	; read the status register
	  		and	0x04				; test transmit bit
	  		jr		nz, Wait_trans	; jump if not done yet
	#endasm

   BitWrPortI  ( PEDR, &PEDRShadow, 0, 2);   // unset DRV_EN

	return MB_SUCCESS;						// show success
} // MODBUS_Serial_Tx

/* START FUNCTION DESCRIPTION *********************************************
MODBUS_Serial_Rx			<Modbus_Slave_LODM.LIB>

DESCRIPTION:	Receive the response from the Modbus Slave
					Uses the global variable Serial_timeout
					It is the responsibility of the caller to handle
					a timeout if required.

PARAMETER1:		address to put the data

RETURN VALUE:	0 = no message
					+n = number of bytes with valid CRC
               MB_CRC_ERROR = invalid CRC

END DESCRIPTION **********************************************************/
/*** BeginHeader MODBUS_Serial_Rx */
int MODBUS_Serial_Rx ( char * DataAddress );
/*** EndHeader */

MODBUS_SLAVE_DEBUG
int MODBUS_Serial_Rx ( char * DataAddress )
{	auto int RxCRC, CalcCRC;
	auto int ByteCount,i, reg, p1;


   // the RS232 driver library was modified to use TimerC so Serial_timeout
   // longer does anything
	ByteCount = serRead( DataAddress, 200 );

   if ( ByteCount )
   {
	#if MODBUS_DEBUG_PRINT & 8
		printf ( "\n\rSer Rx:" );
		for ( i=0; i<ByteCount; i++ ) printf ( " %02X", DataAddress[i] );
	#endif
   	ByteCount -= 2;						// adjust for CRC

	#ifndef USE_MODBUS_CRC
		CalcCRC = getcrc ( DataAddress, ByteCount, 0xFFFF );
		RxCRC = DataAddress[ByteCount] & 0x00FF; // LSByte
   	i = DataAddress[ByteCount+1]<<8;	// MSByte
	#else
		CalcCRC = MODBUS_CRC(DataAddress, ByteCount);
		RxCRC = DataAddress[ByteCount+1] & 0x00FF; // LSByte
   	i = DataAddress[ByteCount]<<8;	// MSByte
	#endif
   	RxCRC = RxCRC | ( i & 0xFF00 );	// merge bytes

	#if MODBUS_DEBUG_PRINT & 4
   	reg = DataAddress[2];
      reg = (reg<<8) + (int)DataAddress[3];
		p1 = DataAddress[4];
      p1 = (p1<<8) + (int)DataAddress[5];
		printf ( "\n\rSer Rx: Addr=%02d Function=%02X Reg=%05d P1=0x%04X  Calc CRC=%04X\n\r",
      	DataAddress[0], DataAddress[1], reg, p1, CalcCRC );
	#endif

	   if ( CalcCRC != RxCRC )
		{
			ByteCount = MB_CRC_ERROR;
		}
   }
   return ByteCount;
} // MODBUS_Serial_Rx


