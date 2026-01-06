/* START LIBRARY DESCRIPTION *********************************************
LODM_lib.LIB

DESCRIPTION:
   This library provides an API for the BL4S1xx single board computers
   family which use the Rabbit I/O chip (RIO).  One Rabbit I/O chip
   is used to drive the SBC's digital I/O structure. This adds significant
   run time configurability to these single board computers.

   This library offers functions to simplify the use of the on-board ADC,
   RS232 port, and Rabbitnet port.  There are functions for the setup
   and use of the digital I/O.

END DESCRIPTION **********************************************************/

/*** Beginheader */
unsigned char atten_set, atten_state, atten_mode, atten_index, MB_address, atten_table[50];
char opt_rx1_mssg[BUFFER_LEN];
char opt_rx2_mssg[BUFFER_LEN];
char atten_offset,atten_tmp;   //GN: declared  atten_tmp and atten_offset as signed byte

/*** EndHeader */

/////////////////////////////
// Digital Input Functions //
/////////////////////////////

/*** BeginHeader digIn */
int digIn(int channel);
/*** EndHeader */

/* START FUNCTION DESCRIPTION ********************************************
digIn                                                       <LODM_lib.LIB>

SYNTAX:        int digIn(int channel)

DESCRIPTION:   Reads the state of any digital input channel.
               This function is non-reentrant.

PARAMETER1:    Input channel to be read, 0 - 11.

               0  - 11 = channels DIN0 - DIN11.

RETURN VALUE:  The logic state of the specified channel.
               0 = Logic low.
               1 = Logic High.
               -EINVAL: channel value is out of range
               -EPERM:  channel functionality does not permit this operation

SEE ALSO:      brdInit, setDigIn, digInBank

END DESCRIPTION **********************************************************/

int digIn(int channel)
{
   auto int rc;

   // read digital input channels
   switch ( channel ) {
   	// LO2_LOCK monitor input
      case  0  :
         rc = BitRdPortI ( PCDR, 3);

         break;
      // DC_OK monitor input
//      case  1  :
//         rc = BitRdPortI ( PCDR, 1);

//       break;
   }

   return rc;
}


/*** BeginHeader digInBank */
int digInBank(int bank);
/*** EndHeader */

/* START FUNCTION DESCRIPTION ********************************************
digInBank                                                   <LODM_lib.LIB>

SYNTAX:        int digInBank(int bank)

DESCRIPTION:   Reads the state of 12 digital input channels DIN0-DIN11 in two
               banks of 8.
               This function is non-reentrant.

Parameter1:    Bank of inputs to read.

RETURN VALUE:  Data read from digital input bank.

               -EINVAL: invalid parameter value

SEE ALSO:      brdInit, digIn, setDigIn

END DESCRIPTION **********************************************************/

int digInBank(int bank)
{
   int data, raw_AD_counts;

   switch (bank)
   {
   case 0:	// V POL Detector Output Monitor
 		data =  anaInDriver( 0x80 | 8);  // read LN0
   	break;

   case 1: // H POL Detector Output Monitor
 		data =  anaInDriver( 0x80 | 9);  // read LN1
   	break;

   case 2: // report back attenuator values
 		data =  anaInDriver( 0x80 | 9);  // read LN1
   	break;


   case -1:
      data = (int)Firmware_Rev;
   	break;

   default:
      data = 0;
   	break;
   }
   return data;
}

//////////////////////////////
// Digital Output Functions //
//////////////////////////////

/*** BeginHeader digOut */
int digOut(int channel, int state);
/*** EndHeader */

/* START FUNCTION DESCRIPTION ********************************************
digOut                                                      <LODM_lib.LIB>

SYNTAX:        int digOut(int channel, int state)

DESCRIPTION:   Sets the state of a configurable Digital Out channel
               DOut0–DOut7 to a logic 0 or a logic 1. This function will
               only allow control of channels that are configured to be a
               general digital output by the setDigOut function.
               This function is non-reentrant.

PARAMETER1:    Digital output channel 0 - 7.

PARAMETER2:    Set output to one of the following states:
               0 = Connects the load to GND.
               1 = Puts the output in a high-impedance state.

RETURN VALUE:  0 on success.
               -EINVAL: invalid parameter value
               -EPERM:  pin function not set to digital output

SEE ALSO:      brdInit, setDigOut, digOutBank

END DESCRIPTION **********************************************************/

int digOut(int channel, int state)
{
	int n,i;
	extern char opt_rx1_mssg[BUFFER_LEN], opt_rx2_mssg[BUFFER_LEN];
	static const char cmd[] = "READ\n";

   i=0;
   switch (channel)
   {
   case 0:	// read status from OPT_RX1 on serial port D
		memset(opt_rx1_mssg,'\0',BUFFER_LEN); // clear memory buffer
   	serDputs(cmd);
   	serDrdFlush();
		while ((n = serDread(opt_rx1_mssg, BUFFER_LEN, 1000)) == 0 && i <600){
      	i++;
      }
      if (i == 600){ //cheesy time out for when serial device isn't connected
			strcpy(opt_rx1_mssg,"opt_rx1 read status failed");
      }

   	break;

   case 1:	// read status from OPT_RX2 on serial port C
		memset(opt_rx2_mssg,'\0',BUFFER_LEN); // clear memory buffer
   	serCputs(cmd);
   	serCrdFlush();
		while ((n = serCread(opt_rx2_mssg, BUFFER_LEN, 1000)) == 0 && i <600){
      	i++;
      }
      if (i == 600){ //cheesy time out for when serial device isn't connected
			strcpy(opt_rx2_mssg,"opt_rx2 read status failed");
      }
   	break;
   }

   return 0;
}

/*** BeginHeader digOutBank */
int digOutBank(int bank, int data);
/*** EndHeader */

/* START FUNCTION DESCRIPTION ********************************************
digOutBank                                                  <LODM_lib.LIB>

SYNTAX:        int digOutBank(int bank, int data)

DESCRIPTION:   Sets the state (logic 0 or logic 1) of a bank of 8
               digital output pins to the states contained in 'data'.
               This function only updates the channels that are
               configured to be digital outputs by the setDigOut
               function. Channels configured to other functions will
               not be affected.
               This function is non-reentrant.

PARAMETER1:    Bank of digital output channels being set.  Only bank 0 is
               supported.

PARAMETER2:    Data value to be written to the specified digital output
               bank, the data format and bitwise value is as follows:

               Data    Bank
               Bits     0
               --------------
         (LSB) D0  ->  DOut0
               D1  ->  DOut1
               D2  ->  DOut2
               D3  ->  DOut3
               D4  ->  DOut4
               D5  ->  DOut5
               D6  ->  DOut6
         (MSB) D7  ->  DOut7

               Bitwise value:
               --------------
               0 = Connects the load to GND.
               1 = Puts the output in a high-impedance state.

RETURN VALUE:  0 on success
               -EINVAL if parameter invalid or board not initialized

SEE ALSO:      brdInit, digOut, setDigOut

END DESCRIPTION **********************************************************/

int digOutBank(int bank, int data)
{
extern unsigned char atten_set, atten_mode,atten_index;
extern char atten_offset;
//GN declared atten_offset as an external signed byte
//GN: Why atten_table is not declared here as an external global variable?

   // A union split up int to char's to see if data is valid
	union {
	   unsigned char chr_val[2];
	   unsigned int int_val;
	} conv_int;

   conv_int.int_val = data;
   if ( bank == 0 ){
   	// save data to global variable.
		atten_set = ((conv_int.chr_val[1] << 4) |  conv_int.chr_val[0]);
   } else if ( bank == 1 ){  // set attenuator mode
   	if ( data == 0 ){
      	atten_mode = 0;   // normal mode
      } else {
      	atten_mode = 1;   // table mode
      }
   } else if ( bank == 2 ){  // set attenuator table index
      //GN This original statement assigns the lower byte of data to atten_index
    	//atten_index  = data;
      //GN This added statement attempts to assign the upper byte of data to the atten_offset variable
      // and the lower byte to atten_index
      atten_index = conv_int.chr_val[0];
      atten_offset = (char)conv_int.chr_val[1];
   } else if ( bank < 53 ){
   	// save data to global variable.
		atten_table[bank - 3] = ((conv_int.chr_val[1] << 4) |  conv_int.chr_val[0]);
   }
   return 0;
}

/*** BeginHeader rd_holding_reg */
int rd_holding_reg(int address) ;
/*** EndHeader */

/* START FUNCTION DESCRIPTION ********************************************
rd_holding_reg                                                      <LODM_lib.LIB>

SYNTAX:        int rd_ser_buf(int address,)

DESCRIPTION:   reads two bytes from the optical reciever status buffer.

PARAMETER1:    address, 0-65 is the buffer for OPT_RX1 and 66-132 is the buffer
					for OPT_RX2

RETURN VALUE:  0 on success.
               -EINVAL: invalid parameter value
               -EPERM:  pin function not set to digital output

END DESCRIPTION **********************************************************/

int rd_holding_reg(int address)
{
	char byte1, byte2;
	int data;
	extern char opt_rx1_mssg[BUFFER_LEN], opt_rx2_mssg[BUFFER_LEN];

   // A union split up int to char's to see if data is valid
	union {
	   unsigned char chr_val[2];
	   unsigned int int_val;
	} conv_int;

   // read buffer for OPT_RX1
   if (address <= (BUFFER_LEN / 2)-1){
	   byte1 = (address * 2);
   	byte2 = byte1++;
      conv_int.chr_val[0] = opt_rx1_mssg[byte1];
      conv_int.chr_val[1] = opt_rx1_mssg[byte2];

      data = conv_int.int_val;

   // read buffer for OPT_RX2
   } else if (address <= BUFFER_LEN - 1){
	   byte1 = (address * 2) - BUFFER_LEN;
   	byte2 = byte1++;
      conv_int.chr_val[0] = opt_rx2_mssg[byte1];
      conv_int.chr_val[1] = opt_rx2_mssg[byte2];

      data = conv_int.int_val;
   } else if (address == BUFFER_LEN){
		byte1 = atten_state;
      conv_int.chr_val[0] = ( byte1 & 0x0f );
      conv_int.chr_val[1] = ( byte1 & 0xf0 ) >> 4;
      data = conv_int.int_val;
   } else if (address == BUFFER_LEN + 1){
      //data =atten_index;    //GN Commented this original statement
      // GN This statement attempts to return atten_offset in the upper byte and atten_index in the lower byte
      // GN LabVIEW Modbus sends data as unsigned words,
      //so try to return monitor data in the same form but format is as int, since data is declared as innt
      conv_int.chr_val[0] = (unsigned char) atten_index;
      conv_int.chr_val[1] = (unsigned char) atten_offset;
      data = conv_int.int_val;
   } else if (address == BUFFER_LEN + 2){
		data = atten_mode;
   } else if (address <= BUFFER_LEN + 52){
		byte1 = atten_table[address - BUFFER_LEN - 3];
      conv_int.chr_val[0] = ( byte1 & 0x0f );
      conv_int.chr_val[1] = ( byte1 & 0xf0 ) >> 4;
      data = conv_int.int_val;
   } else {
   	data = 0;
   }
   return data;
}


/*** BeginHeader event_trigger */
void	event_trigger(void);
/*** EndHeader */

/* START FUNCTION DESCRIPTION ********************************************
event_trigger                                                  <LODM_lib.LIB>

SYNTAX:        void broadcast_Exec(void)

DESCRIPTION:   This command executes a response to a MODBUS broadcast message

RETURN VALUE:  void

END DESCRIPTION **********************************************************/

void event_trigger(void)
{
     // A union split up int to char's to see if data is valid
   int byte1, byte2, offset ;
	union {
	   char chr_val[2];
	   int int_val;
	} conv_int;

   if (atten_mode == 1){ // attenuator table operation
      offset= (int)(atten_offset)-15;
      byte1 = (int)(atten_table[atten_index] & 0x0f)+offset;
      byte2 = (int)((atten_table[atten_index] & 0xf0) >> 4)+offset;

      if (byte1 <0 )  {
          byte1 = 0;
      }
      if (byte1 >15 )  {
         byte1 = 15;
      }

      if (byte2 <0 )  {
          byte2 = 0;
      }
      if (byte2 >15 )  {
         byte2 = 15;
      }

      atten_state=(((unsigned char)byte2)<<4)|(unsigned char)byte1;
      //atten_state= atten_table[atten_index];
      //GN: commented increment of atten_index since it is broadcasted every 20ms from the control program
  	  // atten_index++;
     // if (atten_index > 49){
	  //    atten_index = 0;
     // }
   } else { // normal attenuator operation
	   atten_state = atten_set;
	}
	WrPortI(PADR,&PADRShadow,atten_state);  // writes out attenuator values
}
//////////////////////////
// Board Initialization //
//////////////////////////

/*** BeginHeader brdInit */
void brdInit(void);
/*** EndHeader */

/* START FUNCTION DESCRIPTION ********************************************
brdInit                 <LODM_lib.LIB>

SYNTAX:        void brdInit (void);

DESCRIPTION:   This function initializes the RCM4000 used on the LODM

PARAMETER:     None

RETURN VALUE:  None

END DESCRIPTION **********************************************************/
__nodebug
void brdInit(void)
{

// Define Analog Input configuration settings and channels
#define ADC_SCLKBAUD 115200ul

long tdivisor;
int	i;
unsigned char Port_B;
extern unsigned char MB_address;
extern char opt_rx1_mssg[BUFFER_LEN], opt_rx2_mssg[BUFFER_LEN];

	//disables the watchdog timer
   #asm
  			   ld a,0x51
		ioi 	ld (WDTTR),a
			 	ld a,0x54
		ioi 	ld (WDTTR),a
	#endasm

   //initialize optical receiver status buffer
	memset(opt_rx1_mssg,'\0',BUFFER_LEN);
	memset(opt_rx2_mssg,'\0',BUFFER_LEN);
	strcpy(opt_rx1_mssg,"opt_rx1 status buffer is empty");
	strcpy(opt_rx2_mssg,"opt_rx2 status buffer is empty");

   /////////////////////////////////////////////////////////////////////////
   // Configure Port A
   // Set Port A to all outputs
   /////////////////////////////////////////////////////////////////////////

  	WrPortI(SPCR,&SPCRShadow,0x84);				 // Port A set to outputs

   /////////////////////////////////////////////////////////////////////////
   // Configure Port B
   // Set Port B bits 1,2,3,4,5 to all inputs
   /////////////////////////////////////////////////////////////////////////
   BitWrPortI ( PBDDR, &PBDDRShadow, 0, 1);   //  Set PB1 to input
   BitWrPortI ( PBDDR, &PBDDRShadow, 0, 2);   //  Set PB2 to input
   BitWrPortI ( PBDDR, &PBDDRShadow, 0, 3);   //  Set PB3 to input
   BitWrPortI ( PBDDR, &PBDDRShadow, 0, 4);   //  Set PB4 to input
   BitWrPortI ( PBDDR, &PBDDRShadow, 0, 5);   //  Set PB5 to input

   // read parallel portB to get address bits
   Port_B = RdPortI(PBDR);

   // Read Port B and interpret into Modbus address, PB5=AD0, PB1=AD1, PB2=AD2, PB3=AD3, PB4=AD4
   MB_address = ((Port_B & 0x1E) | ((Port_B & 0x20)>>5));

   /////////////////////////////////////////////////////////////////////////
   // Configure Port E
   // PE2   Control output, RS-485 TX: 0 = disable, 1 = enable
   /////////////////////////////////////////////////////////////////////////

   BitWrPortI ( PEDDR, &PEDDRShadow, 1, 2);   //  Set PE2 to output
   BitWrPortI ( PEDR, &PEDRShadow, 0, 2);     //  Initialize PE2 to 0

// PE0 for debug
//   BitWrPortI ( PEDDR, &PEDDRShadow, 1, 0);   //  Set PE0 to output
//   BitWrPortI ( PEDR, &PEDRShadow, 0, 0);     //  Initialize PE0 to 0

	// Setup Serial Ports to talk to Optical recievers---------------------------
	   serCopen(9600);
   	serCdmaOn (DMA_CHANNEL_ANY,DMA_CHANNEL_ANY);
	   serDopen(9600);
   	serDdmaOn (DMA_CHANNEL_ANY,DMA_CHANNEL_ANY);

	//  ADC setup Start ------------------------------------------------------------
		// Setup PB0 (SCLK)
		BitWrPortI(PBDDR, &PBDDRShadow, 1, 0);

      // Setup PC4/PC5 as TXB/RXB
		BitWrPortI(PCDDR, &PCDDRShadow, 1, 4);
      BitWrPortI(PCFR,  &PCFRShadow,  1, 4);
 		BitWrPortI(PCDDR, &PCDDRShadow, 0, 5);
      BitWrPortI(PCFR,  &PCFRShadow,  0, 5);

      // Use the new 4000 serial timers
      tdivisor = (long)(2.0 * freq_divider * 19200.0/(float)ADC_SCLKBAUD + 0.5) - 1L;
      WrPortI (SBDLR, NULL, (char) tdivisor);
      WrPortI (SBDHR, NULL, (char) (tdivisor >> 8) | 0x80);

		// use internal clock for serial B
		WrPortI (SBCR, &SBCRShadow, 0x0C );
		// Set clock polarity - Rising edge
      WrPortI (SBER, &SBERShadow, 0x00);

		// set mode once
		if (__adcinitflag==FALSE) {
			__ad_readbackmode = 0x00;
		}

		__adcinitflag=TRUE;

      // Set up serial control register ADMODE1
      _ads7870driver(0x18, 0x81);

		// internal osc, cclk=2.5Mhz, ref powered, buf powered, vref= 2.048
		_ads7870driver(0x07, 0x3e);

      // internal osc, cclk=2.5Mhz, ref powered, buf powered, vref= 2.5
//		_ads7870driver(0x07, 0x3c);

// 		_msDelay(1000);						//allow 1 sec once only to charge up cap

      // Clear Rcv Status
   	for(i=0; i<4; i++)
   	{
   		#asm
  			ioi ld	a, (SBDR)			;Do dummy read to clear rcv status
    		#endasm
   	}
      // Do a one time dummy read of A/D to clear the pipeline
		anaInDriver(0x80);

//  ADC setup end --------------------------------------------------------------

		// initialize attenuators and attenuator variables
		atten_set = 0xff;    // set initial attenuation values to 15dB
      atten_state = 0xff;  //	set initial current attenuator state to 15dB
      atten_mode = 0;		// set initial attenuation mode, 0= normal, 1= table
  		atten_index = 0;		// set initial table index for table attenuator set
   	for(i=0; i<49; i++){ // initialize attenuator table all to 15dB
      	atten_table[i] = 0xff;
   	}
      WrPortI(PADR,&PADRShadow,atten_state);  // writes out attenuator values
}


