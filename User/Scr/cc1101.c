#include <stdio.h>
#include "DrvSYS.h"
#include "DrvSPI.h"
#include "DrvGPIO.h"
#include "DrvADC.h"
#include "DrvTimer.h"
#include "DrvUART.h"
#include "NUC1xx.h"
#include "CC1101.h"

SPI_T * SPI[4]={SPI0, SPI1, SPI2, SPI3};

//#ifdef RF_0db
// PATABLE (0 dBm output power)
char paTable[] ={0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0,0xC0};
//#endif
/**********************************************************************************************************************/
void SPI_Init()
{
	DrvSPI_Open(eDRVSPI_PORT0,eDRVSPI_MASTER,eDRVSPI_TYPE1,8);
	DrvSPI_SetClockFreq(eDRVSPI_PORT0,10000, 0);
	DrvSPI_DisableAutoSS(eDRVSPI_PORT0);
	//DrvSPI_SetSlaveSelectActiveLevel(eDRVSPI_PORT0, eDRVSPI_ACTIVE_LOW_FALLING);
	DrvSPI_SetSlaveSelectActiveLevel(eDRVSPI_PORT0, eDRVSPI_ACTIVE_HIGH_RISING);
	//DrvSPI_SetEndian (eDRVSPI_PORT0,eDRVSPI_LSB_FIRST);
	DrvGPIO_InitFunction (E_FUNC_SPI0);

	//Strobe(CCxxx0_SRX);
}
/**********************************************************************************************************************/
RF_SETTINGS rfSettings =        // 433Mhz, 9600Bauds(鮑率)
{
		0x00,
		    0x08,   // FSCTRL1   Frequency synthesizer control.
		    0x00,   // FSCTRL0   Frequency synthesizer control.
		    0x10,   // FREQ2     Frequency control word, high byte.
		    0xA7,   // FREQ1     Frequency control word, middle byte.
		    0x62,   // FREQ0     Frequency control word, low byte.
		    0x5B,   // MDMCFG4   Modem configuration.
		    0xF8,   // MDMCFG3   Modem configuration.
		    0x03,   // MDMCFG2   Modem configuration.
		    0x22,   // MDMCFG1   Modem configuration.
		    0xF8,   // MDMCFG0   Modem configuration.

		    0x00,   // CHANNR    Channel number.
		    0x47,   // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
		    0xB6,   // FREND1    Front end RX configuration.
		    0x10,   // FREND0    Front end RX configuration.
		    0x18,   // MCSM0     Main Radio Control State Machine configuration.
		    0x1D,   // FOCCFG    Frequency Offset Compensation Configuration.
		    0x1C,   // BSCFG     Bit synchronization Configuration.
		    0xC7,   // AGCCTRL2  AGC control.
		    0x00,   // AGCCTRL1  AGC control.
		    0xB2,   // AGCCTRL0  AGC control.

		    0xEA,   // FSCAL3    Frequency synthesizer calibration.
		    0x2A,   // FSCAL2    Frequency synthesizer calibration.
		    0x00,   // FSCAL1    Frequency synthesizer calibration.
		    0x11,   // FSCAL0    Frequency synthesizer calibration.
		    0x59,   // FSTEST    Frequency synthesizer calibration.
		    0x81,   // TEST2     Various test settings.
		    0x35,   // TEST1     Various test settings.
		    0x09,   // TEST0     Various test settings.
		    0x07,   // IOCFG2 0b   GDO2 output pin configuration.
		    0x06,   // IOCFG0D   GDO0 output pin configuration. Refer to SmartRF?Studio User Manual for detailed pseudo register explanation.

		    0x04,   // PKTCTRL1  Packet automation control.
		    0x05,   // PKTCTRL0  Packet automation control.
		    0x00,   // ADDR      Device address.
		    0x0c    // PKTLEN    Packet length.
};
/*******************************************************************************************/
void Init(void)
{
    WriteRfSettings(&rfSettings);				//配置CC1101的寄存器
    WriteBurstReg(CCxxx0_PATABLE,paTable,8);	//配置天線增益
}
/******************************************************************************************/
// Macro to reset the CCxxx0 and DrvSYS_Delay for it to be ready
void RESET_CCxxx0(void)
{
	/*while (MISO);
	CS = 0;
    DrvSYS_Delay(20);
	//while (MISO);
	DrvSPI_SingleWrite(eDRVSPI_PORT0,CCxxx0_SRES);
    DrvSYS_Delay(20);
	CS = 1;*/

	DrvSPI_ClrSS(eDRVSPI_PORT0,eDRVSPI_SS0);
	//CS=0;
	SPI[eDRVSPI_PORT0]->TX[0] = CCxxx0_SRES;	//寫入復位命令
	SPI[eDRVSPI_PORT0]->CNTRL.GO_BUSY = 1;
	while(DrvSPI_IsBusy(eDRVSPI_PORT0)==TRUE);
	DrvSPI_SetSS (eDRVSPI_PORT0,eDRVSPI_SS0);
	//CS=1;
}
/***************************************************************************************/
// Macro to reset the CCxxx0 after power_on and DrvSYS_Delay for it to be ready
// IMPORTANT NOTICE:
// The file DrvSYS_Delay.c must be included if this macro shall be used
// The file is located under: ..\Lib\Chipcon\Hal\CCxx00
//
//                 min 40 us
//             <----------------------->
// CSn      |--|  |--------------------|          |-----------
//          |  |  |                    |          |
//              --                      ----------
//
// MISO                                       |---------------
//          - - - - - - - - - - - - - - - -|  |
//                                          --
//               Unknown / don't care
//
// MOSI     - - - - - - - - - - - - - - - ---------- - - - - -
//                                         | SRES |
//          - - - - - - - - - - - - - - - ---------- - - - - -
//
/***************************************************************************************/
void POWER_UP_RESET_CCxxx0(void)
{
	/*CS = 1;
	DrvSYS_Delay(100);
	CS = 0;
	DrvSYS_Delay(100);
	CS = 1;
	DrvSYS_Delay(400);*/

	DrvSPI_SetSS(eDRVSPI_PORT0,eDRVSPI_SS0);
	DrvSYS_Delay(100);
	DrvSPI_ClrSS (eDRVSPI_PORT0,eDRVSPI_SS0);
	DrvSYS_Delay(100);
	DrvSPI_SetSS(eDRVSPI_PORT0,eDRVSPI_SS0);
	DrvSYS_Delay(400);
	RESET_CCxxx0();				//復位CC1101
}
/***************************************************************************************/
//  void WriteReg(unsigned char addr, unsigned char value)
//
//  DESCRIPTION:
//      Function for writing to a single CCxxx0 register
//
//  ARGUMENTS:
//      unsigned char addr
//          Address of a specific CCxxx0 register to accessed.
//      unsigned char value
//          Value to be written to the specified CCxxx0 register.
/***************************************************************************************/
void WriteReg(unsigned char addr, unsigned char value)
{
	/*DrvSYS_Delay(5);
    CS = 0;
    DrvSYS_Delay(2);
    //while (MISO);
    DrvSPI_SetTxRegister(eDRVSPI_PORT0,addr,8);
    DrvSPI_SingleWrite(eDRVSPI_PORT0,value);
    DrvSYS_Delay(2);
    CS = 1;*/

	//CS = 0;
	DrvSPI_ClrSS(eDRVSPI_PORT0,eDRVSPI_SS0);
    SPI[eDRVSPI_PORT0]->TX[0] = addr;			//寫地址
    SPI[eDRVSPI_PORT0]->CNTRL.GO_BUSY = 1;
    while(DrvSPI_IsBusy(eDRVSPI_PORT0)==TRUE);
    SPI[eDRVSPI_PORT0]->TX[0] = value;			//寫入配置
    SPI[eDRVSPI_PORT0]->CNTRL.GO_BUSY = 1;
    while(DrvSPI_IsBusy(eDRVSPI_PORT0)==TRUE);
    //CS = 1;
    DrvSPI_SetSS (eDRVSPI_PORT0,eDRVSPI_SS0);
}
/**************************************************************************************/
//  void WriteBurstReg(unsigned char addr, unsigned char *buffer, unsigned char count)
//
//  DESCRIPTION:
//      This function writes to multiple CCxxx0 register, using SPI burst access.
//
//  ARGUMENTS:
//      unsigned char addr
//          Address of the first CCxxx0 register to be accessed.
//      unsigned char *buffer
//          Array of bytes to be written into a corresponding range of
//          CCxx00 registers, starting by the address specified in _addr_.
//      unsigned char count
//          Number of bytes to be written to the subsequent CCxxx0 registers.
/*************************************************************************************/
void WriteBurstReg(unsigned char addr, unsigned char *buffer, unsigned char count)
{
	/*unsigned char i;
    DrvSYS_Delay(5);
    CS = 0;
    DrvSYS_Delay(2);
    //while (MISO);
    DrvSPI_SingleWrite(eDRVSPI_PORT0,addr | WRITE_BURST);
    for (i = 0; i < count; i++)
    {
		DrvSPI_SingleWrite(eDRVSPI_PORT0,buffer[i]);
    }
    DrvSYS_Delay(2);
    CS = 1;*/

    unsigned char i;
    //CS = 0;
    DrvSPI_ClrSS(eDRVSPI_PORT0,eDRVSPI_SS0);
	SPI[eDRVSPI_PORT0]->TX[0] = addr|WRITE_BURST;//
	SPI[eDRVSPI_PORT0]->CNTRL.GO_BUSY = 1;
	while(DrvSPI_IsBusy(eDRVSPI_PORT0)==TRUE);
	/*SPI[eDRVSPI_PORT0]->TX[0] = 0xff;
	SPI[eDRVSPI_PORT0]->CNTRL.GO_BUSY = 1;
	while(DrvSPI_IsBusy(eDRVSPI_PORT0)==TRUE);*/
	for (i = 0; i < count; i++)
	{
		SPI[eDRVSPI_PORT0]->TX[0]=buffer[i];////
		SPI[eDRVSPI_PORT0]->CNTRL.GO_BUSY = 1;
		while(DrvSPI_IsBusy(eDRVSPI_PORT0)==TRUE);
	}
	//CS = 1;
	DrvSPI_SetSS (eDRVSPI_PORT0,eDRVSPI_SS0);
}
/************************************************************************************/
//  void Strobe(unsigned char strobe)
//
//  DESCRIPTION:
//      Function for writing a strobe command to the CCxxx0
//
//  ARGUMENTS:
//      unsigned char strobe
//          Strobe command
/***********************************************************************************/
void Strobe(unsigned char strobe)
{
	/*unsigned char x;
    DrvSYS_Delay(5);
    CS = 0;
    DrvSYS_Delay(2);
    //while (MISO);
    x = DrvSPI_SingleWrite(eDRVSPI_PORT0,strobe);
    DrvSYS_Delay(2);
    CS = 1;
    return x;*/

	unsigned char x;
	//CS = 0;
	DrvSPI_ClrSS(eDRVSPI_PORT0,eDRVSPI_SS0);
	SPI[eDRVSPI_PORT0]->TX[0] = strobe;			//寫入命令
	SPI[eDRVSPI_PORT0]->CNTRL.GO_BUSY = 1;
	while(DrvSPI_IsBusy(eDRVSPI_PORT0)==TRUE);
	/*SPI[eDRVSPI_PORT0]->TX[0] = 0xff;
	SPI[eDRVSPI_PORT0]->CNTRL.GO_BUSY = 1;
	while(DrvSPI_IsBusy(eDRVSPI_PORT0)==TRUE);*/
	x=SPI[eDRVSPI_PORT0]->TX[0];
	SPI[eDRVSPI_PORT0]->CNTRL.GO_BUSY = 1;
	while(DrvSPI_IsBusy(eDRVSPI_PORT0)==TRUE);
	//CS = 1;
	DrvSPI_SetSS (eDRVSPI_PORT0,eDRVSPI_SS0);
	return x;
}
/**************************************************************************************/
//  unsigned char ReadReg(unsigned char addr)
//
//  DESCRIPTION:
//      This function gets the value of a single specified CCxxx0 register.
//
//  ARGUMENTS:
//      unsigned char addr
//          Address of the CCxxx0 register to be accessed.
//
//  RETURN VALUE:
//      unsigned char
//          Value of the accessed CCxxx0 register.
/*************************************************************************************/
unsigned char ReadReg(unsigned char addr)
{
	/*unsigned char x;
    DrvSYS_Delay(5);
    CS = 0;
    DrvSYS_Delay(2);
    //while (MISO);
    DrvSPI_SingleWrite(eDRVSPI_PORT0,addr | READ_SINGLE);
    x = DrvSPI_SingleWrite(eDRVSPI_PORT0,0);
    DrvSYS_Delay(2);
    CS = 1;
    return x;*/

	unsigned char x;
	//CS = 0;
	DrvSPI_ClrSS(eDRVSPI_PORT0,eDRVSPI_SS0);
	SPI[eDRVSPI_PORT0]->TX[0] = addr| READ_SINGLE;	//讀寄存器命令
	SPI[eDRVSPI_PORT0]->CNTRL.GO_BUSY = 1;
	while(DrvSPI_IsBusy(eDRVSPI_PORT0)==TRUE);
	/*SPI[eDRVSPI_PORT0]->TX[0] = 0xff;
	SPI[eDRVSPI_PORT0]->CNTRL.GO_BUSY = 1;
	while(DrvSPI_IsBusy(eDRVSPI_PORT0)==TRUE);*/
	x=SPI[eDRVSPI_PORT0]->RX[0];
	SPI[eDRVSPI_PORT0]->CNTRL.GO_BUSY = 1;
	while(DrvSPI_IsBusy(eDRVSPI_PORT0)==TRUE);
	//CS = 1;
	DrvSPI_SetSS (eDRVSPI_PORT0,eDRVSPI_SS0);
	return x;
}
/************************************************************************************************/
//  void ReadBurstReg(unsigned char addr, unsigned char *buffer, unsigned char count)
//
//  DESCRIPTION:
//      This function reads multiple CCxxx0 register, using SPI burst access.
//
//  ARGUMENTS:
//      unsigned char addr
//          Address of the first CCxxx0 register to be accessed.
//      unsigned char *buffer
//          Pointer to a byte array which stores the values read from a
//          corresponding range of CCxxx0 registers.
//      unsigned char count
//          Number of bytes to be read from the subsequent CCxxx0 registers.
/**********************************************************************************************/
void ReadBurstReg(unsigned char addr, unsigned char *buffer, unsigned char count)
{
	/*unsigned char i;
    DrvSYS_Delay(5);
    CS = 0;
    DrvSYS_Delay(2);
    //while (MISO);
    DrvSPI_SingleWrite(eDRVSPI_PORT0,addr | READ_BURST);
    for (i = 0; i < count; i++)
    {
		buffer[i] = DrvSPI_SingleWrite(eDRVSPI_PORT0,0);
    }
    DrvSYS_Delay(2);
    CS = 1;*/

	unsigned char i;
	//CS = 0;
	DrvSPI_ClrSS(eDRVSPI_PORT0,eDRVSPI_SS0);
	SPI[eDRVSPI_PORT0]->TX[0] = addr| READ_BURST;	//寫入要讀的配置寄存器地址和讀命令
	SPI[eDRVSPI_PORT0]->CNTRL.GO_BUSY = 1;
	while(DrvSPI_IsBusy(eDRVSPI_PORT0)==TRUE);
	/*SPI[eDRVSPI_PORT0]->TX[0] = 0xff;
	SPI[eDRVSPI_PORT0]->CNTRL.GO_BUSY = 1;
	while(DrvSPI_IsBusy(eDRVSPI_PORT0)==TRUE);*/
	for (i = 0; i < count; i++)
	{
		buffer[i] =SPI[eDRVSPI_PORT0]->RX[0];
		SPI[eDRVSPI_PORT0]->CNTRL.GO_BUSY = 1;
		while(DrvSPI_IsBusy(eDRVSPI_PORT0)==TRUE);
	}
	//CS = 1;
	DrvSPI_SetSS(eDRVSPI_PORT0,eDRVSPI_SS0);
}
/**************************************************************************************/
//  unsigned char ReadStatus(unsigned char addr)
//
//  DESCRIPTION:
//      This function reads a CCxxx0 status register.
//
//  ARGUMENTS:
//      unsigned char addr
//          Address of the CCxxx0 status register to be accessed.
//
//  RETURN VALUE:
//      unsigned char
//          Value of the accessed CCxxx0 status register.
/*************************************************************************************/
unsigned char ReadStatus(unsigned char addr)
{
	/*unsigned char x;
	DrvSYS_Delay(5);
    CS = 0;
    DrvSYS_Delay(2);
    //while (MISO);
    DrvSPI_SingleWrite(eDRVSPI_PORT0,addr | READ_BURST);
    x = DrvSPI_SingleWrite(eDRVSPI_PORT0,0);
    DrvSYS_Delay(2);
    CS = 1;
    return x;*/

	unsigned char x;
	//CS = 0;
	DrvSPI_ClrSS(eDRVSPI_PORT0,eDRVSPI_SS0);
	SPI[eDRVSPI_PORT0]->TX[0] = addr| READ_BURST;	//寫入要讀的狀態寄存器地址同時寫入讀命令
	SPI[eDRVSPI_PORT0]->CNTRL.GO_BUSY = 1;
	while(DrvSPI_IsBusy(eDRVSPI_PORT0)==TRUE);
	/*SPI[eDRVSPI_PORT0]->TX[0] = 0xff;
	SPI[eDRVSPI_PORT0]->CNTRL.GO_BUSY = 1;
	while(DrvSPI_IsBusy(eDRVSPI_PORT0)==TRUE);*/
	x=SPI[eDRVSPI_PORT0]->RX[0];
	SPI[eDRVSPI_PORT0]->CNTRL.GO_BUSY = 1;
	while(DrvSPI_IsBusy(eDRVSPI_PORT0)==TRUE);
	//CS = 1;
	DrvSPI_SetSS (eDRVSPI_PORT0,eDRVSPI_SS0);
	return x;
}
/******************************************************************************************/
//  void WriteRfSettings(RF_SETTINGS *pRfSettings)
//
//  DESCRIPTION:
//      This function is used to configure the CCxxx0 based on a given rf setting
//
//  ARGUMENTS:
//      RF_SETTINGS *pRfSettings
//          Pointer to a struct containing rf register settings
/******************************************************************************************/
void WriteRfSettings(RF_SETTINGS *pRfSettings)
{
	// Write register settings
	WriteReg(CCxxx0_FSCTRL0,  rfSettings.FSCTRL2);//自已加的
	    // Write register settings
	    WriteReg(CCxxx0_FSCTRL1,  rfSettings.FSCTRL1);
	    WriteReg(CCxxx0_FSCTRL0,  rfSettings.FSCTRL0);
	    WriteReg(CCxxx0_FREQ2,    rfSettings.FREQ2);
	    WriteReg(CCxxx0_FREQ1,    rfSettings.FREQ1);
	    WriteReg(CCxxx0_FREQ0,    rfSettings.FREQ0);
	    WriteReg(CCxxx0_MDMCFG4,  rfSettings.MDMCFG4);
	    WriteReg(CCxxx0_MDMCFG3,  rfSettings.MDMCFG3);
	    WriteReg(CCxxx0_MDMCFG2,  rfSettings.MDMCFG2);
	    WriteReg(CCxxx0_MDMCFG1,  rfSettings.MDMCFG1);
	    WriteReg(CCxxx0_MDMCFG0,  rfSettings.MDMCFG0);
	    WriteReg(CCxxx0_CHANNR,   rfSettings.CHANNR);
	    WriteReg(CCxxx0_DEVIATN,  rfSettings.DEVIATN);
	    WriteReg(CCxxx0_FREND1,   rfSettings.FREND1);
	    WriteReg(CCxxx0_FREND0,   rfSettings.FREND0);
	    WriteReg(CCxxx0_MCSM0 ,   rfSettings.MCSM0 );
	    WriteReg(CCxxx0_FOCCFG,   rfSettings.FOCCFG);
	    WriteReg(CCxxx0_BSCFG,    rfSettings.BSCFG);
	    WriteReg(CCxxx0_AGCCTRL2, rfSettings.AGCCTRL2);
	    WriteReg(CCxxx0_AGCCTRL1, rfSettings.AGCCTRL1);
	    WriteReg(CCxxx0_AGCCTRL0, rfSettings.AGCCTRL0);
	    WriteReg(CCxxx0_FSCAL3,   rfSettings.FSCAL3);
	    WriteReg(CCxxx0_FSCAL2,   rfSettings.FSCAL2);
	    WriteReg(CCxxx0_FSCAL1,   rfSettings.FSCAL1);
	    WriteReg(CCxxx0_FSCAL0,   rfSettings.FSCAL0);
	    WriteReg(CCxxx0_FSTEST,   rfSettings.FSTEST);
	    WriteReg(CCxxx0_TEST2,    rfSettings.TEST2);
	    WriteReg(CCxxx0_TEST1,    rfSettings.TEST1);
	    WriteReg(CCxxx0_TEST0,    rfSettings.TEST0);
	    WriteReg(CCxxx0_IOCFG2,   rfSettings.IOCFG2);
	    WriteReg(CCxxx0_IOCFG0,   rfSettings.IOCFG0);
	    WriteReg(CCxxx0_PKTCTRL1, rfSettings.PKTCTRL1);
	    WriteReg(CCxxx0_PKTCTRL0, rfSettings.PKTCTRL0);
	    WriteReg(CCxxx0_ADDR,     rfSettings.ADDR);
	    WriteReg(CCxxx0_PKTLEN,   rfSettings.PKTLEN);
    //RXMode();
}
/************************************************************************************************/
//  void halRfSendPacket(unsigned char *txBuffer, unsigned char size)
//
//  DESCRIPTION:
//      This function can be used to transmit a packet with packet length up to 63 bytes.
//
//  ARGUMENTS:
//      unsigned char *txBuffer
//          Pointer to a buffer containing the data that are going to be transmitted
//
//      unsigned char size
//          The size of the txBuffer
/************************************************************************************************/
void SendPacket(unsigned char *txBuffer, unsigned char size)
{
	WriteReg(CCxxx0_TXFIFO,size);
	WriteBurstReg(CCxxx0_TXFIFO, txBuffer, size);	//寫入要發送的數據
	Strobe(CCxxx0_STX);	//進入發送模式發送數據
    //ReadStatus(CCxxx0_TXBYTES);
    while(GPE_5!=0);
    while(GPE_5=0);
    //ReadStatus(CCxxx0_TXBYTES);
    Strobe(CCxxx0_SFTX);
}
/*************************************************************************************************/
void RXMode(void)
{
	//Strobe(CCxxx0_SIDLE);
    Strobe(CCxxx0_SRX);		//進入接收狀態
}
/**********************************************************************************************************/
//  BOOL ReceivePacket(unsigned char *rxBuffer, unsigned char *length)
//
//  DESCRIPTION:
//      This function can be used to receive a packet of variable packet length (first byte in the packet
//      must be the length byte). The packet length should not exceed the RX FIFO size.
//
//  ARGUMENTS:
//      unsigned char *rxBuffer
//          Pointer to the buffer where the incoming data should be stored
//      unsigned char *length
//          Pointer to a variable containing the size of the buffer where the incoming data should be
//          stored. After this function returns, that variable holds the packet length.
//
//  RETURN VALUE:
//      BOOL
//          1:   CRC OK
//          0:  CRC NOT OK (or no packet was put in the RX FIFO due to filtering)
/********************************************************************************************************/
unsigned char ReceivePacket(unsigned char *rxBuffer, unsigned char *length)
{
	unsigned char status[2];
	unsigned char packetLength;

	//Strobe(CCxxx0_SIDLE);
	Strobe(CCxxx0_SRX);	//進入接收狀態
	//ReadBurstReg(CCxxx0_RXFIFO, rxBuffer, packetLength);
	//packetLength = ;
	if (ReadStatus(CCxxx0_RXBYTES) & BYTES_IN_RXFIFO)//如果街的字節數不為0
	{
		// Read length byte
        packetLength = ReadReg(CCxxx0_RXFIFO);//讀出第一個字節，此字節為該幀數據長度
        // Read data from RX FIFO and store in rxBuffer
        if (packetLength <= *length)	//如果所要的有效數據長度小於等於接收到的數據包的長度
        {
        	ReadBurstReg(CCxxx0_RXFIFO, rxBuffer, packetLength);//讀出所有接收到的數據
            *length = packetLength;			//把接收數據的長度修改為當前數據的長度
            // Read the 2 appended status bytes (status[0] = RSSI, status[1] = LQI)
            ReadBurstReg(CCxxx0_RXFIFO, status, 2);	//讀出CRC校驗位
            Strobe(CCxxx0_SFRX);	//清洗接收緩衝區
            rssi = status[RSSI];
            lqi = status[LQI];
            // MSB of LQI is the CRC_OK bit
            //return (status[LQI] & CRC_OK);
            return(status[LQI] & CRC_OK);
        }
        else
        {
        	*length = packetLength;
            // Make sure that the radio is in IDLE state before flushing the FIFO
            // (Unless RXOFF_MODE has been changed, the radio should be in IDLE state at this point)
            //Strobe(CCxxx0_SIDLE);
            // Flush RX FIFO
            Strobe(CCxxx0_SFRX);	//清洗接收緩衝區
            return 0;
        }
    } else
	return 0;
}
/*int ReceivePacket(unsigned char *rxBuffer, unsigned char *length)
{
	unsigned char packetLength;
	//packetLength = ReadReg(CCxxx0_RXFIFO);
	//Strobe(CCxxx0_SRX);
	ReadBurstReg(CCxxx0_RXFIFO,RXBUF, packetLength);
	//Strobe(CCxxx0_SIDLE);
	//Strobe(CCxxx0_SFRX);
	//length = packetLength;
}*/
void setSleepMod()
{
	DrvSPI_SetSS (eDRVSPI_PORT0,eDRVSPI_SS0);
	Strobe(CCxxx0_SPWD);
}


