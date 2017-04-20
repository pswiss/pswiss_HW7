#include<xc.h>           // processor SFR definitions
#include<sys/attribs.h>  // __ISR macro
# include<math.h>       // I like to do math
#include<stdio.h>
#include"ILI9163C.h"
#include"I2C2_Commands.h"

// DEVCFG0
#pragma config DEBUG = ON // no debugging
#pragma config JTAGEN = OFF // no jtag
#pragma config ICESEL = ICS_PGx1 // use PGED1 and PGEC1
#pragma config PWP = OFF // no write protect
#pragma config BWP = OFF // no boot write protect
#pragma config CP = OFF // no code protect

// DEVCFG1
#pragma config FNOSC = PRIPLL // use primary oscillator with pll
#pragma config FSOSCEN = OFF // turn off secondary oscillator
#pragma config IESO = OFF // no switching clocks
#pragma config POSCMOD = HS // high speed crystal mode
#pragma config OSCIOFNC = OFF // free up secondary osc pins
#pragma config FPBDIV = DIV_1 // divide CPU freq by 1 for peripheral bus clock
#pragma config FCKSM = CSDCMD // do not enable clock switch
#pragma config WDTPS = PS1048576 // slowest wdt
#pragma config WINDIS = OFF // no wdt window
#pragma config FWDTEN = OFF // wdt off by default
#pragma config FWDTWINSZ = WINSZ_25 // wdt window at 25%

// DEVCFG2 - get the CPU clock to 48MHz
#pragma config FPLLIDIV = DIV_2 // divide input clock to be in range 4-5MHz
#pragma config FPLLMUL = MUL_24 // multiply clock after FPLLIDIV
#pragma config FPLLODIV = DIV_2 // divide clock after FPLLMUL to get 48MHz
#pragma config UPLLIDIV = DIV_2 // divider for the 8MHz input clock, then multiply by 12 to get 48MHz for USB
#pragma config UPLLEN = ON // USB clock on

// DEVCFG3
#pragma config USERID = 0 // some 16bit userid, doesn't matter what
#pragma config PMDL1WAY = OFF // allow multiple reconfigurations
#pragma config IOL1WAY = OFF // allow multiple reconfigurations
#pragma config FUSBIDIO = ON // USB pins controlled by USB module
#pragma config FVBUSONIO = ON // USB BUSON controlled by USB module

#define CS LATBbits.LATB7       // chip select pin

// functions to operate the ILI9163C on the PIC32
// adapted from https://github.com/sumotoy/TFT_ILI9163C/blob/master/TFT_ILI9163C.cpp

// pin connections:
// VCC - 3.3V
// GND - GND
// CS - B7
// RESET - 3.3V
// A0 - B15
// SDA - A1
// SCK - B14
// LED - 3.3V

// B8 is turned into SDI1 but is not used or connected to anything

// Definitions to make life easier
#define CS LATBbits.LATB7       // chip select pin
#define IMU_Address 0b1101011 // Address for the IMU
#define reg_Accel 0x10
#define reg_Gyro 0x11
#define reg_Contr 0x12
#define reg_OUT_TEMP_L 0x20


// Color Definitions
// Color: RRRRR GGGGGG BBBBB
#define colorRED  0b1111100000000000
#define colorGREEN 0b0000001111100000
#define colorBLUE 0b0000000000011111
#define colorPURPLE 0b1011100000011111
#define colorCYAN 0b0000011100011100
#define colorYELLOW 0b1111111111100000
#define colorORANGE 0b1111101011100000
#define colorBLACK 0b0000000000000000
#define colorWHITE 0b1111111111111111

// Character Dimensions
#define charWidth 5
#define charHeight 8
#define screenWidth 130
#define screenHeight 130


void SPI1_init() {
	SDI1Rbits.SDI1R = 0b0100; // B8 is SDI1
    RPA1Rbits.RPA1R = 0b0011; // A1 is SDO1
    TRISBbits.TRISB7 = 0; // SS is B7
    LATBbits.LATB7 = 1; // SS starts high

    // A0 / DAT pin
    ANSELBbits.ANSB15 = 0;
    TRISBbits.TRISB15 = 0;
    LATBbits.LATB15 = 0;
	
	SPI1CON = 0; // turn off the spi module and reset it
    SPI1BUF; // clear the rx buffer by reading from it
    SPI1BRG = 1; // baud rate to 12 MHz [SPI1BRG = (48000000/(2*desired))-1]
    SPI1STATbits.SPIROV = 0; // clear the overflow bit
    SPI1CONbits.CKE = 1; // data changes when clock goes from hi to lo (since CKP is 0)
    SPI1CONbits.MSTEN = 1; // master operation
    SPI1CONbits.ON = 1; // turn on spi1
}

unsigned char spi_io(unsigned char o) {
  SPI1BUF = o;
  while(!SPI1STATbits.SPIRBF) { // wait to receive the byte
    ;
  }
  return SPI1BUF;
}

void LCD_command(unsigned char com) {
    LATBbits.LATB15 = 0; // DAT
    CS = 0; // CS
    spi_io(com);
    CS = 1; // CS
}

void LCD_data(unsigned char dat) {
    LATBbits.LATB15 = 1; // DAT
    CS = 0; // CS
    spi_io(dat);
    CS = 1; // CS
}

void LCD_data16(unsigned short dat) {
    LATBbits.LATB15 = 1; // DAT
    CS = 0; // CS
    spi_io(dat>>8);
    spi_io(dat);
    CS = 1; // CS
}

void LCD_init() {
    int time = 0;
    LCD_command(CMD_SWRESET);//software reset
    time = _CP0_GET_COUNT();
    while (_CP0_GET_COUNT() < time + 48000000/2/2) {} //delay(500);

	LCD_command(CMD_SLPOUT);//exit sleep
    time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/200) {} //delay(5);

	LCD_command(CMD_PIXFMT);//Set Color Format 16bit
	LCD_data(0x05);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/200) {} //delay(5);

	LCD_command(CMD_GAMMASET);//default gamma curve 3
	LCD_data(0x04);//0x04
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_GAMRSEL);//Enable Gamma adj
	LCD_data(0x01);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_NORML);

	LCD_command(CMD_DFUNCTR);
	LCD_data(0b11111111);
	LCD_data(0b00000110);

    int i = 0;
	LCD_command(CMD_PGAMMAC);//Positive Gamma Correction Setting
	for (i=0;i<15;i++){
		LCD_data(pGammaSet[i]);
	}

	LCD_command(CMD_NGAMMAC);//Negative Gamma Correction Setting
	for (i=0;i<15;i++){
		LCD_data(nGammaSet[i]);
	}

	LCD_command(CMD_FRMCTR1);//Frame Rate Control (In normal mode/Full colors)
	LCD_data(0x08);//0x0C//0x08
	LCD_data(0x02);//0x14//0x08
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_DINVCTR);//display inversion
	LCD_data(0x07);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_PWCTR1);//Set VRH1[4:0] & VC[2:0] for VCI1 & GVDD
	LCD_data(0x0A);//4.30 - 0x0A
	LCD_data(0x02);//0x05
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_PWCTR2);//Set BT[2:0] for AVDD & VCL & VGH & VGL
	LCD_data(0x02);
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_VCOMCTR1);//Set VMH[6:0] & VML[6:0] for VOMH & VCOML
	LCD_data(0x50);//0x50
	LCD_data(99);//0x5b
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_VCOMOFFS);
	LCD_data(0);//0x40
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_CLMADRS);//Set Column Address
	LCD_data16(0x00);
    LCD_data16(_GRAMWIDTH);

	LCD_command(CMD_PGEADRS);//Set Page Address
	LCD_data16(0x00);
    LCD_data16(_GRAMHEIGH);

	LCD_command(CMD_VSCLLDEF);
	LCD_data16(0); // __OFFSET
	LCD_data16(_GRAMHEIGH); // _GRAMHEIGH - __OFFSET
	LCD_data16(0);

	LCD_command(CMD_MADCTL); // rotation
    LCD_data(0b00001000); // bit 3 0 for RGB, 1 for GBR, rotation: 0b00001000, 0b01101000, 0b11001000, 0b10101000

	LCD_command(CMD_DISPON);//display ON
	time = _CP0_GET_COUNT();
	while (_CP0_GET_COUNT() < time + 48000000/2/1000) {} //delay(1);

	LCD_command(CMD_RAMWR);//Memory Write
}

void LCD_drawPixel(unsigned short x, unsigned short y, unsigned short color) {
    // check boundary
    LCD_setAddr(x,y,x+1,y+1);
    LCD_data16(color);
}

void LCD_setAddr(unsigned short x0, unsigned short y0, unsigned short x1, unsigned short y1) {
    LCD_command(CMD_CLMADRS); // Column
    LCD_data16(x0);
	LCD_data16(x1);

	LCD_command(CMD_PGEADRS); // Page
	LCD_data16(y0);
	LCD_data16(y1);

	LCD_command(CMD_RAMWR); //Into RAM
}

void LCD_clearScreen(unsigned short color) {
    int i;
    LCD_setAddr(0,0,_GRAMWIDTH,_GRAMHEIGH);
		for (i = 0;i < _GRAMSIZE; i++){
			LCD_data16(color);
		}
}

void draw_Character(unsigned short x, unsigned short y, unsigned short character, unsigned short frontColor, unsigned short backColor, float fontSize )
{
    // Don't draw off the screen
    if(((x+charWidth)<=screenWidth)&&((y+charHeight)<=screenHeight))
    {
        // Initialize counter Variables
        int i = 0;
        int j = 0;
        // Loop through columns
        for(i = 0; i<charWidth*fontSize;i++) 
        {
            // Loop through rows
            for(j = 0; j<charHeight*fontSize;j++)
            {
                // Check if bit at row,column of selected character is 1 or 0 then assign color
                if(((ASCII[character-0x20][(int)(i/fontSize)])&(0b00000001<<((int)(j/fontSize))))!=0) 
                {
                    LCD_drawPixel(x+i,y+j,frontColor);
                }
                else
                {
                    LCD_drawPixel(x+i,y+j,backColor);
                }
            }
        }
    }
}

void draw_Message(unsigned short x, unsigned short y, char* message, unsigned short frontColor, unsigned short backColor, float fontSize)
{
    // Loop through all characters in string
    int i = 0;
    while(message[i]!=0)
    {
        draw_Character(x+i*charWidth*fontSize, y, message[i], frontColor, backColor, fontSize);
        i++;
    }
}

void draw_Rectanle(unsigned short x1, unsigned short x2, unsigned short y1, unsigned short y2,  unsigned short frontColor)
{
    int i = 0;
    int j = 0;
    // Loop through rows
    for(i = 0; i<(x2-x1);i++) 
    {
        // Loop through columns
        for(j = 0; j<y2-y1;j++)
        {
            LCD_drawPixel(x1+i,y1+j,frontColor);
        }
    }
}

void draw_HLine(unsigned short x1, unsigned short y1, int length, int thickness,unsigned short frontColor)
{
    int i = 0;
    int j = 0;
    
    for(i = 0;i<length;i++)
    {
        for(j=-thickness/2;j<thickness/2;j++)
        {
            LCD_drawPixel(x1+i,y1+j,frontColor);
        }
    }
}

void IMU_init(void)
{
    // Initializes the IMU
    unsigned char sendbyte;
    
    i2c_master_start();
    sendbyte = (IMU_Address << 1)|(0b00000000);// Writing    
    i2c_master_send(sendbyte); // Send Device ID with Write Byte
    i2c_master_send(reg_Accel); // CTRL1_XL Register
    i2c_master_send(0b10000010); //1.66 kHz [1000], 2g [00], 100 Hz Filter [10]
    i2c_master_stop();
    
    i2c_master_start();
    sendbyte = (IMU_Address << 1)|(0b00000000);// Writing    
    i2c_master_send(sendbyte); // Send Device ID with Write Byte
    i2c_master_send(reg_Gyro); // CTRL2_G Register
    i2c_master_send(0b10001000); // 1.66kHz [1000], 1000 dps sense [10], [00]
    i2c_master_stop();
    
    i2c_master_start();
    sendbyte = (IMU_Address << 1)|(0b00000000);// Writing    
    i2c_master_send(sendbyte); // Send Device ID with Write Byte
    i2c_master_send(reg_Contr); // CTRL3_C Register
    i2c_master_send(0b00000100); // [00000100]
    i2c_master_stop();
}

void I2C_read_multiple(unsigned char address, unsigned char regRead, unsigned char * data, int length)
{
    // Read from registers
    i2c_master_start();
    unsigned char sendbyte = (IMU_Address << 1)|(0b00000001);// Reading    
    i2c_master_send(sendbyte); // Send Device ID with Read Byte
    i2c_master_send(regRead); // The first reading byte
    i2c_master_restart();
    
    int i = 0;
    // Read all data
    for(i = 0; i<length;i++)
    {
        *data[i] = i2c_master_recv(); // Receive current register
        if(i == length-1)
        {
            i2c_master_ack(1);
        }
        else
        {
            i2c_master_ack(0);
        }
    }
    
    i2c_master_stop(); // End Comms
    
}

int main() {

    __builtin_disable_interrupts();

    // set the CP0 CONFIG register to indicate that kseg0 is cacheable (0x3)
    __builtin_mtc0(_CP0_CONFIG, _CP0_CONFIG_SELECT, 0xa4210583);

    // 0 data RAM access wait states
    BMXCONbits.BMXWSDRM = 0x0;

    // enable multi vector interrupts
    INTCONbits.MVEC = 0x1;

    // disable JTAG to get pins back
    DDPCONbits.JTAGEN = 0;

    // do your TRIS and LAT commands here
    __builtin_enable_interrupts();
        
    // Initialize the Timer
    _CP0_SET_COUNT(0);
    
    int timetoWait = 48000000*0.1/2;      // 10hz
    int numberTimer = 0;
    
    // Configure Bits
    ANSELA = 0;
    ANSELB = 0;
    
    // Initialize communications
    SPI1_init();
    i2c_master_setup();
    // Initialize the LCD
    LCD_init();
    LCD_clearScreen(colorWHITE);

    // Draw initial message to screen
    char message[100];
    sprintf(message,"Hello, World ");
    draw_Message(10, 10, message, colorRED, colorWHITE,1 );
    
    // Initialize the IMU
    IMU_init();
    
         
    while(1) {
        
        // Wait for 0.1 s
        if(_CP0_GET_COUNT()>timetoWait)
        {          
            _CP0_SET_COUNT(0);
            unsigned char data[];
            I2C_read_multiple(IMU_Address, reg_OUT_TEMP_L, data, 14);
            1;
        }
    }
}