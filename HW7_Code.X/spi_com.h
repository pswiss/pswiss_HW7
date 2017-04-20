#define CS LATBbits.LATB7       // chip select pin

void initSPI1() 
{
    // Initialize the SPI connection
    // Portions of code are taken from the example
    // Set up chip select pin as output
    TRISBbits.TRISB9 = 0;
    CS = 1;
    
    // Setup SPI1
    int rData;          // Dummy Variable (following convention of Microchip example)
    SPI1CON = 0;        // Turn off the SPI1 during setup
    rData = SPI1BUF;    // Clear the buffer by reading it
    
    SPI1BRG = 0x01;     // Set Baud rate to max
    SPI1STATbits.SPIROV = 0;    // Clear overflow bit
    SPI1CONbits.CKE = 1;        // Data changes when clock goes from hi to lo
    SPI1CONbits.MSTEN = 1;      // Master Operation
    SPI1CONbits.ON = 1;         //Turn on SPI1
        
    // Configure Pin Bits
    //Put SD01 on RPA1
    RPA1Rbits.RPA1R = 0b0011;
    // Put SDI1 on RPB8
    SDI1Rbits.SDI1R = 0b0100;
 }

unsigned char SPI1_IO(unsigned char write)
{
    SPI1BUF = write;    // Load character into the buffer

    while(!SPI1STATbits.SPIRBF) 
    { // wait to receive the byte
      ;
    }
    return SPI1BUF;     // Return the communication
}

void setVoltage(unsigned char channel, unsigned char voltage)
{
    // Function to pass information to SPI DAC
    
    // Logic to set voltage using SPI
    short message = 0b0011000000000000; // Basic message with Buffer off, Gain 1x, and !shdn active
    short mask;     // Dummy variable for bitwise manipulation of message
    
    mask = 0b1000000000000000*channel;  // Channel select using simple multiplication
    message = message | mask;           // Write channel to message
    
    mask = 0+voltage;           // Load voltage into mask
    mask = mask << 4;           // Shift bits into correct slots
    message = message | mask;   // Load voltage into message
    
    unsigned char msg1 = (message&0xFF00)>>8;   // First part of the message
    unsigned char msg2 = message&0x00FF;        // Second part of the message
        
    CS = 0;     // Enable writing to the DAC
    SPI1_IO(msg1); // MSB
    SPI1_IO(msg2); // LSB
    CS = 1;     // Disable writing to the DAC
}
