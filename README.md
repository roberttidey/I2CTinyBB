# I2CTinyBBBB

- This library is a bitbang I2C driver primarily for ATTiny use
- It is targetted for PortB pins and allows any pins to be used for SDA and SCL
- Main byte output routine is written using inline assembler
- Can achieve i2c clock of 600KHz with 16MHz processor clock
- speed can be set using a delay factor, see .h file for details
- simplified api hides details of start stop etc.
- just set up a buffer containing all bytes needed to be sent.

### Functions
// Init
I2CInit(uint8_t sda, uint8_t scl, uint8_t delayCount);
// Write bytes
uint8_t I2CWrite(uint8_t iAddr, uint8_t *data, uint8_t iLen);
// Read bytes
uint8_t I2CRead(uint8_t iAddr, uint8_t *pData, uint8_t iLen);
// Read bytes from a reg start
uint8_t I2CReadReg(uint8_t iAddr, uint8_t reg, uint8_t *data, uint8_t iLen);


