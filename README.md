# I2CTinyBBBB

- This library is a bitbang I2C driver primarily for ATTiny use
- It is targetted for PortB pins and allows any pins to be used for SDA and SCL
- Can achieve i2c clock of 600KHz with 16MHz processor clock
- speed can be set using a delay factor, see .h file for details
- simplified api hides details of start stop etc.
- just set up a buffer containing all bytes needed to be sent.
- #define USE_ASM at top of cpp file allows using high speed asm routines for byteout and bytein
- #define OPTIMISE_SPEED at top of cpp file enforces inline code for byteout to speed up at the expense of some extra program

### Functions

- I2CInit(uint8_t sda, uint8_t scl, uint8_t delayCount);
- uint8_t I2CWrite(uint8_t iAddr, uint8_t *data, uint8_t iLen);
- uint8_t I2CRead(uint8_t iAddr, uint8_t *data, uint8_t iLen);
- uint8_t I2CReadReg(uint8_t iAddr, uint8_t reg, uint8_t *data, uint8_t iLen);


