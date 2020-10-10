/*
* I2CTinyBB BitBang for ATTiny25,45,85 family
*/

#ifndef I2CTINYBB
#define I2CTINYBB

// Read bytes
uint8_t I2CRead(uint8_t iAddr, uint8_t *pData, uint8_t iLen);

// Read bytes from a reg start
uint8_t I2CReadReg(uint8_t iAddr, uint8_t reg, uint8_t *data, uint8_t iLen);

// Write bytes
uint8_t I2CWrite(uint8_t iAddr, uint8_t *data, uint8_t iLen);

// Init
void I2CInit(uint8_t sda, uint8_t scl, uint8_t delayCount);
/*
CPU						8MHz	16MHz	
delayCount	1	I2C-SCL	300KHz	600KHz
delayCount	4	I2C-SCL	200KHz	400KHz
delayCount	10	I2C-SCL	100KHz	200KHz
delayCount	25	I2C-SCL	50KHz	100KHz
*/
#endif

