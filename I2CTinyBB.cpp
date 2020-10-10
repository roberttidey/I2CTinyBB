/*
* I2C_BitBang for ATTiny25,45,85 family
*/

#include <Arduino.h>
#include <I2CTinyBB.h>
#include <avr/io.h>
//macros
#define SDALOW DDRB |= iSDABit
#define SDAHIGH DDRB &= iSDABitI
#define SCLLOW DDRB |= iSCLBit
#define SCLHIGH DDRB &= iSCLBitI
#define SDAREAD PINB & iSDABit

volatile uint8_t iSDABit, iSCLBit, iSDABitI, iSCLBitI; // bit numbers of the ports
volatile uint8_t dataByte;
volatile uint8_t ack;
volatile int8_t iDelay;
volatile int8_t iDelayS;

void inline sleep_us(int8_t iDelay) {
	while (iDelay)   {
		__asm__ __volatile__ (
		"nop"); //just waiting 2 cycle
		iDelay--;
	}
}

// byte out
static inline uint8_t i2cByteOut(uint8_t b) {
	uint8_t i;
	dataByte = b;

//equivalent logic in c
/*	for (i=0; i<8; i++) {
		if (b & 0x80)
			SDAHIGH;
		else
			SDALOW;
		sleep_us(iDelayS);
		SCLHIGH;
		sleep_us(iDelay);
		SCLLOW;
		b <<= 1;
	}
	//get ack bit
	SDAHIGH;
	sleep_us(iDelay);
	SCLHIGH;
	sleep_us(iDelay);
	ack = SDAREAD;
	SCLLOW;
	return (ack == 0) ? 1:0; // low ACK bit OK*/

	asm(
	"ldi r29, 8 ; loop counter\n"
	"lds r28, (dataByte) ; byte to transmit\n"
	"in r26, 0x17 ; cache ddrg in register\n"
    "1: ; loop start\n"
    "lds r27, (iSDABitI) ; get data mask\n"
    "sbrs r28, 7 ; test msb and skip if 1\n"
    "rjmp 2f \n"
    "and r26, r27 ; data to input (high)\n"
    "rjmp 3f \n"
    "2: \n"
    "com r27 ; invert data mask \n"
    "or r26, r27 ;data to output (low)\n"
    "3: \n"
    "out 0x17, r26 ; update DDRB\n"
    "lds r27, (iDelayS) ;sleep for a bit\n"
    "4: \n"
    "dec r27 \n"
    "brne 4b \n"
    "lds r27, (iSCLBitI) ;get clock mask\n"
    "and r26, r27 ; clock to input (high)\n"
    "out 0x17, r26 ; update DDRB\n"
    "lds r27, (iDelay) ;sleep for a bit\n"
    "5: \n"
    "dec r27 \n"
    "brne 5b \n"
    "lds r27, (iSCLBit) ;get clock mask\n"
    "or r26, r27 ; clock to output (low)\n"
    "out 0x17, r26 ; update DDRB\n"
    "add r28, r28 ;shift data byte \n"
    "dec r29  ;decrement loop count  \n"
    "brne 1b ; loop end \n"
    "lds r27, (iSDABitI) ; data to input (high)\n"
    "and r26, r27 \n"
    "out 0x17, r26 \n"
    "lds r27, (iDelay) ;sleep for a bit\n"
    "6: \n"
    "dec r27 \n"
    "brne 6b \n"
    "lds r27, (iSCLBitI) ;get clock mask\n"
    "and r26, r27 ; clock to input (high)\n"
    "out 0x17, r26 ; update DDRB\n"
    "lds r27, (iDelay) ;sleep for a bit\n"
    "7: \n"
    "dec r27 \n"
    "brne 7b \n"
    "in r28, 0x16 ;read ack\n"
    "lds r27, (iSCLBit) ;get clock mask\n"
    "or r26, r27 ; clock to output (low)\n"
    "out 0x17, r26 ; update DDRB\n"
    "lds r27, (iSDABit) ;get data mask\n"
    "and r28, r27 ;isolate data bit\n"
    "brne 8f ; if set this is nack   \n"
	"ldi r28, 1 ; data 0 ack OK\n"
    "8:         \n"
    "sts (ack), r28    \n"
    : : : "r26", "r27", "r28", "r29" ); //just waiting 2 cycle
	return ack;
}


// Byte In
static inline uint8_t i2cByteIn(uint8_t bLast) {
	uint8_t i;
	uint8_t b = 0;

	SDAHIGH;
	for (i=0; i<8; i++) {
		sleep_us(iDelay);
		SCLHIGH;
		b <<= 1;
		if (SDAREAD != 0)
			b |= 1;
		SCLLOW;
	}
	if (bLast)
		SDAHIGH;
	else
		SDALOW;
	SCLHIGH;
	sleep_us(iDelay);
	SCLLOW;
	sleep_us(iDelay);
	SDALOW;
	return b;
}

//I2C START condition
static inline uint8_t i2cStart(uint8_t addr, uint8_t bRead) {
	uint8_t rc;
	SDALOW;
	sleep_us(iDelay);
	SCLLOW;
	addr <<= 1;
	if (bRead)
		addr++;
	rc = i2cByteOut(addr);
	return rc;
}

//I2C STOP condition
static inline void i2cStop() {
	SDALOW;
	sleep_us(iDelay);
	SCLHIGH;
	sleep_us(iDelay);
	SDAHIGH;
	sleep_us(iDelay);
}

static inline uint8_t i2cWrite(uint8_t *data, uint8_t iLen) {
	uint8_t b;
	uint8_t rc, count = iLen;

	rc = 1;
	while (count && rc == 1) {
		b = *data++;
		rc = i2cByteOut(b);
		if (rc == 1) { // success
			count--;
		}
	}
	return (rc == 1) ? (iLen - count) : 0; // 0 = error
}

static inline void i2cRead(uint8_t *data, uint8_t iLen) {
   while (iLen--) {
      *data++ = i2cByteIn(iLen == 0);
   }
}

// Initialize the I2C BitBang library
void I2CInit(uint8_t sda, uint8_t scl, uint8_t delayCount) {
	iSDABit = 1 << sda;
	iSDABitI = ~iSDABit;
	iSCLBit = 1 << scl;
	iSCLBitI = ~iSCLBit;
	iDelay = delayCount;
	iDelayS = iDelay - 2;
//	if(iDelay < 1) iDelay = 1;
	if(iDelayS < 1) iDelayS = 1;
	// direction in and data low
	DDRB &= ~(iSDABit | iSCLBit);
	PINB &= ~(iSDABit | iSCLBit);
}

// Write I2C data
uint8_t I2CWrite(uint8_t iAddr, uint8_t *data, uint8_t iLen) {
	uint8_t rc = 0;

	rc = i2cStart(iAddr, 0);
	if (rc == 1) { // slave sent ACK for its address
		rc = i2cWrite(data, iLen);
	}
	i2cStop();
	return rc; // returns the number of bytes sent or 0 for error
}

// Read bytes from a register start point
uint8_t I2CReadReg(uint8_t iAddr, uint8_t reg, uint8_t *data, uint8_t iLen) {
	uint8_t rc;

	rc = i2cStart(iAddr, 0); // start a write operation
	if (rc == 1) { // slave sent ACK for its address
		rc = i2cWrite(&reg, 1); // write the register we want to read from
		if (rc == 1) {
			i2cStop();
			rc = i2cStart(iAddr, 1); // start a read operation
			if (rc == 1) {
				i2cRead(data, iLen);
			}
		}
	}
	i2cStop();
	return rc; // returns 1 for success, 0 for error
}

// Read bytes
uint8_t I2CRead(uint8_t iAddr, uint8_t *data, uint8_t iLen) {
	uint8_t rc;

	rc = i2cStart(iAddr, 1);
	if (rc == 1) { // slave sent ACK for its address
		i2cRead(data, iLen);
	}
	i2cStop();
	return rc; // returns 1 for success, 0 for error
}
