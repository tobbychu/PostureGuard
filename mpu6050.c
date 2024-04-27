
/*
MPU6050 lib 0x05

copyright (c) Davide Gironi, 2012

Released under GPLv3.
Please refer to LICENSE file for licensing information.

Edited by Tobby Zhu to adapt to Atmega328PB and support multiple sensors
*/


#include <stdlib.h>
#include <string.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>

#include "uart.h"
#include "mpu6050.h"

#if MPU6050_GETATTITUDE == 1 || MPU6050_GETATTITUDE == 2
#include <math.h>  //include libm
#endif

//path to i2c fleury lib
#include MPU6050_I2CFLEURYPATH

volatile uint8_t buffer[50];


/************************************************************************/
/* get interface number from MPU Instance                                                                     */
/************************************************************************/
uint8_t getInterface(Instance instance) {
	uint8_t interface;
	if (instance == MPU6050_INSTANCE_1 || instance == MPU6050_INSTANCE_2) {
		interface = 0;
	} else if (instance == MPU6050_INSTANCE_3 || instance == MPU6050_INSTANCE_4) {
		interface = 1;
	}
	return interface;
}

uint8_t getAddr(Instance instance) {
	uint8_t mpuAddress;
	if (instance == MPU6050_INSTANCE_1 || instance == MPU6050_INSTANCE_3) {
		mpuAddress = (0x68 <<1);
	} else if (instance == MPU6050_INSTANCE_2 || instance == MPU6050_INSTANCE_4) {
		mpuAddress = (0x69 <<1);
	}
	return mpuAddress;
}


int getQuaternionIndex(Instance instance) {
	int qInd = 0;
	if (instance == MPU6050_INSTANCE_1) {
		qInd = 0;
	} else if (instance == MPU6050_INSTANCE_2) {
		qInd = 1;
	} else if (instance == MPU6050_INSTANCE_3) {
		qInd = 2;
	} else if (instance == MPU6050_INSTANCE_4) {
		qInd = 3;
	}
	return qInd;
}

	


/*
 * read bytes from chip register
 */
int8_t mpu6050_readBytes(Instance instance, uint8_t regAddr, uint8_t length, uint8_t *data) {
	uint8_t interface = getInterface(instance);
	uint8_t MPU6050_ADDR = getAddr(instance);
	uint8_t i = 0;
	int8_t count = 0;
	//sprintf(buffer, "WriteBytes ADDR=%d\n", MPU6050_ADDR);
	//UART_putstring(buffer);
	if(length > 0) {
		//request register
		i2c_start(interface);
		i2c_write(interface, MPU6050_ADDR);
		i2c_write(interface, regAddr);
		_delay_us(10);
		//read data
		i2c_start(interface);
		i2c_write(interface, (MPU6050_ADDR) | 1);
		for(i=0; i<length; i++) {
			count++;
			if(i==length-1)
				data[i] = i2c_readNak(interface);
			else
				data[i] = i2c_readAck(interface);
		}
		i2c_stop(interface);
	}
	return count;
}

/*
 * read 1 byte from chip register
 */
int8_t mpu6050_readByte(Instance instance, uint8_t regAddr, uint8_t *data) {
    return mpu6050_readBytes(instance, regAddr, 1, data);
}

/*
 * write bytes to chip register
 */
void mpu6050_writeBytes(Instance instance, uint8_t regAddr, uint8_t length, uint8_t* data) {
	uint8_t interface = getInterface(instance);
	uint8_t MPU6050_ADDR = getAddr(instance);
	

	
	if(length > 0) {
		//write data
		i2c_start(interface);
		i2c_write(interface, MPU6050_ADDR);
		i2c_write(interface, regAddr); //reg
		for (uint8_t i = 0; i < length; i++) {
			i2c_write(interface, (uint8_t) data[i]);
		}
		i2c_stop(interface);
	}
}

/*
 * write 1 byte to chip register
 */
void mpu6050_writeByte(Instance instance, uint8_t regAddr, uint8_t data) {
    return mpu6050_writeBytes(instance, regAddr, 1, &data);
}

/*
 * read bits from chip register
 */
int8_t mpu6050_readBits(Instance instance, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data) {
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    int8_t count = 0;
    if(length > 0) {
		uint8_t b;
		if ((count = mpu6050_readByte(instance, regAddr, &b)) != 0) {
			uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
			b &= mask;
			b >>= (bitStart - length + 1);
			*data = b;
		}
    }
    return count;
}

/*
 * read 1 bit from chip register
 */
int8_t mpu6050_readBit(Instance instance, uint8_t regAddr, uint8_t bitNum, uint8_t *data) {
    uint8_t b;
    uint8_t count = mpu6050_readByte(instance, regAddr, &b);
    *data = b & (1 << bitNum);
    return count;
}

/*
 * write bit/bits to chip register
 */
void mpu6050_writeBits(Instance instance, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) {
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
	if(length > 0) {
		uint8_t b = 0;
		if (mpu6050_readByte(instance, regAddr, &b) != 0) { //get current data
			uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
			data <<= (bitStart - length + 1); // shift data into correct position
			data &= mask; // zero all non-important bits in data
			b &= ~(mask); // zero all important bits in existing byte
			b |= data; // combine data with existing byte
			mpu6050_writeByte(instance, regAddr, b);
		}
	}
}

/*
 * write one bit to chip register
 */
void mpu6050_writeBit(Instance instance, uint8_t regAddr, uint8_t bitNum, uint8_t data) {
    uint8_t b;
    mpu6050_readByte(instance, regAddr, &b);
    b = (data != 0) ? (b | (1 << bitNum)) : (b & ~(1 << bitNum));
    mpu6050_writeByte(instance, regAddr, b);
}

/*
 * write word to chip register
 */
void mpu6050_writeWord(Instance instance, uint8_t regAddr, uint16_t data) {
	uint8_t interface = getInterface(instance);
	uint8_t MPU6050_ADDR = getAddr(instance);
	//write data
	i2c_start(interface);
	i2c_write(interface, MPU6050_ADDR);
	i2c_write(interface, regAddr); //reg
	i2c_write(interface, (uint8_t)(data >> 8)); // send MSB
	i2c_write(interface, (uint8_t)data); // send LSB
	i2c_stop(interface);
}


#if MPU6050_GETATTITUDE == 2

/*
 * set a chip memory bank
 */
void mpu6050_setMemoryBank(Instance instance, uint8_t bank, uint8_t prefetchEnabled, uint8_t userBank) {
    bank &= 0x1F;
    if (userBank) bank |= 0x20;
    if (prefetchEnabled) bank |= 0x40;
    mpu6050_writeByte(instance, MPU6050_RA_BANK_SEL, bank);
}

/*
 * set memory start address
 */
void mpu6050_setMemoryStartAddress(Instance instance, uint8_t address) {
	mpu6050_writeByte(instance, MPU6050_RA_MEM_START_ADDR, address);
}

/*
 * read a memory block
 */
void mpu6050_readMemoryBlock(Instance instance, uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address) {
	mpu6050_setMemoryBank(instance, bank, 0, 0);
	mpu6050_setMemoryStartAddress(instance, address);
    uint8_t chunkSize;
    for (uint16_t i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;

        // read the chunk of data as specified
        mpu6050_readBytes(instance, MPU6050_RA_MEM_R_W, chunkSize, data + i);

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) bank++;
            mpu6050_setMemoryBank(instance, bank, 0, 0);
            mpu6050_setMemoryStartAddress(instance, address);
        }
    }
}

/*
 * write a memory block
 */
uint8_t mpu6050_writeMemoryBlock(Instance instance, const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, uint8_t verify, uint8_t useProgMem) {
	mpu6050_setMemoryBank(instance, bank, 0, 0);
	mpu6050_setMemoryStartAddress(instance, address);
    uint8_t chunkSize;
    uint8_t *verifyBuffer = 0;
    uint8_t *progBuffer = 0;
    uint16_t i;
    uint8_t j;
    if (verify) verifyBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    if (useProgMem) progBuffer = (uint8_t *)malloc(MPU6050_DMP_MEMORY_CHUNK_SIZE);
    for (i = 0; i < dataSize;) {
        // determine correct chunk size according to bank position and data size
        chunkSize = MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // make sure we don't go past the data size
        if (i + chunkSize > dataSize) chunkSize = dataSize - i;

        // make sure this chunk doesn't go past the bank boundary (256 bytes)
        if (chunkSize > 256 - address) chunkSize = 256 - address;

        if (useProgMem) {
            // write the chunk of data as specified
            for (j = 0; j < chunkSize; j++) progBuffer[j] = pgm_read_byte(data + i + j);
        } else {
            // write the chunk of data as specified
            progBuffer = (uint8_t *)data + i;
        }

        mpu6050_writeBytes(instance, MPU6050_RA_MEM_R_W, chunkSize, progBuffer);

        // verify data if needed
        if (verify && verifyBuffer) {
        	mpu6050_setMemoryBank(instance, bank, 0, 0);
            mpu6050_setMemoryStartAddress(instance, address);
            mpu6050_readBytes(instance, MPU6050_RA_MEM_R_W, chunkSize, verifyBuffer);
            if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) {
                free(verifyBuffer);
                if (useProgMem) free(progBuffer);
                return 0; // uh oh.
            }
        }

        // increase byte index by [chunkSize]
        i += chunkSize;

        // uint8_t automatically wraps to 0 at 256
        address += chunkSize;

        // if we aren't done, update bank (if necessary) and address
        if (i < dataSize) {
            if (address == 0) bank++;
            mpu6050_setMemoryBank(instance, bank, 0, 0);
            mpu6050_setMemoryStartAddress(instance, address);
        }
    }
    if (verify) free(verifyBuffer);
    if (useProgMem) free(progBuffer);
    return 1;
}

/*
 * write a dmp configuration set
 */
uint8_t mpu6050_writeDMPConfigurationSet(Instance instance, const uint8_t *data, uint16_t dataSize, uint8_t useProgMem) {
    uint8_t *progBuffer = 0;
    uint8_t success, special;
    uint16_t i, j;
    if (useProgMem) {
        progBuffer = (uint8_t *)malloc(8); // assume 8-byte blocks, realloc later if necessary
    }

    // config set data is a long string of blocks with the following structure:
    // [bank] [offset] [length] [byte[0], byte[1], ..., byte[length]]
    uint8_t bank, offset, length;
    for (i = 0; i < dataSize;) {
        if (useProgMem) {
            bank = pgm_read_byte(data + i++);
            offset = pgm_read_byte(data + i++);
            length = pgm_read_byte(data + i++);
        } else {
            bank = data[i++];
            offset = data[i++];
            length = data[i++];
        }

        // write data or perform special action
        if (length > 0) {
            // regular block of data to write
            if (useProgMem) {
                if (sizeof(progBuffer) < length) progBuffer = (uint8_t *)realloc(progBuffer, length);
                for (j = 0; j < length; j++) progBuffer[j] = pgm_read_byte(data + i + j);
            } else {
                progBuffer = (uint8_t *)data + i;
            }
            success = mpu6050_writeMemoryBlock(instance, progBuffer, length, bank, offset, 1, 0);
            i += length;
        } else {
            // special instruction
            // NOTE: this kind of behavior (what and when to do certain things)
            // is totally undocumented. This code is in here based on observed
            // behavior only, and exactly why (or even whether) it has to be here
            // is anybody's guess for now.
            if (useProgMem) {
                special = pgm_read_byte(data + i++);
            } else {
                special = data[i++];
            }
            if (special == 0x01) {
                // enable DMP-related interrupts

            	//mpu6050_writeBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_ZMOT_BIT, 1); //setIntZeroMotionEnabled
            	//mpu6050_writeBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_FIFO_OFLOW_BIT, 1); //setIntFIFOBufferOverflowEnabled
            	//mpu6050_writeBit(MPU6050_RA_INT_ENABLE, MPU6050_INTERRUPT_DMP_INT_BIT, 1); //setIntDMPEnabled
            	mpu6050_writeByte(instance, MPU6050_RA_INT_ENABLE, 0x32);  // single operation

                success = 1;
            } else {
                // unknown special command
                success = 0;
            }
        }

        if (!success) {
            if (useProgMem) free(progBuffer);
            return 0; // uh oh
        }
    }
    if (useProgMem) free(progBuffer);
    return 1;
}

/*
 * get the fifo count
 */
uint16_t mpu6050_getFIFOCount(Instance instance) {
	mpu6050_readBytes(instance, MPU6050_RA_FIFO_COUNTH, 2, (uint8_t *)buffer);
    return (((uint16_t)buffer[0]) << 8) | buffer[1];
}

/*
 * read fifo bytes
 */
void mpu6050_getFIFOBytes(Instance instance, uint8_t *data, uint8_t length) {
	mpu6050_readBytes(instance, MPU6050_RA_FIFO_R_W, length, data);
}

/*
 * get the interrupt status
 */
uint8_t mpu6050_getIntStatus(Instance instance) {
	mpu6050_readByte(instance, MPU6050_RA_INT_STATUS, (uint8_t *)buffer);
    return buffer[0];
}

/*
 * reset fifo
 */
void mpu6050_resetFIFO(Instance instance) {
	mpu6050_writeBit(instance, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, 1);
}

/*
 * get gyro offset X
 */
int16_t mpu6050_getXGyroOffset(Instance instance) {
	mpu6050_readBytes(instance, MPU6050_RA_XG_OFFS_USRH, 2, (uint8_t *)buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}

/*
 * set gyro offset X
 */
void mpu6050_setXGyroOffset(Instance instance, int16_t offset) {
	mpu6050_writeWord(instance, MPU6050_RA_XG_OFFS_USRH, offset);
}

/*
 * get gyro offset Y
 */
int16_t mpu6050_getYGyroOffset(Instance instance) {
	mpu6050_readBytes(instance, MPU6050_RA_YG_OFFS_USRH, 2, (uint8_t *)buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}

/*
 * set gyro offset Y
 */
void mpu6050_setYGyroOffset(Instance instance, int16_t offset) {
	mpu6050_writeWord(instance, MPU6050_RA_YG_OFFS_USRH, offset);
}

/*
 * get gyro offset Z
 */
int16_t mpu6050_getZGyroOffset(Instance instance) {
	mpu6050_readBytes(instance, MPU6050_RA_ZG_OFFS_USRH, 2, (uint8_t *)buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}

/*
 * set gyro offset Z
 */
void mpu6050_setZGyroOffset(Instance instance, int16_t offset) {
	mpu6050_writeWord(instance, MPU6050_RA_ZG_OFFS_USRH, offset);
}

/*
 * get accel offset X
 */
int16_t mpu6050_getXAccelOffset(Instance instance) {
	mpu6050_readBytes(instance, MPU6050_RA_XA_OFFS_H, 2, (uint8_t *)buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}

/*
 * set accel offset X
 */
void mpu6050_setXAccelOffset(Instance instance, int16_t offset) {
	mpu6050_writeWord(instance, MPU6050_RA_XA_OFFS_H, offset);
}

/*
 * get accel offset Y
 */
int16_t mpu6050_getYAccelOffset(Instance instance) {
	mpu6050_readBytes(instance, MPU6050_RA_YA_OFFS_H, 2, (uint8_t *)buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}

/*
 * set accel offset Y
 */
void mpu6050_setYAccelOffset(Instance instance, int16_t offset) {
	mpu6050_writeWord(instance, MPU6050_RA_YA_OFFS_H, offset);
}

/*
 * get accel offset Z
 */
int16_t mpu6050_getZAccelOffset(Instance instance) {
	mpu6050_readBytes(instance, MPU6050_RA_ZA_OFFS_H, 2, (uint8_t *)buffer);
	return (((int16_t)buffer[0]) << 8) | buffer[1];
}

/*
 * set accel offset Z
 */
void mpu6050_setZAccelOffset(Instance instance, int16_t offset) {
	mpu6050_writeWord(instance, MPU6050_RA_ZA_OFFS_H, offset);
}
#endif

/*
 * set sleep disabled
 */
void mpu6050_setSleepDisabled(Instance instance) {
	mpu6050_writeBit(instance, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 0);
}

/*
 * set sleep enabled
 */
void mpu6050_setSleepEnabled(Instance instance) {
	mpu6050_writeBit(instance, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 1);
}


/*
 * test connectino to chip
 */
uint8_t mpu6050_testConnection(Instance instance) {
	mpu6050_readBits(instance, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, (uint8_t *)buffer);
	if(buffer[0] == 0x34)
		return 1;
	else
		return 0;
}

/*
 * initialize the accel and gyro
 */
void mpu6050_init(Instance instance) {
	uint8_t interface = getInterface(instance);
	
	#if MPU6050_I2CINIT == 1
	//init i2c
	i2c_init(interface);
	_delay_us(10);
	#endif

	//allow mpu6050 chip clocks to start up
	_delay_ms(100);

	//set sleep disabled
	mpu6050_setSleepDisabled(instance);
	//wake up delay needed sleep disabled
	_delay_ms(10);

	//set clock source
	//  it is highly recommended that the device be configured to use one of the gyroscopes (or an external clock source)
	//  as the clock reference for improved stability
	mpu6050_writeBits(instance, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, MPU6050_CLOCK_PLL_XGYRO);
	//set DLPF bandwidth to 42Hz
	mpu6050_writeBits(instance, MPU6050_RA_CONFIG, MPU6050_CFG_DLPF_CFG_BIT, MPU6050_CFG_DLPF_CFG_LENGTH, MPU6050_DLPF_BW_42);
    //set sampe rate
	mpu6050_writeByte(instance, MPU6050_RA_SMPLRT_DIV, 4); //1khz / (1 + 4) = 200Hz
	//set gyro range
	mpu6050_writeBits(instance, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, MPU6050_GYRO_FS);
	//set accel range
	mpu6050_writeBits(instance, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, MPU6050_ACCEL_FS);

	#if MPU6050_GETATTITUDE == 1
	MPU6050_TIMERINIT
	#endif
}

//can not accept many request if we alreay have getattitude requests
/*
 * get raw data
 */
void mpu6050_getRawData(Instance instance, int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
	mpu6050_readBytes(instance, MPU6050_RA_ACCEL_XOUT_H, 14, (uint8_t *)buffer);

    *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    *az = (((int16_t)buffer[4]) << 8) | buffer[5];
    *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t)buffer[12]) << 8) | buffer[13];
}

/*
 * get raw data converted to g and deg/sec values
 */
void mpu6050_getConvData(Instance instance, double* axg, double* ayg, double* azg, double* gxds, double* gyds, double* gzds) {
	int16_t ax = 0;
	int16_t ay = 0;
	int16_t az = 0;
	int16_t gx = 0;
	int16_t gy = 0;
	int16_t gz = 0;
	mpu6050_getRawData(instance, &ax, &ay, &az, &gx, &gy, &gz);

	#if MPU6050_CALIBRATEDACCGYRO == 1
    *axg = (double)(ax-MPU6050_AXOFFSET)/MPU6050_AXGAIN;
    *ayg = (double)(ay-MPU6050_AYOFFSET)/MPU6050_AYGAIN;
    *azg = (double)(az-MPU6050_AZOFFSET)/MPU6050_AZGAIN;
    *gxds = (double)(gx-MPU6050_GXOFFSET)/MPU6050_GXGAIN;
	*gyds = (double)(gy-MPU6050_GYOFFSET)/MPU6050_GYGAIN;
	*gzds = (double)(gz-MPU6050_GZOFFSET)/MPU6050_GZGAIN;
	#else
    *axg = (double)(ax)/MPU6050_AGAIN;
    *ayg = (double)(ay)/MPU6050_AGAIN;
    *azg = (double)(az)/MPU6050_AGAIN;
    *gxds = (double)(gx)/MPU6050_GGAIN;
	*gyds = (double)(gy)/MPU6050_GGAIN;
	*gzds = (double)(gz)/MPU6050_GGAIN;
	#endif
}

#if MPU6050_GETATTITUDE == 1

volatile float q0[4] = {1.0f, 1.0f, 1.0f, 1.0f}, q1[4] = {0.0f, 0.0f, 0.0f, 0.0f}, q2[4] = {0.0f, 0.0f, 0.0f, 0.0f}, q3[4] = {0.0f, 0.0f, 0.0f, 0.0f};
volatile float integralFBx[4] = {0.0f, 0.0f, 0.0f, 0.0f},  integralFBy[4] = {0.0f, 0.0f, 0.0f, 0.0f}, integralFBz[4] = {0.0f, 0.0f, 0.0f, 0.0f};
/*
 * Mahony update function (for 6DOF)
 */
void mpu6050_mahonyUpdate(Instance instance, float gx, float gy, float gz, float ax, float ay, float az) {
	cli();
	float norm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	int qInd = getQuaternionIndex(instance);
	
	//sprintf(buffer, "Update Mahony\n");
	//UART_putstring(buffer);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		norm = sqrt(ax * ax + ay * ay + az * az);
		ax /= norm;
		ay /= norm;
		az /= norm;

		// Estimated direction of gravity and vector perpendicular to magnetic flux
		halfvx = q1[qInd] * q3[qInd] - q0[qInd] * q2[qInd];
		halfvy = q0[qInd] * q1[qInd] + q2[qInd] * q3[qInd];
		halfvz = q0[qInd] * q0[qInd] - 0.5f + q3[qInd] * q3[qInd];

		// Error is sum of cross product between estimated and measured direction of gravity
		halfex = (ay * halfvz - az * halfvy);
		halfey = (az * halfvx - ax * halfvz);
		halfez = (ax * halfvy - ay * halfvx);

		// Compute and apply integral feedback if enabled
		if(mpu6050_mahonytwoKiDef > 0.0f) {
			integralFBx[qInd] += mpu6050_mahonytwoKiDef * halfex * (1.0f / mpu6050_mahonysampleFreq);	// integral error scaled by Ki
			integralFBy[qInd] += mpu6050_mahonytwoKiDef * halfey * (1.0f / mpu6050_mahonysampleFreq);
			integralFBz[qInd] += mpu6050_mahonytwoKiDef * halfez * (1.0f / mpu6050_mahonysampleFreq);
			gx += integralFBx[qInd];	// apply integral feedback
			gy += integralFBy[qInd];
			gz += integralFBz[qInd];
		} else {
			integralFBx[qInd] = 0.0f;	// prevent integral windup
			integralFBy[qInd] = 0.0f;
			integralFBz[qInd] = 0.0f;
		}

		// Apply proportional feedback
		gx += mpu6050_mahonytwoKpDef * halfex;
		gy += mpu6050_mahonytwoKpDef * halfey;
		gz += mpu6050_mahonytwoKpDef * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * (1.0f / mpu6050_mahonysampleFreq));		// pre-multiply common factors
	gy *= (0.5f * (1.0f / mpu6050_mahonysampleFreq));
	gz *= (0.5f * (1.0f / mpu6050_mahonysampleFreq));
	qa = q0[qInd];
	qb = q1[qInd];
	qc = q2[qInd];
	q0[qInd] += (-qb * gx - qc * gy - q3[qInd] * gz);
	q1[qInd] += (qa * gx + qc * gz - q3[qInd] * gy);
	q2[qInd] += (qa * gy - qb * gz + q3[qInd] * gx);
	q3[qInd] += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	norm = sqrt(q0[qInd] * q0[qInd] + q1[qInd] * q1[qInd] + q2[qInd] * q2[qInd] + q3[qInd] * q3[qInd]);
	q0[qInd] /= norm;
	q1[qInd] /= norm;
	q2[qInd] /= norm;
	q3[qInd] /= norm;
	sei();
}

/*
 * update quaternion
 */
void mpu6050_updateQuaternion(Instance instance) {
	cli();
	int16_t ax = 0;
	int16_t ay = 0;
	int16_t az = 0;
	int16_t gx = 0;
	int16_t gy = 0;
	int16_t gz = 0;
	double axg = 0;
	double ayg = 0;
	double azg = 0;
	double gxrs = 0;
	double gyrs = 0;
	double gzrs = 0;
	
	//sprintf(buffer, "Update Quaternion\n");
	//UART_putstring(buffer);

	//get raw data
	while(1) {
		//sprintf(buffer, "Getting raw data\n");
		//UART_putstring(buffer);
		
		mpu6050_readBit(instance, MPU6050_RA_INT_STATUS, MPU6050_INTERRUPT_DATA_RDY_BIT, (uint8_t *)buffer);
		if(buffer[0])
			break;
		_delay_us(10);
	}

	mpu6050_readBytes(instance, MPU6050_RA_ACCEL_XOUT_H, 14, (uint8_t *)buffer);
    ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    az = (((int16_t)buffer[4]) << 8) | buffer[5];
    gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    gz = (((int16_t)buffer[12]) << 8) | buffer[13];

	#if MPU6050_CALIBRATEDACCGYRO == 1
	axg = (double)(ax-MPU6050_AXOFFSET)/MPU6050_AXGAIN;
	ayg = (double)(ay-MPU6050_AYOFFSET)/MPU6050_AYGAIN;
	azg = (double)(az-MPU6050_AZOFFSET)/MPU6050_AZGAIN;
	gxrs = (double)(gx-MPU6050_GXOFFSET)/MPU6050_GXGAIN*0.01745329; //degree to radians
	gyrs = (double)(gy-MPU6050_GYOFFSET)/MPU6050_GYGAIN*0.01745329; //degree to radians
	gzrs = (double)(gz-MPU6050_GZOFFSET)/MPU6050_GZGAIN*0.01745329; //degree to radians
	#else
	axg = (double)(ax)/MPU6050_AGAIN;
	ayg = (double)(ay)/MPU6050_AGAIN;
	azg = (double)(az)/MPU6050_AGAIN;
	gxrs = (double)(gx)/MPU6050_GGAIN*0.01745329; //degree to radians
	gyrs = (double)(gy)/MPU6050_GGAIN*0.01745329; //degree to radians
	gzrs = (double)(gz)/MPU6050_GGAIN*0.01745329; //degree to radians
	#endif
	
	sei();

    //compute data
    mpu6050_mahonyUpdate(instance, gxrs, gyrs, gzrs, axg, ayg, azg);
}

void mpu6050_resetQuaternion(Instance instance) {
	int qInd = getQuaternionIndex(instance);
	q0[qInd] = 1.0f;
	q1[qInd] = 0.0f;
	q2[qInd] = 0.0f;
	q3[qInd] = 0.0f;
	integralFBx[qInd] = 0.0f;
	integralFBy[qInd] = 0.0f;
	integralFBz[qInd] = 0.0f;
	mpu6050_updateQuaternion(instance);
}

/*
 * update timer for attitude
 */
MPU6050_TIMERUPDATE

/*
 * get quaternion
 */
void mpu6050_getQuaternion(Instance instance, double *qw, double *qx, double *qy, double *qz) {
	int qInd = getQuaternionIndex(instance);
	*qw = q0[qInd];
	*qx = q1[qInd];
	*qy = q2[qInd];
	*qz = q3[qInd];
}

/*
 * get euler angles
 * aerospace sequence, to obtain sensor attitude:
 * 1. rotate around sensor Z plane by yaw
 * 2. rotate around sensor Y plane by pitch
 * 3. rotate around sensor X plane by roll
 */
void mpu6050_getRollPitchYaw(Instance instance, double *roll, double *pitch, double *yaw) {
	int qInd = getQuaternionIndex(instance);
	*yaw = atan2(2*q1[qInd]*q2[qInd] - 2*q0[qInd]*q3[qInd], 2*q0[qInd]*q0[qInd] + 2*q1[qInd]*q1[qInd] - 1);
	*pitch = -asin(2*q1[qInd]*q3[qInd] + 2*q0[qInd]*q2[qInd]);
	*roll = atan2(2*q2[qInd]*q3[qInd] - 2*q0[qInd]*q1[qInd], 2*q0[qInd]*q0[qInd] + 2*q3[qInd]*q3[qInd] - 1);
}

#endif

