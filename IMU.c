/*
    These are the base functions for the BerryIMUv1 and BerryIMUv2


    Both the BerryIMUv1 and BerryIMUv2 are supported

    Feel free to do whatever you like with this code.
    Distributed as-is; no warranty is given.

    http://ozzmaker.com/
*/

#include <stdint.h>
#include "linux/i2c-dev.h"
#include "LSM9DS1.h"

int file;
int LSM9DS1 = 0;

void  readBlock(uint8_t command, uint8_t size, uint8_t *data){
	int result = i2c_smbus_read_i2c_block_data(file, command, size, data);
	if (result != size){
		printf("Failed to read block from I2C.");
		exit(1);
	}
}

void selectDevice(int file, int addr){
	if (ioctl(file, I2C_SLAVE, addr) < 0) {
		printf("Failed to select I2C device.");
	}
}


void readACC(int  a[]){
	uint8_t block[6];
	selectDevice(file,LSM9DS1_ACC_ADDRESS);
	readBlock(0x80 |  LSM9DS1_OUT_X_L_XL, sizeof(block), block);       
	
	// Combine readings for each axis.
	a[0] = (int16_t)(block[0] | block[1] << 8);
	a[1] = ((int16_t)(block[2] | block[3] << 8))- 60;
	a[2] = ((int16_t)(block[4] | block[5] << 8)) - 1330;
}


void readMAG(int  m[]){
	uint8_t block[6];
	selectDevice(file,LSM9DS1_MAG_ADDRESS);
	readBlock(0x80 |  LSM9DS1_OUT_X_L_M, sizeof(block), block);    

	// Combine readings for each axis.
	m[0] = (int16_t)(block[0] | block[1] << 8);
	m[1] = (int16_t)(block[2] | block[3] << 8);
	m[2] = (int16_t)(block[4] | block[5] << 8);
}


void readGYR(int g[]){
	uint8_t block[6];
	selectDevice(file,LSM9DS1_GYR_ADDRESS);
	readBlock(0x80 |  LSM9DS1_OUT_X_L_M, sizeof(block), block);    
	
	// Combine readings for each axis.
	g[0] = (int16_t)(block[0] | block[1] << 8);
	g[1] = (int16_t)(block[2] | block[3] << 8);
	g[2] = (int16_t)(block[4] | block[5] << 8);
}


void writeAccReg(uint8_t reg, uint8_t value){
	selectDevice(file,LSM9DS1_ACC_ADDRESS);

	int result = i2c_smbus_write_byte_data(file, reg, value);
	if (result == -1){
		printf ("Failed to write byte to I2C Acc.");
		exit(1);
	}
}

void writeMagReg(uint8_t reg, uint8_t value){
	selectDevice(file,LSM9DS1_MAG_ADDRESS);
  
	int result = i2c_smbus_write_byte_data(file, reg, value);
	if (result == -1){
		printf("Failed to write byte to I2C Mag.");
		exit(1);
	}
}


void writeGyrReg(uint8_t reg, uint8_t value){
	selectDevice(file,LSM9DS1_GYR_ADDRESS);
  
	int result = i2c_smbus_write_byte_data(file, reg, value);
	if (result == -1){
		printf("Failed to write byte to I2C Gyr.");
		exit(1);
	}
}


void detectIMU(){

	__u16 block[I2C_SMBUS_BLOCK_MAX];
	int res, bus,  size;

	char filename[20];
	sprintf(filename, "/dev/i2c-%d", 1);
	file = open(filename, O_RDWR);
	if (file<0) {
		printf("Unable to open I2C bus!");
			exit(1);
	}

	//Detect if BerryIMUv2 (Which uses a LSM9DS1) is connected
	selectDevice(file,LSM9DS1_MAG_ADDRESS);
	int LSM9DS1_WHO_M_response = i2c_smbus_read_byte_data(file, LSM9DS1_WHO_AM_I_M);

	selectDevice(file,LSM9DS1_GYR_ADDRESS);	
	int LSM9DS1_WHO_XG_response = i2c_smbus_read_byte_data(file, LSM9DS1_WHO_AM_I_XG);

    if (LSM9DS1_WHO_XG_response == 0x68 && LSM9DS1_WHO_M_response == 0x3d){
		printf ("\n\n\n#####   BerryIMUv2/LSM9DS1  DETECTED    #####\n\n");
		LSM9DS1 = 1;
	}
  
	if (!LSM9DS1){
		printf ("NO IMU DETECTED\n");
		exit(1);
	}
}

void calibrarion(float acc[], float gyro[]){
	int  acc_raw[3]={0};
	int  gyr_raw[3]={0};
	for(int i=0; i<32; i++){
		readACC(acc_raw);
		acc[0] += acc_raw[0];
		acc[1] += acc_raw[1];
		acc[2] += acc_raw[2];
		readGYR(gyr_raw);
		gyro[0] += gyr_raw[0];
		gyro[1] += gyr_raw[1];
		gyro[2] += gyr_raw[2];
	}
	acc[0] = acc[0] /32.0;
	acc[1] = acc[1] /32.0;
	acc[2] = acc[2] /32.0;

	gyro[0] = gyro[0] /32.0;
	gyro[1] = gyro[1] /32.0;
	gyro[2] = gyro[2] /32.0;
}


void enableIMU(){
	if (LSM9DS1){//For BerryIMUv2
		// Enable the gyroscope
		writeGyrReg(LSM9DS1_CTRL_REG4,0b00111000);      // z, y, x axis enabled for gyro
		writeGyrReg(LSM9DS1_CTRL_REG1_G,0b10111000);    // Gyro ODR = 476Hz, 2000 dps
		writeGyrReg(LSM9DS1_ORIENT_CFG_G,0b10111000);   // Swap orientation 

		// Enable the accelerometer
		writeAccReg(LSM9DS1_CTRL_REG5_XL,0b00111000);   // z, y, x axis enabled for accelerometer
		writeAccReg(LSM9DS1_CTRL_REG6_XL,0b00101000);   // +/- 16g

		//Enable the magnetometer
		writeMagReg(LSM9DS1_CTRL_REG1_M, 0b10011100);   // Temp compensation enabled,Low power mode mode,80Hz ODR
		writeMagReg(LSM9DS1_CTRL_REG2_M, 0b01000000);   // +/-12gauss
		writeMagReg(LSM9DS1_CTRL_REG3_M, 0b00000000);   // continuos update
		writeMagReg(LSM9DS1_CTRL_REG4_M, 0b00000000);   // lower power mode for Z axis
	}

}



