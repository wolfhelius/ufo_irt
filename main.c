/*

	MLX90614

*/
#include "avr_compiler.h"
#include "twi_master_driver.h"
#include "twi_slave_driver.h"
#include <string.h>
#include <util/delay.h>	


/*! Defining an example slave address. */
// this is the slave address of the device itself, not a slave it talks to
#define SLAVE_ADDRESS    0x55

/*! CPU speed 2MHz, BAUDRATE 100kHz and Baudrate Register Settings */
#define CPU_SPEED   2000000
#define BAUDRATE	 100000
#define TWI_BAUDSETTING TWI_BAUD(CPU_SPEED, BAUDRATE)

/*! Defining number of bytes in local arrays. */
#define SENSOR_NVAL     1
#define DEBUG_NVAL	    2  // debug >= data
#define DATA_BYTES      4  // 4 = float, 2 = int16, 1 = byte

#define FIRMWARE_VERSION 0
#define HARDWARE_VERSION 0

/*! MLX defines */
#define MLX90614_DEFAULT_ADDRESS 0x5A
uint8_t MLX_ADDRESS = MLX90614_DEFAULT_ADDRESS;

/* Global variables */
TWI_Master_t twieMaster;    /*!< TWI master module. */
TWI_Slave_t twicSlave;      /*!< TWI slave module. */

float data_values[DEBUG_NVAL] = {-99.0, -99.0};
float null_value = -99.0;
uint8_t sensor_sid[SENSOR_NVAL] = {0x10};
uint8_t debug_sid[DEBUG_NVAL] = {0x10, 0x11};

float * return_datum;

uint8_t data_ready = 0;

uint8_t command = 0;
uint8_t arg1 = 0;
uint8_t arg2 = 0;
uint8_t iSID = 0;
uint8_t return_value;

//shamelessly taken from http://www.phanderson.com/picaxe/mlx90614.html
uint8_t calcCRC(uint8_t crc, uint8_t b) {
    
    b = b ^ crc;
    
    for (int i = 0; i < 8; i++) {
        if (b & 0x80) b = (b << 1) ^ 0x07;
        else          b = (b << 1);
    }
    return b;
}

/*! MLX Functions */
uint8_t MLX_readSensors(float *T_BODY, float *T_IR){

	twieMaster.readData[0] = 0;
	twieMaster.readData[1] = 0;
	twieMaster.readData[2] = 0;

	uint8_t command = 0x06;
	uint8_t twi_status = TWI_MasterWriteRead(&twieMaster, MLX_ADDRESS, &command, 1, 3);
	while (twieMaster.status != TWIM_STATUS_READY); 
    uint16_t rawB = (twieMaster.readData[1]<<8) + (twieMaster.readData[0]&0xFC); 
    (*T_BODY) = ((float) rawB * 0.02) - 273.15;

	twieMaster.readData[0] = 0;
	twieMaster.readData[1] = 0;
	twieMaster.readData[2] = 0;

	command = 0x07;
	twi_status = TWI_MasterWriteRead(&twieMaster, MLX_ADDRESS, &command, 1, 3);
	while (twieMaster.status != TWIM_STATUS_READY); 
    uint16_t rawIR = (twieMaster.readData[1]<<8) + (twieMaster.readData[0]&0xFC); 
    (*T_IR) = ((float) rawIR * 0.02) - 273.15;

    return twieMaster.bytesRead; 

}

uint8_t MlxRedefineSlaveAddress(uint8_t OldSlaveAddress, uint8_t NewSlaveAddress){

    if (NewSlaveAddress > 0x77) return 1;

    uint8_t ret, crc;
    //write to EEPROM address 0b01110 with command 0b00101110 (0x2E)
    uint8_t cmd = 0x2E;
    
    //first gotta clear EEPROM by writing zero
    crc = 0;
    crc = calcCRC(crc, OldSlaveAddress<<1);  //slave address
    crc = calcCRC(crc, cmd);                       //eeprom address
    crc = calcCRC(crc, 0);  //low byte
    crc = calcCRC(crc, 0);  //high byte

	// write the new slave address to the old slave address
	uint8_t command[4] = {cmd, 0, 0, crc};
	uint8_t twi_status = TWI_MasterWriteRead(&twieMaster, OldSlaveAddress, &command, 4, 0);
	while (twieMaster.status != TWIM_STATUS_READY); 

    crc = 0;
    crc = calcCRC(crc, OldSlaveAddress<<1);  //slave address
    crc = calcCRC(crc, cmd);                         //eeprom address
    crc = calcCRC(crc, NewSlaveAddress); //low byte
    crc = calcCRC(crc, 0);          //high byte

    command[1] = NewSlaveAddress;
    command[3] = crc;
	twi_status = TWI_MasterWriteRead(&twieMaster, OldSlaveAddress, &command, 4, 0);
	while (twieMaster.status != TWIM_STATUS_READY); 
    
	// make sure we can read the new slave address

	twieMaster.readData[0] = 0;
	twieMaster.readData[1] = 0;
	twieMaster.readData[2] = 0;

	command[0] = 0x06;
	twi_status = TWI_MasterWriteRead(&twieMaster, MLX_ADDRESS, &command, 1, 3);
	while (twieMaster.status != TWIM_STATUS_READY); 
    uint16_t rawB = (twieMaster.readData[1]<<8) + (twieMaster.readData[0]&0xFC); 
    float T = ((float) rawB * 0.02) - 273.15;

    if (T>-40 && T<60)  MLX_ADDRESS = NewSlaveAddress;

    return MLX_ADDRESS;



	// twieMaster.readData[0] = 0;

	// // make sure we can read the new slave address
	// twi_status = TWI_MasterWriteRead(&twieMaster, NewSlaveAddress, &command, 1, 2);
	// while (twieMaster.status != TWIM_STATUS_READY); 

	// // response tells us whether we reached the new slave
	// uint8_t response = twieMaster.readData[0]
	// if (response == NewSlaveAddress){
	// 	return NewSlaveAddress;
	// } else {
	// 	return OldSlaveAddress;
	// }

}


uint8_t matchSID(uint8_t SID){
	for (uint8_t iSID = 0; iSID<DEBUG_NVAL; iSID++){
		if(debug_sid[iSID] == SID) return iSID;
	}
	return -1;
}

void collectSensorData(void){

	float T_BODY = null_value; 
	float T_IR = null_value; 
	uint8_t rv = MLX_readSensors(&T_BODY, &T_IR);

	data_values[0] = T_IR; 
	data_values[1] = T_BODY;

	data_ready = 1; 
}

// Our slave is always on port C
void TWIC_SlaveProcessData(void)
{

	// uint8_t bufIndex = twicSlave.bytesReceived;
	command = twicSlave.receivedData[0];
	if (twicSlave.bytesReceived>0) arg1 = twicSlave.receivedData[1];
	if (twicSlave.bytesReceived>1) arg2 = twicSlave.receivedData[2];

	switch (command){
		case 0x10:
			// data ready query
			twicSlave.sendData[0] = data_ready;
			break;
		case 0x20:
			// send sensor value, regular or debug
			iSID = matchSID(arg1);
			if (iSID>=0){
				return_datum = &data_values[iSID];			
			} else {
				return_datum = &null_value;
			}
			memcpy((void *)twicSlave.sendData, return_datum, sizeof(float));	
			data_ready = 0; 		
			break;
		case 0x30:
			// send sensor nval
			twicSlave.sendData[0] = SENSOR_NVAL;
			break;
		case 0x40:
			// send debug nval
			twicSlave.sendData[0] = DEBUG_NVAL;
			break;	
		case 0x50:
			// send sensor sid array
			memcpy((void *)twicSlave.sendData, sensor_sid, SENSOR_NVAL);
			break;	
		case 0x60:
			// send debug sid array
			memcpy((void *)twicSlave.sendData, debug_sid, DEBUG_NVAL);
			break;	
		case 0x70:
			// send data type (== number of bytes)
			twicSlave.sendData[0] = DATA_BYTES;
			break;
		case 0x80:
			// trigger new set of measurements
			collectSensorData();
			break;
		case 0x90:
			// For MLX - redefine slave address from default value
			return_value = MlxRedefineSlaveAddress(arg1, arg2);
			twicSlave.sendData[0] = return_value;
			break;

	}
}


int main(void)
{
	// the setup
	data_ready = 0;
	for (uint8_t i=0; i<DEBUG_NVAL; i++){
		data_values[i] = null_value;
	}
	
	// When daughter talks to chips on board, it is port E
	// When daughter talks to arduino as slave, it is port C
	/* Initialize TWI master. */
	TWI_MasterInit(&twieMaster,
	              &TWIE,
	              TWI_MASTER_INTLVL_HI_gc,
	              TWI_BAUDSETTING);

	/* Initialize TWI slave. */
	TWI_SlaveInitializeDriver(&twicSlave, &TWIC, TWIC_SlaveProcessData);
	TWI_SlaveInitializeModule(&twicSlave,
	                          SLAVE_ADDRESS,
	                          TWI_SLAVE_INTLVL_LO_gc);

	/* Enable LO interrupt level. */
	PMIC.CTRL |= PMIC_LOLVLEN_bm | PMIC_HILVLEN_bm;
	sei();

	// collect measurements as master
	//collectSensorData();

	// the loop
	while (1) {

	}
}

/*! TWIE Master Interrupt vector. */
ISR(TWIE_TWIM_vect){
	TWI_MasterInterruptHandler(&twieMaster);
}

/*! TWIC Slave Interrupt vector. */
ISR(TWIC_TWIS_vect){
	TWI_SlaveInterruptHandler(&twicSlave);
}
