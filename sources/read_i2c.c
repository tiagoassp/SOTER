#include "../headers/read_i2c.h"

SensorData I2C_Com( void ) {

	SensorData sensorData = {};
	char acce_coords[6];

	I2C_AcknowledgeConfig(I2C2, ENABLE);

	I2C_GenerateSTART(I2C2, ENABLE);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C2, 0x3A, I2C_Direction_Transmitter);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2C2, 0x01);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_GenerateSTART(I2C2, ENABLE);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C2, 0x3B, I2C_Direction_Receiver);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	for (int i=0;i<5;i++){
		while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
		acce_coords[i]=I2C_ReceiveData(I2C2);
	}

	I2C_AcknowledgeConfig(I2C2, DISABLE);
	I2C_GenerateSTOP(I2C2, ENABLE);
	while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
	acce_coords[5]= I2C_ReceiveData(I2C2);


	int16_t x_read=acce_coords[1]+(acce_coords[0]<<8);
	x_read=x_read>>4;
	int16_t y_read=acce_coords[3]+(acce_coords[2]<<8);
	y_read=y_read>>4;
	int16_t z_read=acce_coords[5]+(acce_coords[4]<<8);
	z_read=z_read>>4;

	x_read=-x_read;
	y_read=-y_read;
	z_read=-z_read;

	sensorData.x = x_read;
	sensorData.y = y_read;
	sensorData.z = z_read;

	return sensorData;
}
