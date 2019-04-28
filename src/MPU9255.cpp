/*
  MPU9255.cpp - General control functions.
*/

#include "MPU9255.h"
#include "Arduino.h"
#define SDA_PORT PORTA
#define SDA_PIN 7 // = A4
#define SCL_PORT PORTA
#define SCL_PIN 5 // = A5

#include <SoftWire.h>
SoftWire MyWire = SoftWire();
//initialise MPU9255
//Returns: 0 if success, 1 if imu or magnetometer fails
uint8_t MPU9255::init()
{
  //if ((MPU9255_SDA == -1)  or (MPU9255_SCL == -1))
    MyWire.begin();//enable I2C interface
  //else
//	  MyWire.begin(MPU9255_SDA,MPU9255_SCL);//enable I2C interface

  Hreset();//reset the chip
  write(MPU_address,CONFIG, 0x03); // set DLPF_CFG to 11
  write(MPU_address,SMPLRT_DIV, 0x00);// set prescaler sample rate to 0
  write(MPU_address,GYRO_CONFIG, 0x01);// set gyro to 3.6 KHz bandwidth, and 0.11 ms using FCHOICE bits
  write(MPU_address,INT_PIN_CFG, 0x02);// enable bypass
  write(MAG_address, CNTL, 0x16);//set magnetometer to read in mode 2 and enable 16 bit measurements

  //read magnetometer sensitivity
  mx_sensitivity = (((read(MAG_address, ASAX)-128)*0.5)/128)+1;
  my_sensitivity = (((read(MAG_address, ASAY)-128)*0.5)/128)+1;
  mz_sensitivity = (((read(MAG_address, ASAZ)-128)*0.5)/128)+1;

  //read factory gyroscope offset
  GX_offset = uint8ToUint16(read(MPU_address,XG_OFFSET_L), read(MPU_address,XG_OFFSET_H));
  GY_offset = uint8ToUint16(read(MPU_address,YG_OFFSET_L), read(MPU_address,YG_OFFSET_H));
  GZ_offset = uint8ToUint16(read(MPU_address,ZG_OFFSET_L), read(MPU_address,ZG_OFFSET_H));


  //Based on http://www.digikey.com/en/pdf/i/invensense/mpu-hardware-offset-registers .
  //read factory accelerometer offset

  //read the register values and save them as a 16 bit value
  AX_offset = (read(MPU_address,XA_OFFSET_H)<<8) | (read(MPU_address,XA_OFFSET_L));
  AY_offset = (read(MPU_address,YA_OFFSET_H)<<8) | (read(MPU_address,YA_OFFSET_L));
  AZ_offset = (read(MPU_address,ZA_OFFSET_H)<<8) | (read(MPU_address,ZA_OFFSET_L));
  //shift offset values to the right to remove the LSB
  AX_offset = AX_offset>>1;
  AY_offset = AY_offset>>1;
  AZ_offset = AZ_offset>>1;

  return (testIMU() || testMag());//return the output
}

//set gyroscope offset.
//Parameters :
// * axis selected_axis - selected axis
// * int16_t offset     - selected offset
void MPU9255::set_gyro_offset(axis selected_axis, int16_t offset)
{
  switch(selected_axis)
  {
    case X_axis:
      offset = offset + GX_offset;//add offset to the factory offset
      write(MPU_address,XG_OFFSET_L,(offset & 0xFF));//write low byte
      write(MPU_address,XG_OFFSET_H,(offset>>8));//write high byte
      break;

    case Y_axis:
      offset = offset + GY_offset;
      write(MPU_address,YG_OFFSET_L,(offset & 0xFF));
      write(MPU_address,YG_OFFSET_H,(offset>>8));
      break;

    case Z_axis:
      offset = offset + GZ_offset;
      write(MPU_address,ZG_OFFSET_L,(offset & 0xFF));
      write(MPU_address,ZG_OFFSET_H,(offset>>8));
      break;
  }
}

//set accelerometer offset
//Parameters :
// * axis selected_axis - selected axis
// * int16_t offset     - selected offset
void MPU9255::set_acc_offset(axis selected_axis, int16_t offset)
{

  switch(selected_axis)
  {
    case X_axis:
      offset = offset + AX_offset;//add offset to the factory offset
      write(MPU_address,XA_OFFSET_L,(offset & 0xFF)<<1);//write low byte
      write(MPU_address,XA_OFFSET_H,(offset>>7));//write high byte
      break;

    case Y_axis:
      offset = offset + AY_offset;
      write(MPU_address,YA_OFFSET_L,(offset & 0xFF)<<1);
      write(MPU_address,YA_OFFSET_H,(offset>>7));
      break;

    case Z_axis:
      offset = offset + AZ_offset;
      write(MPU_address,ZA_OFFSET_L,(offset & 0xFF)<<1);
      write(MPU_address,ZA_OFFSET_H,(offset>>7));
      break;
  }
}

//set accelerometer bandwidth
//Parameters :
// * bandwidth selected_bandwidth - selected bandwidth
void MPU9255::set_acc_bandwidth(bandwidth selected_bandwidth)
{
  switch(selected_bandwidth)
  {
    case acc_1113Hz:
      write_OR(MPU_address,ACCEL_CONFIG_2,(1<<3));//set accel_fchoice_b to 1
      break;

    case acc_460Hz:
      //set accel_fchoice_b to 0 and  A_DLPF_CFG to 0(000)
      write_AND(MPU_address,ACCEL_CONFIG_2,~((1<<3)|(1<<2)|(1<<1)|(1<<0)));
      break;

    case acc_184Hz:
      write_AND(MPU_address,ACCEL_CONFIG_2,~(1<<3));//set accel_fchoice_b to 0
      //set A_DLPF_CFG to 1(001)
      write_AND(MPU_address,ACCEL_CONFIG_2,~((1<<1)|(1<<2)));
      write_OR(MPU_address,ACCEL_CONFIG_2,(1<<0));
      break;

    case acc_92Hz:
      write_AND(MPU_address,ACCEL_CONFIG_2,~(1<<3));//set accel_fchoice_b to 0
      //set A_DLPF_CFG to 2(010)
      write_AND(MPU_address,ACCEL_CONFIG_2,~((1<<0)|(1<<2)));
      write_OR(MPU_address,ACCEL_CONFIG_2,(1<<1));
      break;

    case acc_41Hz:
      write_AND(MPU_address,ACCEL_CONFIG_2,~(1<<3));//set accel_fchoice_b to 0
      //set A_DLPF_CFG to 3(011)
      write_AND(MPU_address,ACCEL_CONFIG_2,~(1<<2));
      write_OR(MPU_address,ACCEL_CONFIG_2,(1<<0)|(1<<1));
      break;

    case acc_20Hz:
      write_AND(MPU_address,ACCEL_CONFIG_2,~(1<<3));//set accel_fchoice_b to 0
      //set A_DLPF_CFG to 4(100)
      write_AND(MPU_address,ACCEL_CONFIG_2,~((1<<0)|(1<<1)));
      write_OR(MPU_address,ACCEL_CONFIG_2,(1<<2));
      break;

    case acc_10Hz:
      write_AND(MPU_address,ACCEL_CONFIG_2,~(1<<3));//set accel_fchoice_b to 0
      //set A_DLPF_CFG to 5(101)
      write_AND(MPU_address,ACCEL_CONFIG_2,~(1<<1));
      write_OR(MPU_address,ACCEL_CONFIG_2,(1<<0)|(1<<2));
      break;

    case acc_5Hz:
      write_AND(MPU_address,ACCEL_CONFIG_2,~(1<<3));//set accel_fchoice_b to 0
      //set A_DLPF_CFG to 6(110)
      write_AND(MPU_address,ACCEL_CONFIG_2,~(1<<0));
      write_OR(MPU_address,ACCEL_CONFIG_2,(1<<1)|(1<<2));
      break;
  }
}

//set gyroscope bandwidth
//Parameters :
// * bandwidth selected_bandwidth - selected bandwidth
void MPU9255::set_gyro_bandwidth(bandwidth selected_bandwidth)
{
  switch(selected_bandwidth)
  {
    case gyro_8800Hz:
      write_OR(MPU_address,GYRO_CONFIG,(1<<0));//set Fchoice_b <0> to 1
      break;

    case gyro_3600Hz:
      write_AND(MPU_address,GYRO_CONFIG,~(1<<0));//set Fchoice_b <0> to 0
      write_OR(MPU_address,GYRO_CONFIG,(1<<1));//set Fchoice_b <1> to 1
      break;

    case gyro_250Hz:
      write_AND(MPU_address,GYRO_CONFIG,~((1<<0)|(1<<1)));//set both Fchoice_b to 0
      //write 0(000) to DLPF_CFG
      write_AND(MPU_address,CONFIG,~((1<<0)|(1<<1)|(1<<2)));
      break;

    case gyro_184Hz:
      write_AND(MPU_address,GYRO_CONFIG,~((1<<0)|(1<<1)));//set both Fchoice_b to 0
      //write 1(001) to DLPF_CFG
      write_AND(MPU_address,CONFIG,~((1<<1)|(1<<2)));
      write_OR(MPU_address,CONFIG,(1<<0));
      break;

    case gyro_92Hz:
      write_AND(MPU_address,GYRO_CONFIG,~((1<<0)|(1<<1)));//set both Fchoice_b to 0
      //write 2(010) to DLPF_CFG
      write_AND(MPU_address,CONFIG,~((1<<2)|(1<<0)));
      write_OR(MPU_address,CONFIG,(1<<1));
      break;

    case gyro_41Hz:
      write_AND(MPU_address,GYRO_CONFIG,~((1<<0)|(1<<1)));//set both Fchoice_b to 0
      //write 3(011) to DLPF_CFG
      write_AND(MPU_address,CONFIG,~(1<<2));
      write_OR(MPU_address,CONFIG,(1<<0)|(1<<1));
      break;

    case gyro_20Hz:
      write_AND(MPU_address,GYRO_CONFIG,~((1<<0)|(1<<1)));//set both Fchoice_b to 0
      //write 4(100) to DLPF_CFG
      write_AND(MPU_address,CONFIG,~((1<<1)|(1<<0)));
      write_OR(MPU_address,CONFIG,(1<<2));
      break;

    case gyro_10Hz:
      write_AND(MPU_address,GYRO_CONFIG,~((1<<0)|(1<<1)));//set both Fchoice_b to 0
      //write 5(101) to DLPF_CFG
      write_AND(MPU_address,CONFIG,~(1<<1));
      write_OR(MPU_address,CONFIG,(1<<2)|(1<<0));
      break;

    case gyro_5Hz:
      write_AND(MPU_address,GYRO_CONFIG,~((1<<0)|(1<<1)));//set both Fchoice_b to 0
      //write 6(110) to DLPF_CFG
      write_AND(MPU_address,CONFIG,~(1<<0));
      write_OR(MPU_address,CONFIG,(1<<1)|(1<<2));
      break;
  }
}

//convert selected scale to to register value
//Parameters :
// * uint8_t current_state - previous register state
// * scales selected_scale - selected scale
//Returns : New register value (uint8_t)
uint8_t MPU9255::getScale(uint8_t current_state, scales selected_scale)
{
  if(selected_scale == scale_2g || selected_scale == scale_250dps)
  {
    current_state &= ~((1<<3)|(1<<4));
  }

  if(selected_scale == scale_4g || selected_scale == scale_500dps)
  {
    current_state &= ~(1<<4);
    current_state |= (1<<3);
  }

  if(selected_scale == scale_8g || selected_scale == scale_1000dps)
  {
    current_state &= ~(1<<3);
    current_state |= (1<<4);
  }

  if(selected_scale == scale_16g || selected_scale == scale_2000dps)
  {
    current_state |= (1<<4)|(1<<3);
  }

  return current_state;
}

//Set accelerometer scale
//Parameters:
// *scales selected_scale - Selected scale
void MPU9255::set_acc_scale(scales selected_scale)
{
  uint8_t val = read(MPU_address,ACCEL_CONFIG);//read old register value
  val = getScale(val,selected_scale);//get new register value
  write(MPU_address,ACCEL_CONFIG,val);//commit changes
}

//Set gyroscope scale
//Parameters:
// * scales selected_scale - Selected scale
void MPU9255::set_gyro_scale(scales selected_scale)
{
  uint8_t val=read(MPU_address,GYRO_CONFIG);
  val = getScale(val,selected_scale);
  write(MPU_address,GYRO_CONFIG,val);
}

//test IMU (gyroscope and accelerometer)
//Returns 0 if success, 1 if failure
uint8_t MPU9255::testIMU()
{
  if(read(MPU_address,WHO_AM_I)==0xFF)
  {
    return 1;
  }
  return 0;
}

//test magnetometer
//Returns 0 if success, 1 if failure
uint8_t MPU9255::testMag()
{
  if(read(MAG_address,MAG_ID)==0xFF)
  {
    return 1;
  }
  return 0;
}
/*
MPU9255_Communication.cpp - MPU9255 <-> CPU communication functions
*/

#include "MPU9255.h"
#include "Arduino.h"

//Request data
//Parameters:
// * uint8_t address    - adress of the device
// * uint8_t subAddress - address of the register (the read starts from that register)
// * uint8_t bytes      - number of requested bytes
void MPU9255::requestBytes(uint8_t address, uint8_t subAddress, uint8_t bytes)
{
  MyWire.beginTransmission(address);
  MyWire.write(subAddress);
  MyWire.endTransmission(false);
  MyWire.requestFrom(address, bytes);
}

//read an array of bytes from the device
//Parameters:
// * uint8_t *output - pointer to the output array
// * char size       - number of bytes to read
void MPU9255::readArray(uint8_t *output, char size)
{
  for(char i = 0; i<size; i++)
  {
    output[i] = MyWire.read();//read byte and put it into rawData table
  }
}

//Read one byte of data
//Parameters:
// * uint8_t address    - device address
// * uint8_t subAddress - register address
//Returns : Readed byte of data (uint8_t)
uint8_t MPU9255::read(uint8_t address, uint8_t subAddress)
{
  requestBytes(address,subAddress,1);//request one byte from the register
  uint8_t data = MyWire.read();//read one byte of data
  return data;
}

// Write one byte of data
//Parameters:
// * uint8_t address    - device address
// * uint8_t subAddress - register address
// * uint8_t data       - one byte of data that we want to put in the register
void MPU9255::write(uint8_t address, uint8_t subAddress, uint8_t data)
{
  MyWire.beginTransmission(address);
  MyWire.write(subAddress);
  MyWire.write(data);
  MyWire.endTransmission();
}

//Change state of the register using OR operation
//Parameters:
// * uint8_t address    - device address
// * uint8_t subAddress - register address
// * uint8_t data       - one byte of data that we want to put in the register
void MPU9255::write_OR(uint8_t address, uint8_t subAddress, uint8_t data)
{
  uint8_t c = read(address,subAddress);
  c = c | data;
  write(address,subAddress,c);
}

//Change state of the register using AND operation
//Parameters:
// * uint8_t address    - device address
// * uint8_t subAddress - register address
// * uint8_t data       - one byte of data that we want to put in the register
void MPU9255::write_AND(uint8_t address, uint8_t subAddress, uint8_t data)
{
  uint8_t c = read(address,subAddress);
  c = c & data;
  write(address,subAddress,c);
}
