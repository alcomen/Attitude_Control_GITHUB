//Arduino 1.0+ only

#include <Wire.h>

#define CTRL_REG1 0x20
#define CTRL_REG2 0x21
#define CTRL_REG3 0x22
#define CTRL_REG4 0x23
#define CTRL_REG5 0x24

int L3G4200D_Address = 105; //I2C address of the L3G4200D

float constante = 0;

int x;
int y;
int z;


void getGyroValues(){

  byte xMSB = readRegister(L3G4200D_Address, 0x29);
  byte xLSB = readRegister(L3G4200D_Address, 0x28);
  x = ((xMSB << 8) | xLSB);

  byte yMSB = readRegister(L3G4200D_Address, 0x2B);
  byte yLSB = readRegister(L3G4200D_Address, 0x2A);
  y = ((yMSB << 8) | yLSB);

  byte zMSB = readRegister(L3G4200D_Address, 0x2D);
  byte zLSB = readRegister(L3G4200D_Address, 0x2C);
  z = ((zMSB << 8) | zLSB); 
  
        constante = 0.06104; //(range de leitura / resoluço => 2000 / 32768)
          
        //get the raw data
        sen_data.gyro_x_raw = x;///Gyro_Gain_X_L3G4200D;    // X axis 
        sen_data.gyro_y_raw = y;///Gyro_Gain_X_L3G4200D;    // Y axis 
        sen_data.gyro_z_raw = z;///Gyro_Gain_X_L3G4200D;    // Z axis
        
        //subtract the offset
        sen_data.gyro_x = sen_data.gyro_x_raw * constante;    // X axis 
        sen_data.gyro_y = sen_data.gyro_y_raw * constante;    // Y axis 
        sen_data.gyro_z = sen_data.gyro_z_raw * constante;    // Z axis
        
        //change the sign if needed
        sen_data.gyro_x *= SENSOR_SIGN[0];    // X axis 
        sen_data.gyro_y *= SENSOR_SIGN[1];    // Y axis 
        sen_data.gyro_z *= SENSOR_SIGN[2];    // Z axis
        
        // load gyro values to DCM
        gyro[0] = sen_data.gyro_x;
        gyro[1] = sen_data.gyro_y;
        gyro[2] = sen_data.gyro_z;
        
        //change to variable to DCM calc
        //gyro[0] = sen_data.gyro_x;
        //gyro[1] = sen_data.gyro_y;
        //gyro[2] = sen_data.gyro_z;
        
        // load to Ethernet and convert to rad
        sen_data_local.gyro_x = ToRad(sen_data.gyro_x);
        sen_data_local.gyro_y = ToRad(sen_data.gyro_y);
        sen_data_local.gyro_z = ToRad(sen_data.gyro_z);
        
        //Serial.println(sen_data_local.gyro_y);
        //Serial.println(ToRad(sen_data_local.gyro_y));
        // status OK
        sen_data_local.gyro_x_s = 1;
        sen_data_local.gyro_y_s = 1;
        sen_data_local.gyro_z_s = 1;
        //sen_data.gyro_x_s = 1;
        //sen_data.gyro_y_s = 1;
        //sen_data.gyro_z_s = 1;
}

int setupL3G4200D(int scale){
  //From  Jim Lindblom of Sparkfun's code

  // Enable x, y, z and turn off power down:
  writeRegister(L3G4200D_Address, CTRL_REG1, 0b00001111);

  // If you'd like to adjust/use the HPF, you can edit the line below to configure CTRL_REG2:
  writeRegister(L3G4200D_Address, CTRL_REG2, 0b00000000);

  // Configure CTRL_REG3 to generate data ready interrupt on INT2
  // No interrupts used on INT1, if you'd like to configure INT1
  // or INT2 otherwise, consult the datasheet:
  writeRegister(L3G4200D_Address, CTRL_REG3, 0b00001000);

  // CTRL_REG4 controls the full-scale range, among other things:

  if(scale == 250){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00000000);
  }else if(scale == 500){
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00010000);
  }else{
    writeRegister(L3G4200D_Address, CTRL_REG4, 0b00110000);
  }

  // CTRL_REG5 controls high-pass filtering of outputs, use it
  // if you'd like:
  writeRegister(L3G4200D_Address, CTRL_REG5, 0b00000000);
}

void writeRegister(int deviceAddress, byte address, byte val) {
    Wire.beginTransmission(deviceAddress); // start transmission to device 
    Wire.write(address);       // send register address
    Wire.write(val);         // send value to write
    Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

    int v;
    Wire.beginTransmission(deviceAddress);
    Wire.write(address); // register to read
    Wire.endTransmission();

    Wire.requestFrom(deviceAddress, 1); // read a byte

    while(!Wire.available()) {
        // waiting
    }

    v = Wire.read();
    return v;
}
