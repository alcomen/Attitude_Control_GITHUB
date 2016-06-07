/***************************************************************************************************************
* Razor AHRS Firmware v1.4.2
* 9 Degree of Measurement Attitude and Heading Reference System
* for Sparkfun "9DOF Razor IMU" (SEN-10125 and SEN-10736)
* and "9DOF Sensor Stick" (SEN-10183, 10321 and SEN-10724)
*
* Released under GNU GPL (General Public License) v3.0
* Copyright (C) 2013 Peter Bartz [http://ptrbrtz.net]
* Copyright (C) 2011-2012 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
*
* Infos, updates, bug reports, contributions and feedback:
*     https://github.com/ptrbrtz/razor-9dof-ahrs
*
*
* History:
*   * Original code (http://code.google.com/p/sf9domahrs/) by Doug Weibel and Jose Julio,
*     based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel. Thank you!
*
*   * Updated code (http://groups.google.com/group/sf_9dof_ahrs_update) by David Malik (david.zsolt.malik@gmail.com)
*     for new Sparkfun 9DOF Razor hardware (SEN-10125).
*
*   * Updated and extended by Peter Bartz (peter-bartz@gmx.de):
*     * v1.3.0
*       * Cleaned up, streamlined and restructured most of the code to make it more comprehensible.
*       * Added sensor calibration (improves precision and responsiveness a lot!).
*       * Added binary yaw/pitch/roll output.
*       * Added basic serial command interface to set output modes/calibrate sensors/synch stream/etc.
*       * Added support to synch automatically when using Rovering Networks Bluetooth modules (and compatible).
*       * Wrote new easier to use test program (using Processing).
*       * Added support for new version of "9DOF Razor IMU": SEN-10736.
*       --> The output of this code is not compatible with the older versions!
*       --> A Processing sketch to test the tracker is available.
*     * v1.3.1
*       * Initializing rotation matrix based on start-up sensor readings -> orientation OK right away.
*       * Adjusted gyro low-pass filter and output rate settings.
*     * v1.3.2
*       * Adapted code to work with new Arduino 1.0 (and older versions still).
*     * v1.3.3
*       * Improved synching.
*     * v1.4.0
*       * Added support for SparkFun "9DOF Sensor Stick" (versions SEN-10183, SEN-10321 and SEN-10724).
*     * v1.4.1
*       * Added output modes to read raw and/or calibrated sensor data in text or binary format.
*       * Added static magnetometer soft iron distortion compensation
*     * v1.4.2
*       * (No core firmware changes)
*
* TODOs:
*   * Allow optional use of EEPROM for storing and reading calibration values.
*   * Use self-test and temperature-compensation features of the sensors.
***************************************************************************************************************/

/*
  "9DOF Razor IMU" hardware versions: SEN-10125 and SEN-10736
  ATMega328@3.3V, 8MHz
  ADXL345  : Accelerometer
  HMC5843  : Magnetometer on SEN-10125
  HMC5883L : Magnetometer on SEN-10736
  ITG-3200 : Gyro
  Arduino IDE : Select board "Arduino Pro or Pro Mini (3.3v, 8Mhz) w/ATmega328"
*/

/*
  "9DOF Sensor Stick" hardware versions: SEN-10183, SEN-10321 and SEN-10724
  ADXL345  : Accelerometer
  HMC5843  : Magnetometer on SEN-10183 and SEN-10321
  HMC5883L : Magnetometer on SEN-10724
  ITG-3200 : Gyro
*/

/*
  Axis definition (differs from definition printed on the board!):
    X axis pointing forward (towards the short edge with the connector holes)
    Y axis pointing to the right
    and Z axis pointing down.
    
  Positive yaw   : clockwise
  Positive roll  : right wing down
  Positive pitch : nose up
  
  Transformation order: first yaw then pitch then roll.
*/

/*
  Serial commands that the firmware understands:
  
  "#o<params>" - Set OUTPUT mode and parameters. The available options are:
  
      // Streaming output
      "#o0" - DISABLE continuous streaming output. Also see #f below.
      "#o1" - ENABLE continuous streaming output.
      
      // Angles output
      "#ob" - Output angles in BINARY format (yaw/pitch/roll as binary float, so one output frame
              is 3x4 = 12 bytes long).
      "#ot" - Output angles in TEXT format (Output frames have form like "#YPR=-142.28,-5.38,33.52",
              followed by carriage return and line feed [\r\n]).
      
      // Sensor calibration
      "#oc" - Go to CALIBRATION output mode.
      "#on" - When in calibration mode, go on to calibrate NEXT sensor.
      
      // Sensor data output
      "#osct" - Output CALIBRATED SENSOR data of all 9 axes in TEXT format.
                One frame consist of three lines - one for each sensor: acc, mag, gyr.
      "#osrt" - Output RAW SENSOR data of all 9 axes in TEXT format.
                One frame consist of three lines - one for each sensor: acc, mag, gyr.
      "#osbt" - Output BOTH raw and calibrated SENSOR data of all 9 axes in TEXT format.
                One frame consist of six lines - like #osrt and #osct combined (first RAW, then CALIBRATED).
                NOTE: This is a lot of number-to-text conversion work for the little 8MHz chip on the Razor boards.
                In fact it's too much and an output frame rate of 50Hz can not be maintained. #osbb.
      "#oscb" - Output CALIBRATED SENSOR data of all 9 axes in BINARY format.
                One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
      "#osrb" - Output RAW SENSOR data of all 9 axes in BINARY format.
                One frame consist of three 3x3 float values = 36 bytes. Order is: acc x/y/z, mag x/y/z, gyr x/y/z.
      "#osbb" - Output BOTH raw and calibrated SENSOR data of all 9 axes in BINARY format.
                One frame consist of 2x36 = 72 bytes - like #osrb and #oscb combined (first RAW, then CALIBRATED).
      
      // Error message output        
      "#oe0" - Disable ERROR message output.
      "#oe1" - Enable ERROR message output.
    
    
  "#f" - Request one output frame - useful when continuous output is disabled and updates are
         required in larger intervals only. Though #f only requests one reply, replies are still
         bound to the internal 20ms (50Hz) time raster. So worst case delay that #f can add is 19.99ms.
         
         
  "#s<xy>" - Request synch token - useful to find out where the frame boundaries are in a continuous
         binary stream or to see if tracker is present and answering. The tracker will send
         "#SYNCH<xy>\r\n" in response (so it's possible to read using a readLine() function).
         x and y are two mandatory but arbitrary bytes that can be used to find out which request
         the answer belongs to.
          
          
  ("#C" and "#D" - Reserved for communication with optional Bluetooth module.)
  
  Newline characters are not required. So you could send "#ob#o1#s", which
  would set binary output mode, enable continuous streaming output and request
  a synch token all at once.
  
  The status LED will be on if streaming output is enabled and off otherwise.
  
  Byte order of binary output is little-endian: least significant byte comes first.
*/



/*****************************************************************/
/*********** USER SETUP AREA! Set your options here! *************/
/*****************************************************************/

// HARDWARE OPTIONS
/*****************************************************************/
// Select your hardware here by uncommenting one line!
//#define HW__VERSION_CODE 10125 // SparkFun "9DOF Razor IMU" version "SEN-10125" (HMC5843 magnetometer)
#define HW__VERSION_CODE 10736 // SparkFun "9DOF Razor IMU" version "SEN-10736" (HMC5883L magnetometer)
//#define HW__VERSION_CODE 10183 // SparkFun "9DOF Sensor Stick" version "SEN-10183" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10321 // SparkFun "9DOF Sensor Stick" version "SEN-10321" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10724 // SparkFun "9DOF Sensor Stick" version "SEN-10724" (HMC5883L magnetometer)


// OUTPUT OPTIONS
/*****************************************************************/
// Set your serial port baud rate used to send out data here!
#define OUTPUT__BAUD_RATE 115200

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT__DATA_INTERVAL 20  // in milliseconds

// Output mode definitions (do not change)
#define OUTPUT__MODE_CALIBRATE_SENSORS 0 // Outputs sensor min/max values as text for manual calibration
#define OUTPUT__MODE_ANGLES 1 // Outputs yaw/pitch/roll in degrees
#define OUTPUT__MODE_SENSORS_CALIB 2 // Outputs calibrated sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_RAW 3 // Outputs raw (uncalibrated) sensor values for all 9 axes
#define OUTPUT__MODE_SENSORS_BOTH 4 // Outputs calibrated AND raw sensor values for all 9 axes
// Output format definitions (do not change)
#define OUTPUT__FORMAT_TEXT 0 // Outputs data as text
#define OUTPUT__FORMAT_BINARY 1 // Outputs data as binary float

// Select your startup output mode and format here!
int output_mode = OUTPUT__MODE_ANGLES;
int output_format = OUTPUT__FORMAT_TEXT;

// Select if serial continuous streaming output is enabled per default on startup.
#define OUTPUT__STARTUP_STREAM_ON true  // true or false

// If set true, an error message will be output if we fail to read sensor data.
// Message format: "!ERR: reading <sensor>", followed by "\r\n".
boolean output_errors = true;  // true or false

// Bluetooth
// You can set this to true, if you have a Rovering Networks Bluetooth Module attached.
// The connect/disconnect message prefix of the module has to be set to "#".
// (Refer to manual, it can be set like this: SO,#)
// When using this, streaming output will only be enabled as long as we're connected. That way
// receiver and sender are synchronzed easily just by connecting/disconnecting.
// It is not necessary to set this! It just makes life easier when writing code for
// the receiving side. The Processing test sketch also works without setting this.
// NOTE: When using this, OUTPUT__STARTUP_STREAM_ON has no effect!
#define OUTPUT__HAS_RN_BLUETOOTH false  // true or false


// SENSOR CALIBRATION
/*****************************************************************/
// How to calibrate? Read the tutorial at http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
// Put MIN/MAX and OFFSET readings for your board here!
// Accelerometer
// "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define ACCEL_X_MIN ((float) -250)
#define ACCEL_X_MAX ((float) 250)
#define ACCEL_Y_MIN ((float) -250)
#define ACCEL_Y_MAX ((float) 250)
#define ACCEL_Z_MIN ((float) -250)
#define ACCEL_Z_MAX ((float) 250)

// Magnetometer (standard calibration mode)
// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define MAGN_X_MIN ((float) -600)
#define MAGN_X_MAX ((float) 600)
#define MAGN_Y_MIN ((float) -600)
#define MAGN_Y_MAX ((float) 600)
#define MAGN_Z_MIN ((float) -600)
#define MAGN_Z_MAX ((float) 600)

// Magnetometer (extended calibration mode)
// Uncommend to use extended magnetometer calibration (compensates hard & soft iron errors)
//#define CALIBRATION__MAGN_USE_EXTENDED true
//const float magn_ellipsoid_center[3] = {0, 0, 0};
//const float magn_ellipsoid_transform[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Gyroscope
// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
#define GYRO_AVERAGE_OFFSET_X ((float) 0.0)
#define GYRO_AVERAGE_OFFSET_Y ((float) 0.0)
#define GYRO_AVERAGE_OFFSET_Z ((float) 0.0)

/*
// Calibration example:
// "accel x,y,z (min/max) = -277.00/264.00  -256.00/278.00  -299.00/235.00"
#define ACCEL_X_MIN ((float) -277)
#define ACCEL_X_MAX ((float) 264)
#define ACCEL_Y_MIN ((float) -256)
#define ACCEL_Y_MAX ((float) 278)
#define ACCEL_Z_MIN ((float) -299)
#define ACCEL_Z_MAX ((float) 235)
// "magn x,y,z (min/max) = -511.00/581.00  -516.00/568.00  -489.00/486.00"
//#define MAGN_X_MIN ((float) -511)
//#define MAGN_X_MAX ((float) 581)
//#define MAGN_Y_MIN ((float) -516)
//#define MAGN_Y_MAX ((float) 568)
//#define MAGN_Z_MIN ((float) -489)
//#define MAGN_Z_MAX ((float) 486)
// Extended magn
#define CALIBRATION__MAGN_USE_EXTENDED true
const float magn_ellipsoid_center[3] = {91.5, -13.5, -48.1};
const float magn_ellipsoid_transform[3][3] = {{0.902, -0.00354, 0.000636}, {-0.00354, 0.9, -0.00599}, {0.000636, -0.00599, 1}};
// Extended magn (with Sennheiser HD 485 headphones)
//#define CALIBRATION__MAGN_USE_EXTENDED true
//const float magn_ellipsoid_center[3] = {72.3360, 23.0954, 53.6261};
//const float magn_ellipsoid_transform[3][3] = {{0.879685, 0.000540833, -0.0106054}, {0.000540833, 0.891086, -0.0130338}, {-0.0106054, -0.0130338, 0.997494}};
//"gyro x,y,z (current/average) = -40.00/-42.05  98.00/96.20  -18.00/-18.36"
#define GYRO_AVERAGE_OFFSET_X ((float) -42.05)
#define GYRO_AVERAGE_OFFSET_Y ((float) 96.20)
#define GYRO_AVERAGE_OFFSET_Z ((float) -18.36)
*/


// DEBUG OPTIONS
/*****************************************************************/
// When set to true, gyro drift correction will not be applied
#define DEBUG__NO_DRIFT_CORRECTION false
// Print elapsed time after each I/O loop
#define DEBUG__PRINT_LOOP_TIME false


/*****************************************************************/
/****************** END OF USER SETUP AREA!  *********************/
/*****************************************************************/










// Check if hardware version code is defined
#ifndef HW__VERSION_CODE
  // Generate compile error
  #error YOU HAVE TO SELECT THE HARDWARE YOU ARE USING! See "HARDWARE OPTIONS" in "USER SETUP AREA" at top of Razor_AHRS.ino!
#endif

#include <Wire.h>
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <SPI.h>
#include <SD.h>

#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi

//========================================
// Output Data Configuration
// Turn on/off the different data outputs
// For the Mongoose Visualization Software to work, the first 3 need to be turned on
// To achieve the max sample rate, you will need to only turn on PRINT_EULER

#define PRINT_ETHERNET          0   //Will print the Ethernet Packarges receive
#define PRINT_EULER             1   //Will print the Euler angles Roll, Pitch and Yaw
#define PRINT_SENSOR_DATA       1   //Will print the Corrected Sensor Data
#define PRINT_SENSOR_DATA_RAW   0   //Will print the raw uncorrected Sensor Data
#define PRINT_DCM               0   //Will print the whole direction cosine matrix
#define PRINT_MagCal            0
#define PRINT_MOTORS            0   //Will print the speed of motors
#define SD_CARD                 0   //If using SD CARD SD_CARD = 1, else SD_CARD = 0
#define INTERACTION_NUMBER      5   // Interaction to execute print angles output

/**
// Rede
/*
  GROUND STATION
  IP: 172.16.0.01
  PORT: 
  SYS_STATE: 35105
  PLA_RESP: 35106
  TRANS_RESP: 35107
  
  IMA
  IP: 172.16.0.4
  PORT: 35204
  
  ESFERA
  IP: 172.160.0.62
  PORT: 35308
*/
String test = "";
// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network:
byte mac[] = {  
  0x40, 0xD8, 0x55, 0x10, 0x40, 0x5C };
  
// LOCAL
IPAddress ip(172, 16, 0, 62);//(192, 168, 0, 62);
unsigned int localPort = 35308;     

// REMOTO
IPAddress ipRemote(172, 16, 0, 4);//(192, 168, 0, 100);//52); ou final 55
unsigned int portRemote = 35204;

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet,
char ReplyBuffer[] = "acknowledged";       // a string to send back

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP Udp;

// Flag utilizado para chaveaer entre controle interno ou externo
boolean override_control_flag = 0;

// SD CARD
// On the Ethernet Shield, CS is pin 4. Note that even if it's not
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.
const int chipSelect = 4;

File dataFile;

// IGT-3200 Sensitivity (from datasheet) => 14.375 LSBs/°/s
// Tested values : 
#define Gyro_Gain_X   14.375 //X axis Gyro gain
#define Gyro_Gain_Y   14.375 //Y axis Gyro gain
#define Gyro_Gain_Z   14.375 //Z axis Gyro gain

#define Gyro_Gain_X_L3G4200D  2000
#define Gyro_Gain_Y_L3G4200D  2000
#define Gyro_Gain_Z_L3G4200D  2000

int SENSOR_SIGN[9] = { 1,1,-1,1,1,-1,1,1,-1};  //Correct directions x,y,z - gyros, accels, magnetormeter


// Motor Var
int16_t motor_speed_default = 5000;// 5000;

int16_t motor_speed_limit = 10000;

char motor_sequence_request = 0;
  
uint16_t interaction = 0;          // Main Loop count 
  
char Motors_Vel_Index = 0;
uint8_t print_var = 0;
uint8_t *rx_ptr;
uint8_t rx_buff[10];
uint16_t motor_req_x = 0;
uint16_t motor_req_y = 0;
uint16_t motor_req_z = 0;
String inputString = "";         // a string to hold incoming data

// IGT-3200 Sensitivity (from datasheet) => 14.375 LSBs/�/s
// Tested values : 
#define Gyro_Gain_X   14.375 //X axis Gyro gain
#define Gyro_Gain_Y   14.375 //Y axis Gyro gain
#define Gyro_Gain_Z   14.375 //Z axis Gyro gain

/** \struct s_sensor_offsets
 * \brief Structure for holding offsets and calibration values for the accel, gyro, and magnetom.
 */
struct s_sensor_offsets
{
    float gyro_offset[3];       /**< Gyrometer offset. */
    float accel_offset[3];      /**< Accelerometer offset. */
    float magnetom_offset[3];   /**< Magnetometer offset. */
    float magnetom_XY_Theta;    /**< magnetom_XY_Theta. */
    float magnetom_XY_Scale;    /**< magnetom_XY_Scale. */
    float magnetom_YZ_Theta;    /**< magnetom_YZ_Theta. */
    float magnetom_YZ_Scale;    /**< magnetom_YZ_Scale. */
};


/** \struct s_sensor_data
 * \brief Structure for holding raw and calibration corrected data from the sensors.
 * Raw data is uncorrected and corresponds to the
 * true sensor axis, not the redefined platform orientation
 * This data has been corrected based on the calibration values.
 */
struct s_ima_data
{
    
    //char msg_hdr;           /**< Message Header. */
    char msg_id;            /**< Message ID. */
    float accel_x;          /**< Accelerometer X data. */
    uint32_t accel_x_s;     /**< Accelerometer X status. */
    float accel_y;          /**< Accelerometer Y data. */
    uint32_t accel_y_s;     /**< Accelerometer Y status. */
    float accel_z;          /**< Accelerometer Z data. */
    uint32_t accel_z_s;     /**< Accelerometer Z status. */
    float magnetom_x;       /**< Magnetometer X data. */
    uint32_t magnetom_x_s;  /**< Magnetometer X status. */
    float magnetom_y;       /**< Magnetometer Y data */
    uint32_t magnetom_y_s;  /**< Magnetometer Y status. */
    float magnetom_z;       /**< Magnetometer Z data */
    uint32_t magnetom_z_s;  /**< Magnetometer Z status. */
    float gyro_x;           /**< Gyro X data. */
    uint32_t gyro_x_s;      /**< Gyro X status. */
    float gyro_y;           /**< Gyro Y data. */
    uint32_t gyro_y_s;      /**< Gyro Y status. */
    float gyro_z;           /**< Gyro Z data. */
    uint32_t gyro_z_s;      /**< Gyro Z status. */
    float roll;             /**< Roll angle. */
    float pitch;            /**< Pitch angle. */
    float yaw;              /**< Yaw angle. */
    
    float quat[4];          /**< Quaternions. */
    
    short motor_x_sp;       /**< Motor X actual speed. */
    uint32_t motor_x_st;    /**< Motor X status. */
    short motor_y_sp;       /**< Motor Y actual speed. */
    uint32_t motor_y_st;    /**< Motor Y status. */
    short motor_z_sp;       /**< Motor Z actual speed. */
    uint32_t motor_z_st;    /**< Motor Z status. */
    char  optional;         /**< Optional byte. */ 
 //-----------------------------------------------------------------------
 
    int gyro_x_raw;         /**< Gyrometer X raw data. */
    int gyro_y_raw;         /**< Gyrometer Y raw data. */
    int gyro_z_raw;         /**< Gyrometer Z raw data. */
    int accel_x_raw;        /**< Accelerometer X raw data. */
    int accel_y_raw;        /**< Accelerometer Y raw data. */
    int accel_z_raw;        /**< Accelerometer Z raw data. */
    int magnetom_x_raw;     /**< Magnetometer X raw data. */
    int magnetom_y_raw;     /**< Magnetometer Y raw data. */
    int magnetom_z_raw;     /**< Magnetometer Z raw data. */
    float magnetom_heading; /**< Magnetometer heading. */
    short baro_temp;        /**< Barometer temperature data. */
    long baro_pres;         /**< Barometer pressure data. */
};
/** \struct s_incoming
 * \brief Structure to store data received from IMA.
 */
struct s_motor
{
    char msg_id;        /**< Message ID. */
    char act_en;        /**< Actuators enable. */
    short motor_x_sp;   /**< Motor X speed desired. */
    char  motor_x_w;    /**< Motor X rotation direction. */
    short motor_y_sp;   /**< Motor Y speed desired. */
    char  motor_y_w;    /**< Motor Y rotation direction. */
    short motor_z_sp;   /**< Motor Z speed desired. */
    char  motor_z_w;    /**< Motor Z rotation direction. */
    char optional;      /**< Optional byte. */
    char dummy;         /**< Byte used to store the last byte received which is dummy. */
    byte flag;
};

//strucsen_offsetr holding raw and calibration corrected data from the sensors
struct r_data
{
    //r is uncorrected and corresponds to the
    //true sensor axis, not the redefined platform orientation
    int gyro_x_raw;
    int gyro_y_raw;
    int gyro_z_raw;
    int accel_x_raw;
    int accel_y_raw;
    int accel_z_raw;
    int magnetom_x_raw;
    int magnetom_y_raw;
    int magnetom_z_raw;
  
    //This data has been corrected based on the calibration values
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float accel_x;
    float accel_y;
    float accel_z;
    float magnetom_x;
    float magnetom_y;
    float magnetom_z;
    float magnetom_heading;
    short baro_temp;  
    long baro_pres;
};

  //s_sensor_offsets sen_offset = {0,0,0,0,0,0,0,0,0};
  s_ima_data sen_data = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

  s_ima_data              sen_data_to_ima;                  /**< Estrutura em BigEndian pronta para ser enviada para o IMA. */
  s_ima_data              sen_data_local;                   /**< Estrutura em LittleEndian. */
  s_motor                 mot_data_received;                /**< Estrutura recebida através da Ethernet e convertida em LittleEndian. */

// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))


// Gain for gyroscope (ITG-3200)
#define GYRO_GAIN 0.06957 // Same gain on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second

// DCM parameters
#define Kp_ROLLPITCH 0.02f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f

// Stuff
#define STATUS_LED_PIN 13  // Pin number of status LED
#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

// Sensor variables
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float accel_min[3];
float accel_max[3];

float magnetom[3];
float magnetom_min[3];
float magnetom_max[3];
float magnetom_tmp[3];

float gyro[3];
float gyro_average[3];
int gyro_num_samples = 0;

// DCM variables
float MAG_Heading;
float Accel_Vector[3]= {0, 0, 0}; // Store the acceleration in a vector
float Gyro_Vector[3]= {0, 0, 0}; // Store the gyros turn rate in a vector
float Omega_Vector[3]= {0, 0, 0}; // Corrected Gyro_Vector data
float Omega_P[3]= {0, 0, 0}; // Omega Proportional correction
float Omega_I[3]= {0, 0, 0}; // Omega Integrator
float Omega[3]= {0, 0, 0};
float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};
float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Euler angles
float yaw;
float pitch;
float roll;

// DCM timing in the main loop
unsigned long timestamp;
unsigned long timestamp_old;
float G_Dt; // Integration time for DCM algorithm

// More output-state variables
boolean output_stream_on;
boolean output_single_on;
int curr_calibration_sensor = 0;
boolean reset_calibration_session_flag = true;
int num_accel_errors = 0;
int num_magn_errors = 0;
int num_gyro_errors = 0;

void read_sensors() {
  //Read_Gyro(); // Read gyroscope
  getGyroValues(); // ITG3200
  Read_Accel(); // Read accelerometer
  Read_Magn(); // Read magnetometer
}

// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
// TODO re-init global vars?
void reset_sensor_fusion() {
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};

  read_sensors();
  timestamp = millis();
  
  // GET PITCH
  // Using y-z-plane-component/x-component of gravity vector
  pitch = -atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));
	
  // GET ROLL
  // Compensate pitch of gravity vector 
  Vector_Cross_Product(temp1, accel, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  // Normally using x-z-plane-component/y-component of compensated gravity vector
  // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
  // Since we compensated for pitch, x-z-plane-component equals z-component:
  roll = atan2(temp2[1], temp2[2]);
  
  // GET YAW
  Compass_Heading();
  yaw = MAG_Heading;
  
  // Init rotation matrix
  init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
}

// Apply calibration to raw sensor readings
void compensate_sensor_errors() {
    // Compensate accelerometer error
    accel[0] = (accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
    accel[1] = (accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
    accel[2] = (accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;

    // Compensate magnetometer error
#if CALIBRATION__MAGN_USE_EXTENDED == true
    for (int i = 0; i < 3; i++)
      magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
    Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);
#else
    magnetom[0] = (magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
    magnetom[1] = (magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
    magnetom[2] = (magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;
#endif

    // Compensate gyroscope error
    gyro[0] -= GYRO_AVERAGE_OFFSET_X;
    gyro[1] -= GYRO_AVERAGE_OFFSET_Y;
    gyro[2] -= GYRO_AVERAGE_OFFSET_Z;
}

// Reset calibration session if reset_calibration_session_flag is set
void check_reset_calibration_session()
{
  // Raw sensor values have to be read already, but no error compensation applied

  // Reset this calibration session?
  if (!reset_calibration_session_flag) return;
  
  // Reset acc and mag calibration variables
  for (int i = 0; i < 3; i++) {
    accel_min[i] = accel_max[i] = accel[i];
    magnetom_min[i] = magnetom_max[i] = magnetom[i];
  }

  // Reset gyro calibration variables
  gyro_num_samples = 0;  // Reset gyro calibration averaging
  gyro_average[0] = gyro_average[1] = gyro_average[2] = 0.0f;
  
  reset_calibration_session_flag = false;
}

void turn_output_stream_on()
{
  output_stream_on = true;
  digitalWrite(STATUS_LED_PIN, HIGH);
}

void turn_output_stream_off()
{
  output_stream_on = false;
  digitalWrite(STATUS_LED_PIN, LOW);
}

// Blocks until another byte is available on serial port
char readChar()
{
  while (Serial.available() < 1) { } // Block
  return Serial.read();
}

void setup()
{
  // Init serial output
  Serial.begin(OUTPUT__BAUD_RATE);          // Terminal
  Serial1.begin(OUTPUT__BAUD_RATE);         // Communication with motors via ESC
  
  // Messagens
  Serial.println();
  Serial.println("Inicializando...");
  Serial.println("PROJETO - PJ_PAS.PPSE");
  
  // Init status LED
  pinMode (STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  
  // Ethernet Communication
  Ethernet.begin(mac,ip);
  Udp.begin(localPort);
  
  // Init sensors
  delay(50);  // Give sensors enough time to start
  I2C_Init();
  Accel_Init();
  Magn_Init();
  setupL3G4200D(2000);
  //Gyro_Init();
  
  // Read sensors, init DCM algorithm
  delay(20);  // Give sensors enough time to collect data
  reset_sensor_fusion();
  
  /* Set message header. This can be any byte. */
  sen_data_local.msg_id=25;
  
  /* Initialize Motor controllers. */
  Init_Motors();
  
  send_2_motors_acm(1, 0);      // x
  send_2_motors_acm(2, 0);      // z
  send_2_motors_acm(3, 0);      // y

  // Init output
#if (OUTPUT__HAS_RN_BLUETOOTH == true) || (OUTPUT__STARTUP_STREAM_ON == false)
  turn_output_stream_off();
#else
  turn_output_stream_on();
#endif
}

// Main loop
void loop()
{
  uint8_t *Buffer_ptr;
  uint8_t a;
  char byte_received;
  
  // Request values from Motors Feedback to send to IMA
  switch(motor_sequence_request)
  {
    case 0 :  sen_data_local.motor_z_sp = (request_motor_speed(2).toInt())-motor_speed_default;
              motor_sequence_request++;
              break;
              
    case 1 :  sen_data_local.motor_x_sp = (request_motor_speed(1).toInt())-motor_speed_default;
              motor_sequence_request++;
              break;
              
    case 2 :  sen_data_local.motor_y_sp = (request_motor_speed(3).toInt())-motor_speed_default;
              motor_sequence_request = 0;
              break;
  }

  // Ethernet Routine
        // if there's data available, read a packet
        int packetSize = Udp.parsePacket();
          if(packetSize)
          {
            // read the packet into packetBufffer
            Udp.read(packetBuffer,UDP_TX_PACKET_MAX_SIZE);
            
            #if PRINT_ETHERNET == 1
            
            Serial.println("UDP DATA RECEIVED");
            
            Serial.print("Received packet of size ");
            Serial.println(packetSize);
            Serial.print("From ");
            IPAddress remote = Udp.remoteIP();
            
            for (int i =0; i < 4; i++)
            {
              Serial.print(remote[i], DEC);
              if (i < 3)
              {
                Serial.print(".");
              }
            }
            
            Serial.print(", port ");
            Serial.println(Udp.remotePort());
  
            Serial.println("Contents: ");
            
            for(a=0; a<12; a++)
            {
              Serial.println((int)packetBuffer[a], DEC);
            }
            
            #endif
            
            // Inicialização do ponteiro Buffer com o Buffer de recepção via Ethernet
            // Envio dos dados do Buffer da Ethernet para o mot_data.msg_id
            Buffer_ptr = (uint8_t *) &packetBuffer;
            read_from_ima_ethernet(Buffer_ptr);                  // Envia os dados recebidos do Buffer de Ethernet para a estrutura mot_data
            
            send_2_motors_acm(1, mot_data_received.motor_x_sp);  // Envia comando de velocidade para o motor
            //sen_data_local.motor_x_sp = request_motor_speed(1).toInt();                    // Requisita velocidade do motor
            
            send_2_motors_acm(3, mot_data_received.motor_y_sp);  // Envia comando de velocidade para o motor
            //sen_data_local.motor_y_sp = request_motor_speed(3).toInt();                    // Requisita velocidade do motor
            
            send_2_motors_acm(2, mot_data_received.motor_z_sp);  // Envia comando de velocidade para o motor
            //sen_data_local.motor_z_sp = request_motor_speed(2).toInt();                    // Requisita velocidade do motor
            
            // Carrega valores constantes para testes de Depuraçao
            //sen_data_local.motor_x_sp = 1000;
            //sen_data_local.motor_y_sp = 0;
            //sen_data_local.motor_z_sp = 0;
            
            //pitch = ToRad(10);
            //roll = ToRad(20);
            //yaw = ToRad(30);
            
            send_to_ima();
            
            #if PRINT_ETHERNET == 1
              Serial.println("Resposta enviada via UDP");
            #endif
            //aux_ptr = (uint8_t *) &mot_data_to_ima.motor_x_sp;
            //aux_dst = (uint8_t *) &m_vel;
            
            //aux_ptr = (uint8_t *) &mot_data_to_ima;
          }  
  
  
  // Read incoming control messages
  if (Serial.available())
  {
    
    byte_received = Serial.read();
    // Se recebe 'a' muda para controle interno ou vice versa
    
    switch(byte_received)
    {
      case 'a' : override_control_flag = 0xFF;
                 break;
                 
      case 'm' : override_control_flag = 0;
                 send_2_motors_acm(2, 0); // Volta rotação para offset
                 break;
    }
    //
  }

  // Time to read the sensors again?
  if((millis() - timestamp) >= OUTPUT__DATA_INTERVAL)
  {
    timestamp_old = timestamp;
    timestamp = millis();
    if (timestamp > timestamp_old)
      G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else G_Dt = 0;

    // Update sensor readings
    read_sensors();

    if (output_mode == OUTPUT__MODE_CALIBRATE_SENSORS)  // We're in calibration mode
    {
      check_reset_calibration_session();  // Check if this session needs a reset
      if (output_stream_on || output_single_on) output_calibration(curr_calibration_sensor);
    }
    else if (output_mode == OUTPUT__MODE_ANGLES)  // Output angles
    {
      // Apply sensor calibration
      compensate_sensor_errors();
    
      // Run DCM algorithm
      Compass_Heading(); // Calculate magnetic heading
      Matrix_update();
      Normalize();
      Drift_correction();
      Euler_angles();
      
      // Change values to Ethernet
      sen_data_local.roll = roll;
      sen_data_local.pitch = pitch;
      sen_data_local.yaw = yaw;
      
      if ((output_stream_on || output_single_on))// && (interaction > INTERACTION_NUMBER))
     {
       interaction = 0;
       output_angles();
     }
    }
    else  // Output sensor values
    {      
      if (output_stream_on || output_single_on) output_sensors();
    }
    
    output_single_on = false;
    
#if DEBUG__PRINT_LOOP_TIME == true
    Serial.print("loop time (ms) = ");
    Serial.println(millis() - timestamp);
#endif
  }
#if DEBUG__PRINT_LOOP_TIME == true
  else
  {
    Serial.println("waiting...");
  }
#endif
//

//#if (override_control)
if(override_control_flag)
{
  yaw_control_over_ride();
  Serial.println("Modo Manual");
}
  //Serial.println(override_control_flag);
//#endif
interaction++;
//Serial.println(interaction);
//
}
