#ifndef quadcopter1
#define quadcopterl

#if (ARDUINO >=100)
  #include "Arduino.h"
#else
  #include "WProgram.h"
#endif


#include "C:\Program Files (x86)\Arduino\hardware\teensy\avr\libraries\Servo\Servo.h"
#include <Wire.h>
#include <MPU6050.h>
#include <LPS.h>
#include <LIS3MDL.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <Adafruit_BMP280.h>
#include <SPI.h>
#include <SD.h>


LIS3MDL mag;
MPU6050 sensor;
LPS ps;    //Pololu Pressure
TinyGPS gps;
SoftwareSerial ss(4, 3);  //GPS
Adafruit_BMP280 bmp;
File myFile;    //SD_Card


class Initialiser {
  public:
    Initialiser();

    //method that handles all initialisations
    void InitialiseSystem();
   

    //methods to initialise each individual system
    void initialisePololuPressure();
    void initialiseSD_Card();
    void initialisePropellers();
    void initialiseGPS();
    void initialiseHSR05();
    void initialiseBN055();
    void initialiseMPU6050();
    void initialiseBluetooth();
    void initialiseLeg_motors();
    void initialiseBMS();
    void initialiseMagnetometer();

    
  private:

  //HSR05 init
    Servo s1_prop;
    Servo s2_prop;
    Servo s3_prop;
    Servo s4_prop;

  //GPS
  bool newData;
  unsigned long chars;
  unsigned short sentences, failed;


};


class Buzzer  {
  public:
     
    Buzzer();
    void powerOn();
    void powerOff();

    

  private:

    float E = 329.628;
    float F_sharp = 369.994;
    int16_t G = 392;
    int16_t A  = 440;

    int buzzer_pin = 14;

};


class Led {

  public:
  
    Led(int r, int g, int b);

    void blinkLed();


  private:

    uint16_t red_value;
    uint16_t green_value;
    uint16_t blue_value;
 
    uint16_t red_pin_led = A1;
    uint16_t green_pin_led = A2;
    uint16_t blue_pin_led = A3;

};


class BMS {
  public:
    BMS();
    float getVoltage();
    
  private:
    uint8_t voltage_read_pin = A3; 
    uint8_t val;

};

class ControlSystem {


  public:
    ControlSystem();

    void eject();

    //getError methods
    float getPololuPressureError(float usr_altitude);
    float getBN055PressureError(float usr_altitude);
    float * getGPSError(float usr_coord);
    float getMPU6050Error(float usr_pitchAngle, float usr_rollAngle);
    float getMagnetometerError(float usr_yawAngle);
    float getHSR05Error();
    
    
  private:
    
    //legs
    #define PY1 20
    #define PY2 21

    //HSR05
    #define trig_pin 3
    #define echo_pin 4
    float duration;
    float distance;
    uint8_t desired_distance = 0.5;

    //Pressure_Pololu && BMP280
    pressure = 0;
    int myAltitude_above_sea_level = 1583;

    //GPS
    float error[3];
    float NEMA_Sentence[3];
    bool newData = false;
    unsigned long chars;
    unsigned short sentences, failed;
  
    //Magnetometer
    double d;
    double xGaussData;
    double yGaussData;
    char report[80];
    int PI_CONST = 3.1415;
    //the MPU6050 rad_to_deg variable is used in Magnetometer as well

    //MPU6050
    int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
    float Acceleration_angle[2];
    float Gyro_angle[2];
    float Total_angle[2];
    float desired_pitch_angle = 0, 
    fLOt desired_roll_angle = 0;
    float elapsedTime, time, timePrev;
    float desired_angle = 0;
    float rad_to_deg = 180/3.141592654;

    //enum to find quaterile states
    enum Quaterile {

	Q1, Q2, Q3, Q4		

    };

    

};


class Control {
  
  public:
    float previous_error;

    Control(float usr_altitude, float height_above_objects, 
             float usr_pitchAngle, float usr_rollAngle, 
              float usr_yawAngle, float usr_coord[]);

    //methods to calculate the 4 actuator speeds of each system
    float getPololuPressurePID_Values();
    float getBN055PressurePID_Values();
    float getGPSPID_Lat_Values();
    float getGPSPID_Long_Values();
    float getMPU6050PID_Pitch_Values();
    float getMPU6050PID_Roll_Values();
    float getMagnetometerPID_Values();
    float getHSR05PID_Values();

    //Logic
    void Logic();

    //Motor mixing algorithm
    void MMA();
    

  private:

    //Instantiations
    ControlSystem controlSystem;
    Buzzer buzzer;
    BMS bms;
    Led led;
    ErrorDetection errordetection;
    

    //motors
    int throttle = 1100;
    float pwm_s1;
    float pwm_s2;
    float pwm_s3;
    float pwm_s4;
    Servo s1_prop;
    Servo s2_prop;
    Servo s3_prop;
    Servo s4_prop;

    //PID for motors
    float PID_s1;
    float PID_s2;
    float PID_s3;
    float PID_s4;

    //Control
    int throttle = 1100;
    float PID, error;
    float Lat_PID, Long_PID;
    float pid_p = 0;
    //integral values need individual definition cause of 
       //controller PID summing
    float pid_i_pololu = 0;
    float pid_i_BMP280 = 0;
    float pid_i_Lat_GPS = 0;
    float pid_i_Long_GPS = 0;
    float pid_i_MPU6050_pitch = 0;
    float pid_i_MPU6050_roll = 0;
    float pid_i_Magnetometer = 0;
    float pid_d_HSR05 = 0;
    float kp;
    float ki;
    float kd;

    //Error values
    float pololuPressure_error;
    float BN055_error;
    float HSR05_error;
    float Magnetometer_error;
    float * GPS_error;
    float MPU6050_error_Pitch;
    float MPU6050_error_Roll;


    //Control Error Values
    int flying_altitide;
    float elapsedTime, time, timePrev;

    
    
};

class ErrorDetection {
  public:
    ErrorDetection();

    bool flag();

    bool batteryVoltageReading();
    bool HSR05_error();
    bool BN055_error();
    bool GPS_error();
    bool Magnetometer_error();
    bool PololuPressure_error();
    bool MPU6050_error();

  private:

    float _voltageReading;
    float _distanceE;
    float _pressureE;
    float _gpsE;
    float _magE;
    float _pololuPressureE;
    float _MPU6050E_pitch;
    float _MPU6050E_roll;


};

class SystemStates {
  public:
  
    void Run() //runs system states 

    //System States
    void StartFlight();
    void Hover();
    void Land();
    int endFlight();

    //for GPS Hover parameter //NB needs to be the same as whatever 
                                 //the current NEMA sentence is
    float * getCurrentNEMASentence(); 
    
  private:
    ControlSystem controlsystem;

    //default params for all objects of this class
    int pitchAngle;
    int rollAngle;

    //for method Hover
    float current_altitude_error;
    float current_yaw_error;
    float height_above_object;
    float *current_coord;

    //for method Translate
    float desired_yaw_angle;
    float usr_altitude = 1;
    float height_above_objects = 0.5;
    float usr_yawAngle;
    float * usr_coord;
    float lat_coord;
    float lon_coord;

    //GPS current NEMA sentence
    float * defaultNEMASentence;
    float NEMA_Sentence[2];

    // variables to calculate current bearing
    float d;   //distance between two points
    float r;   // radius of the earth
    float Ad;  // Angualr distance
    float bearing;  



  
};


class Bluetooth {
    public:
      Bluetooth();
      char getInput();
      
    private:
      char state = 0;
};

#endif
