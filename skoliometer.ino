#include <MPU6050_tockn.h>
#include "Wire.h"

MPU6050 mpu6050(Wire);
/*koneksi: pake 3.3Volt, GND, SDA(A4)-SCL(A5), sama INT ke pin interrupt (di UNO = pin no.2)*/
#define pushButt 3 // Push Button buat start dan finish measurement

//Rotary Encoder
#define outputA 4 //6 //Dt
#define outputB 5 //7 //CLK
float const d = 3.7;//satuan dlm cm
float jarak;
int counter = 0;
int putaran = 0; 
int aState;
int aLastState; 
int isReadingData = 0;

//SD Card
#include "SD.h"
#include"SPI.h"
const int CSpin = 10;
String dataString =""; // holds the data to be written to the SD card

File sensorData;
File copeAngle;

// MPU control/status vars
float roll;
//float calibrateRoll = 0.00;
// int isCalibrated = 0;

//Cope Angle Measurement
float TI=0;
float LI=0;
float TC,LC;

void setup() {
    // initialize serial communication
    Serial.begin(9600);
    Wire.begin();
    mpu6050.begin();
    
    //pin declaration
    pinMode (outputA,INPUT);
    pinMode (outputB,INPUT);
    pinMode(pushButt, INPUT_PULLUP);

    //begin SD Card
    Serial.print("Initializing SD card...");
    pinMode(CSpin, OUTPUT);
    
    // see if the card is present and can be initialized:
    if (!SD.begin(CSpin)) {
        Serial.println("Card failed, or not present");
        // don't do anything more:
        return;
    }
    
    // wait for ready
    Serial.println(F("\nPress Push Button to Start: "));
    while(digitalRead(pushButt) == LOW);
    while(digitalRead(pushButt) != LOW);
    while(digitalRead(pushButt) == LOW);
    isReadingData = 1;

    // Reads the initial state of the outputA
    aLastState = digitalRead(outputA);

    //Open File for SD Card
    sensorData = SD.open("data.csv", FILE_WRITE);
    copeAngle = SD.open("cope.csv", FILE_WRITE);

    // Calibrating MPU6050
    Serial.println(F("Initializing MPU6050"));
    mpu6050.calcGyroOffsets(true);
}

void loop() {
    //update mpu
    mpu6050.update();
    roll = mpu6050.getAngleZ();
    
    //Serial.println(roll);
        //switch from calibration to measurement or stop reading from sensor
        if(digitalRead(pushButt)==LOW){
            if ( isReadingData==1 ){
                isReadingData=0;
                Serial.println("Measurement Finished");
                TC = 2.6*TI - 1.4*LI;
                LC = 2.0*LI - 1.5*TI;
                dataString = String(TC) + "," + String(LC);
                copeAngle.println(dataString);
                sensorData.close(); // close the file
                copeAngle.close();
                delay(300);
            }
        }
                //variable roll
        //roll = mpu6050.getAngleZ();
                //roll = mpu6050.getAngleZ() - calibrateRoll;

                /*if ( isCalibrated==0 ){
                    calibrateRoll = ypr[2] * 180/M_PI;
                    if( abs(calibrateRoll)<=7.00 ){
                      isCalibrated = 1;
                      calibrateRoll=abs(calibrateRoll);
                      Serial.println("Calibration Finished");
                      Serial.print("Calibrated Roll = ");
                      Serial.println(calibrateRoll);
                    } else{
                      isCalibrated = 0;
                    }
                } */

        aState = digitalRead(outputA); // Reads the "current" state of the outputA
        // If the previous and the current state of the outputA are different, that means a Pulse has occured
        if (aState != aLastState){     
            // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
            if (digitalRead(outputB) != aState) { 
                counter ++;
            } else {
                counter --;
            }
            jarak = counter*0.1*3.1416*d;
            Serial.print("Jarak : ");
            Serial.print(jarak, 4);//4 digit belakang koma
            Serial.println(" cm");

            //write data
            Serial.print("Sudut\t");
            Serial.println(roll);

            //write data to SD Card
            dataString = String(jarak) + "," + String(roll); // convert to CSV
            sensorData.println(dataString);
            Serial.println("Data printed");

            //measure TI and LI
            if( jarak >0.00 && jarak<25.00 ){
                if( abs(roll)>TI ){
                    TI = roll;
                }
            } else if( jarak>26.00 && jarak < 70.00 ){
                if( abs(roll)>LI ){
                    LI = roll;
                }
            } 
            
        } 
        aLastState = aState; // Updates the previous state of the outputA with the current states
}
