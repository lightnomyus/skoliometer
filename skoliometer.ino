#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
/*koneksi: pake 3.3Volt, GND, SDA(A4)-SCL(A5), sama INT ke pin interrupt (di UNO = pin no.2)*/

#define OUTPUT_READABLE_YAWPITCHROLL
#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
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
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float roll;
float calibrateRoll = 0.00;
int isCalibrated = 0;

//ISR detection here
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

//Cope Angle Measurement
float TI=0;
float LI=0;
float TC,LC;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(115200);
    while (!Serial); // wait for Leonardo enumeration, others continue immediately
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    
    //pin declaration
    pinMode(INTERRUPT_PIN, INPUT);
    pinMode (outputA,INPUT);
    pinMode (outputB,INPUT);
    pinMode(pushButt, INPUT_PULLUP);
   
    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

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

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // Reads the initial state of the outputA
    aLastState = digitalRead(outputA);

    //Open File for SD Card
    sensorData = SD.open("data.csv", FILE_WRITE);
    copeAngle = SD.open("cope.csv", FILE_WRITE);
}

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    
    //isReadingData=1 when calibration, =2 when measurement
    while( isReadingData ==1 ){
        // wait for MPU interrupt or extra packet(s) available
        while (!mpuInterrupt && fifoCount < packetSize) {
            if (mpuInterrupt && fifoCount < packetSize) {
              // try to get out of the infinite loop 
              fifoCount = mpu.getFIFOCount();
            }  

            //switch from calibration to measurement or stop reading from sensor
            if(digitalRead(pushButt)==LOW){
//                 if( isReadingData==1 ){
//                     isReadingData ==2;
//                     calibrateRoll = ypr[2] * 180/M_PI;
//                     Serial.print("Calibrated Yawn = ");
//                     Serial.println(calibrateRoll);
//                     delay(300);
//                 }
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

            //if( isReadingData==2 ){
                // when measuring, rotary encoder is processed here
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

                    //roll = ypr[2] * 180/M_PI;
                    roll = ypr[2] * 180/M_PI - calibrateRoll;
        
                    //write data
                    Serial.print("ypr\t");
                    Serial.println(roll);

                    //write data to SD Card
                    dataString = String(jarak) + "," + String(roll); // convert to CSV
                    sensorData.println(dataString);
                    Serial.println("Data printed");

                    //measure TI and LI
                    if( jarak >0 && jarak<25 ){
                        if( abs(roll)>TI ){
                            TI = roll;
                        }
                    } else if( jarak>26 && jarak < 70 ){
                        if( abs(roll)>LI ){
                            LI = roll;
                        }
                    } 
                    
                } 
                aLastState = aState; // Updates the previous state of the outputA with the current state
            //}



        }
        // reset interrupt flag and get INT_STATUS byte
        mpuInterrupt = false;
        mpuIntStatus = mpu.getIntStatus();
    
        // get current FIFO count
        fifoCount = mpu.getFIFOCount();
    
        // check for overflow (this should never happen unless our code is too inefficient)
        if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
            // reset so we can continue cleanly
            mpu.resetFIFO();
            fifoCount = mpu.getFIFOCount();
            Serial.println(F("FIFO overflow!"));
    
        // otherwise, check for DMP data ready interrupt (this should happen frequently)
        } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)) {
            // wait for correct available data length, should be a VERY short wait
            while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    
            // read a packet from FIFO
            mpu.getFIFOBytes(fifoBuffer, packetSize);

            //clear every read FIFO
            mpu.resetFIFO();
            
            // track FIFO count here in case there is > 1 packet available
            // (this lets us immediately read more without waiting for an interrupt)
            fifoCount -= packetSize;
    
            #ifdef OUTPUT_READABLE_YAWPITCHROLL
                // display Euler angles in degrees
                mpu.dmpGetQuaternion(&q, fifoBuffer);
                mpu.dmpGetGravity(&gravity, &q);
                mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

                if ( isCalibrated==0 ){
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
                }

            #endif
        }



    }

}
