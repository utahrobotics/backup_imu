
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#define LED_PIN 13
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif



Quaternion mpuQ; //This is the quaternion we'll get from the IMU
MPU6050 mpu;

uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
int dmpStatus;

int16_t ax, ay, az;
int16_t gx, gy, gz;

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup()
{
    pinMode(LED_PIN, OUTPUT);
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    mpu.initialize();
    pinMode(2, INPUT); //Interrupt pin
    dmpStatus = mpu.dmpInitialize();
    //Gyro Offsets????

    //These were all grabbed using the ZEROing example code provided in the mpu6050 library
    mpu.setXAccelOffset(-375);
    mpu.setYAccelOffset(-4783);
    mpu.setZAccelOffset(1445);
    mpu.setXGyroOffset(51);
    mpu.setYGyroOffset(-12);
    mpu.setZGyroOffset(-3);
    

    if(dmpStatus == 0)
        mpu.setDMPEnabled(true);

    packetSize = mpu.dmpGetFIFOPacketSize();

   attachInterrupt(digitalPinToInterrupt(2), dmpDataReady, RISING);
   Serial.begin(57600);
}

void loop()
{
   // digitalWrite(LED_PIN,LOW);

    if(dmpStatus != 0)
        return;
    mpuInterrupt = false;

    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    while(!mpuInterrupt)
    {
      //This holds everything so we can make sure we have data before tryig to talk to the IMU
    }

     if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
     }
    else if(mpuIntStatus & 0x02)
    {
       
         // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        //digitalWrite(LED_PIN, HIGH);
         // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&mpuQ, fifoBuffer);
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        //begin sending over serial
        Serial.println("a");//This just makes it so we can sync by calling readline.
        //Start with the quaternion
        byte * sendingReference = (byte*) &mpuQ.w;
        Serial.write(sendingReference,4);
        sendingReference = (byte*) &mpuQ.x;
        Serial.write(sendingReference,4);
        sendingReference = (byte*) &mpuQ.y;
        Serial.write(sendingReference,4);
        sendingReference = (byte*) &mpuQ.z;
        Serial.write(sendingReference,4);
        
        //start sending accelerometer data.
        sendingReference = (byte*) &ax;
        Serial.write(sendingReference,2);   
        sendingReference = (byte*) &ay;
        Serial.write(sendingReference,2);
        sendingReference = (byte*) &az;
        Serial.write(sendingReference,2);      

        //start sending gyro data.
        sendingReference = (byte*) &gx;
        Serial.write(sendingReference,2);   
        sendingReference = (byte*) &gy;
        Serial.write(sendingReference,2);
        sendingReference = (byte*) &gz;
        Serial.write(sendingReference,2);      
        
        mpuInterrupt = false;
    }
    
}
