#include <I2Cdev.h>
#include <PID_v1.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include<Servo.h>
Servo myservo;


bool moveFlag = false;
MPU6050 mpu;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
uint8_t servo_angle;
// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float last_time;
float angle;
 

/*********Tune these 4 values for your BOT*********/
double setpoint1= 181; //set the value when the bot is perpendicular to ground using serial monitor. 
//Read the project documentation on circuitdigest.com to learn how to set these values
double Kpp = 20; //Set this first
double Kdp = 0; //Set this secound
double Kip = 0; //Finally set this 

/******End of values setting*********/

double input1, output1;
PID pid1(&input1, &output1, &setpoint1, Kpp, Kip, Kdp, DIRECT);

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup() {
    angle=90;
     myservo.write(angle);
     delay(150);
      last_time=0;
  pinMode(8,OUTPUT);
  digitalWrite(8,HIGH);
  myservo.attach(5);
  myservo.write(70);
  Serial.begin(115200);

  // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize(); 

     // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    devStatus = mpu.dmpInitialize();
    
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1688); 

//      mpu.setXGyroOffset(62);
//      mpu.setYGyroOffset(21);
//      mpu.setZGyroOffset(1);
//      mpu.setZAccelOffset(771);

      // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
        
        //setup PID
        pid1.SetMode(AUTOMATIC);
        pid1.SetSampleTime(100);
        pid1.SetOutputLimits(90,270);
  }
    else
    {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
  
      
}

 

void loop() {
 
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        //no mpu data - performing PID calculations and output to motors     
        pid1.Compute();   
        //Print the value of Input and Output on serial monitor to check how it is working.
        Serial.print(input1); Serial.print(" =>"); Serial.print(output1);Serial.println(" "); 
   
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current  count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024)
    {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    }
    else if (mpuIntStatus & 0x02)
    {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        mpu.dmpGetQuaternion(&q, fifoBuffer); //get value for q
        mpu.dmpGetGravity(&gravity, &q); //get value for gravity
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); //get value for ypr
        input1 = ypr[2] * 180/M_PI + 180;
        if(input1>setpoint1+5)
        {
          if(millis()-last_time>15){
          angle=angle -1.5;
          myservo.write((int)angle);
          last_time=millis();
          }
        }
        if(input1<setpoint1-5)
        {
          if(millis()-last_time>15)
          {
          angle=angle +1.5;
          myservo.write((int)angle);
          last_time=millis();
          }
        }
        
        
        
   }
   
//  delay(15);
   
}
