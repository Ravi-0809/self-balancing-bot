#include <I2Cdev.h>
#include <PID_v1.h>
#include <MPU6050_6Axis_MotionApps20.h>
#define moving 0
#define enableYaw 0

bool moveFlag = false;
MPU6050 mpu;

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

 

/*********Tune these 4 values for your BOT*********/
double setpoint1= 178.5, setpoint2 = 314.5; //set the value when the bot is perpendicular to ground using serial monitor. 
//Read the project documentation on circuitdigest.com to learn how to set these values
double Kpp = 40; //Set this first
double Kdp = 0.75; //Set this secound
double Kip = 300; //Finally set this 

double Kpy = 10; //Set this first
double Kdy = 0.7; //Set this secound
double Kiy = 150; //Finally set this 
/******End of values setting*********/

double input1, input2, output1, output2;
PID pid1(&input1, &output1, &setpoint1, Kpp, Kip, Kdp, DIRECT);
PID pid2(&input2, &output2, &setpoint2, Kpy, Kiy, Kdy, DIRECT); 
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
    mpuInterrupt = true;
}

void setup() {
  
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
//    mpu.setXGyroOffset(220);
//    mpu.setYGyroOffset(76);
//    mpu.setZGyroOffset(-85);
//    mpu.setZAccelOffset(1688); 

      mpu.setXGyroOffset(62);
      mpu.setYGyroOffset(21);
      mpu.setZGyroOffset(1);
      mpu.setZAccelOffset(771);

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
        pid1.SetSampleTime(10);
        pid1.SetOutputLimits(-255, 255);
        pid2.SetMode(AUTOMATIC);
        pid2.SetSampleTime(10);
        pid2.SetOutputLimits(-50, 50);  
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
    
//Initialise the Motor outpu pins
    #define RM1 9
    #define RM2 6
    #define LM1 10
    #define LM2 11

    // To control the motors. To stop the motors alltogether change to 0
    
    pinMode (6, OUTPUT);
    pinMode (9, OUTPUT);
    pinMode (10, OUTPUT);
    pinMode (11, OUTPUT);
    pinMode(7,OUTPUT);
    pinMode(8,OUTPUT);

//By default turn off both the motors
    analogWrite(11,LOW);
    analogWrite(10,LOW);
    analogWrite(6,LOW);
    analogWrite(9,LOW);
    digitalWrite(7,HIGH);
    digitalWrite(8,HIGH);
}

 

void loop() {
 
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize)
    {
        //no mpu data - performing PID calculations and output to motors     
        pid1.Compute();   
        pid2.Compute();
        //Print the value of Input and Output on serial monitor to check how it is working.
        Serial.print(input1); Serial.print(" =>"); Serial.print(output1);Serial.print(" "); Serial.print(input2); Serial.print(" =>"); Serial.println(output2);

        if (moving == 1)
        {        
          if (input1>150 && input1<210){//If the Bot is falling 
            if (output1>0) //Falling towards front 
            Forward(); //Rotate the wheels forward 
            else if (output1<0) //Falling towards back
            Reverse(); //Rotate the wheels backward
            moveFlag = true; 
          }
          else
          {
            Stop();
            continue;
          }

          if(enableYaw)
          {
            if (input2>290 && input2<340){//If the Bot is turning
              if (output2>0)
              Right(); 
              else if (output2<0)
              Left(); 
              moveFlag = true;
            }
          }
          if(!moveFlag) //If Bot is dead
          {
            moveFlag = false;
            Stop(); //Hold the wheels still
          }
        }
        
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
        if(enableYaw)
          input2 = ypr[0] * 180/M_PI + 180;

   }
   
}

void Forward() //Code to rotate the wheel forward 
{
    analogWrite(RM2,output1);
    analogWrite(RM1,0);
    analogWrite(LM1,output1);
    analogWrite(LM2,0);
    Serial.print("F"); //Debugging information 
}

void Reverse() //Code to rotate the wheel Backward  
{
    analogWrite(RM2,0);
    analogWrite(RM1,output1*-1);
    analogWrite(LM1,0);
    analogWrite(LM2,output1*-1); 
    Serial.print("B");
}

void Left() //Code to rotate the wheel Backward  
{
    analogWrite(RM2,0);
    analogWrite(RM1,output2*-1);
    analogWrite(LM1,output2*-1);
    analogWrite(LM2,0); 
    Serial.print("L");
}

void Right() //Code to stop both the wheels
{
    analogWrite(RM1,output2*-1);
    analogWrite(RM2,0);
    analogWrite(LM2,0);
    analogWrite(LM1,output2*-1);
    Serial.print("R");
}

void Stop() //Code to stop both the wheels
{
    analogWrite(RM2,0);
    analogWrite(RM1,0);
    analogWrite(LM2,0);
    analogWrite(LM1,0); 
    Serial.print("S");
}
