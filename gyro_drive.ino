

//GYRO INITIALIZATIONS

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high




#define OUTPUT_READABLE_YAWPITCHROLL




#define LED_PIN 13  // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
char r='o';

                                                                 // MPU control/status vars
bool dmpReady = false;                                                 // set true if DMP init was successful
uint8_t mpuIntStatus;                                              // holds actual interrupt status byte from MPU
uint8_t devStatus;                                                // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;                                                // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;                                               // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];                                            // FIFO storage buffer

// orientation/motion vars
Quaternion q;                                                       // [w, x, y, z]         quaternion container
VectorInt16 aa;                                                // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;                                                 // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;                                               // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;                                              // [x, y, z]            gravity vector
float euler[3];                                                     // [psi, theta, phi]    Euler angle container
float ypr[3];                                                         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

                                                                        // packet structure for InvenSense teapot demouint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

float yaw=0 , initial_yaw=0;



int enc_1=0;
int enc_2=1;                        //enc intitialization

int motor_1_fw=23;
int motor_1_bw=22;         
int motor_2_fw=27;                  //motor intitialization
int motor_2_bw=26;


int motor_1_pwm=8;
int motor_2_pwm=9;                   //motor intitialization 



int initial_pos_1=0;
int current_pos_1=0;
float steps_1=0;                                //steps func intitialization for 1st enc
int enc_steps_1(int,int,float,int);




int initial_pos_2=0;
int current_pos_2=0;                            //steps func intitialization for 2nd enc
float steps_2=0;


int f;
float t=979;

int target_2=-979*0;
float kp_2=0.248 , ki_2=0.00687, kd_2=0;
boolean a_2=LOW,b_2=LOW;int pwmval_2=0;              //pid initialization enc 2

int pidfunc_2(int,int,int);
int target_form_2[10];



int target_1=1020*0;
float kp_1=0.248, ki_1=0.006871, kd_1=0;
boolean a_1=LOW,b_1=LOW;int pwmval_1=0;              //pid initialization enc 1

int pidfunc_1(int,int,int);
int target_form_1[10];

int initial_timer=millis();
int final_timer=millis();


float kp_g=2, ki_g=0.0, kd_g=0;
float target_g=0;


void setup() {
     #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 12;                                                                   // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

                                                                                             // initialize serial communication
                                                                                           // (115200 chosen because it is required for Teapot Demo output, but it's
                                                                                             // really up to you depending on your project)
    Serial.begin(9600);
    Serial.println("start");
    while (!Serial); 
      mpu_initialize();                                                                                        // join I2C bus (I2Cdev library doesn't do this automatically)
   
while(1)
{initial_yaw=gyro_values();Serial.println(initial_yaw);


if(millis()>15500)
{break;}
}

pinMode(enc_1,INPUT);
pinMode(enc_2,INPUT);


pinMode(motor_1_fw,OUTPUT);
pinMode(motor_1_bw,OUTPUT);
pinMode(motor_2_fw,OUTPUT);
pinMode(motor_2_bw,OUTPUT);
pinMode(motor_1_pwm,OUTPUT);
pinMode(motor_2_pwm,OUTPUT);


initial_pos_1=analogRead(enc_1);
initial_pos_2=analogRead(enc_2);



}





void loop() {

    yaw=gyro_values()-initial_yaw;
    //Serial.println(yaw);


      /*if(target_1>1000   ||    target_1<-1000)   
  
  {//kp_1=0.218; ki_1=0.006871; kd_1=0;
   kp_1=0.245-abs(target_1/500000); ki_1=0.007871-abs(target_1/500000); kd_1=0;}
  else
  {kp_1=0.218; ki_1=0.006871; kd_1=0;}

  if(target_2>1000   ||    target_2<-1000)
  {//kp_2=0.262 ; ki_2=0.00687; kd_2=0;
   kp_2=0.262-abs(target_2/500000) ; ki_2=0.00757-abs(target_1/500000); kd_2=0;
  }
  else
  {kp_2=0.262 ; ki_2=0.00687; kd_2=0;}*/

/*
if(Serial.available()>0)
{int s=Serial.read();
if(s=='q')
{
target_1=target_1+1000+(1000/1020*41);
target_2=target_2-1000;}
if(s=='r')
{t=t-200;
target_1=target_1-1000+(1000/1020*41);
target_2=target_2+1000;}
if(s=='t')
{target_1=target_1+820;
target_2=target_2+970;}
if(s=='s')
{target_1=target_1-820;
target_2=target_2-970;}
int m1=pidfunc_2(0,0,0);
int m2=pidfunc_1(0,0,0);
if(s=='u')
{yaw=gyro_values();}
}*/
/*ki_1=0.002*1000/target_1;
ki_2=0.002*1000/target_2;

if(target_1<1000   &&   target_1>-1000)
{ki_1=0.00015;}
if(target_2<1000   &&   target_2>-1000)
{ki_2=0.0001;}





/*final_timer=millis();
if(final_timer-initial_timer>8000)
{//t=-t+500;
initial_timer=final_timer;
}*/

current_pos_1=analogRead(enc_1);
steps_1=enc_steps(current_pos_1,initial_pos_1,steps_1,0);{  initial_pos_1=current_pos_1;}//Serial.print(steps_1);Serial.print("          ");


current_pos_2=analogRead(enc_2);
steps_2=enc_steps(current_pos_2,initial_pos_2,steps_2,1);{  initial_pos_2=current_pos_2;}//Serial.print(steps_2);Serial.print("          ");
//Serial.print(yaw);
//Serial.print(target_1);Serial.print("          ");Serial.println(target_2);



//pwmval_2=pidfunc_2(steps_2,target_2,pwmval_2);
//pwmval_1=pidfunc_1(steps_1,target_1,pwmval_1);


if(Serial.available()>0)
{r=Serial.read();}
digitalWrite(13,HIGH);
if(r=='s')
{int q=pid_func_gyro(target_g);
Serial.print(yaw);Serial.print("           ");Serial.println(q);
digitalWrite(motor_1_fw,1);
digitalWrite(motor_1_bw,0);
analogWrite(motor_1_pwm,220+q);//Serial.print(pwmval_1);Serial.print("          ");



digitalWrite(motor_2_fw,0);
digitalWrite(motor_2_bw,1);
analogWrite(motor_2_pwm,180-q);//Serial.println(pwmval_2);
digitalWrite(13,LOW);}

else
{Serial.println("brake");
digitalWrite(motor_1_fw,1);
digitalWrite(motor_1_bw,0);
analogWrite(motor_1_pwm,0);//Serial.print(pwmval_1);Serial.print("          ");



digitalWrite(motor_2_fw,0);
digitalWrite(motor_2_bw,1);
analogWrite(motor_2_pwm,0);//Serial.println(pwmval_2);delay(4000);r='o';
}












//digitalWrite(motor_1_fw,LOW);
//digitalWrite(motor_1_bw,HIGH);
//analogWrite(motor_1_pwm,250);


//digitalWrite(motor_2_fw,HIGH);
//digitalWrite(motor_2_bw,LOW);
//analogWrite(motor_2_pwm,250);
//Serial.println(steps_1+steps_2);
}







void mpu_initialize()
{mpu.initialize();
devStatus = mpu.dmpInitialize();
 mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

if (devStatus == 0) {
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
    } else {
        
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);}







float gyro_values()
{while (!mpuInterrupt && fifoCount < packetSize) {
   }
 mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
 fifoCount = mpu.getFIFOCount();
 if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
                                                                                     // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));
                                                                                       // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
                                                                              // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
               mpu.getFIFOBytes(fifoBuffer, packetSize);
               mpu.resetFIFO();                                                      // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

       #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            /*Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);*/
        #endif

       
        return(ypr[0]*180/M_PI);
    }
    }


int enc_steps(int cp,int ip,float s,int l)
{ 
  int ref_vol;
  if(l=0)
  {ref_vol=1020;}
  else
  {ref_vol=979;}

  if(cp>ip+2  ||  cp+2<ip  )
  {digitalWrite(13,HIGH);
    if(cp>ip)
    {   if(cp>ip+480)
             {s=s-(ip+1+ref_vol-cp);}
        else
      {s=s+(cp-ip);}
      }

   if(cp<ip)
    {   if(cp+480<ip)
             {s=s+(cp+1+ref_vol-ip);}
        else
              {s=s-(ip-cp);}
              }
    
    
   
    //Serial.print(s);Serial.print("                   ");
    //delay(10);
  
    

   
   
 

  }
   return(s);
}




int pidfunc_2(int currentvalue,int targetvalue,int pwmvalue)

 {
   pwmvalue=0;
    int error=targetvalue-currentvalue;static int lasterror=0;static float totalerror=160;
    float pidterm=0;
    


    if(currentvalue==0   &&   targetvalue==0     &&    pwmvalue==0)
    {totalerror=0;}



    
     
     if(error>0)
    {a_2=HIGH;b_2=LOW;}
    else
    {a_2=LOW;b_2=HIGH;}

  
    totalerror=totalerror+(error);//Serial.println(totalerror);
    pidterm=kp_2*(error)+ki_2*(totalerror)+kd_2*(error-lasterror);totalerror=constrain(totalerror,-17000,17000);
    lasterror=error;pidterm=abs(pidterm);
    //Serial.print(constrain(abs(pwmvalue+pidterm),0,235));Serial.print("          ");
    //Serial.println(error);Serial.print("      ");
    
    return(constrain((pwmvalue+pidterm),0,233-5));
   
   }




int pidfunc_1(int currentvalue,int targetvalue,int pwmvalue)

 {
   pwmvalue=0;
    int error=targetvalue-currentvalue;static int lasterror=0;static float totalerror=0;
    float pidterm=0;
  


      if(currentvalue==0   &&   targetvalue==0     &&    pwmvalue==0)
    {totalerror=0;}


 
     
     
     if(error>0)
    {a_1=HIGH;b_1=LOW;}
    else
    {a_1=LOW;b_1=HIGH;}
    
    
    totalerror=totalerror+(error);totalerror=constrain(totalerror,-17000,17000);
    pidterm=kp_1*(error)+ki_1*(totalerror)+kd_1*(error-lasterror);
    lasterror=error;pidterm=abs(pidterm);
    //Serial.print(constrain(abs(pwmvalue+pidterm),0,235));Serial.print("          ");
    //Serial.println(error);Serial.print("      ");
   
    return(constrain((pwmvalue+pidterm),0,235));
   
   }



int pid_func_gyro(int target_yaw)
{float currentvalue=gyro_values()-initial_yaw;
float error=currentvalue-target_yaw; static int lasterror=0;static float totalerror=0;
float pidterm=0;
 totalerror=totalerror+(error);totalerror=constrain(totalerror,-1700,1700);
    pidterm=kp_g*(error)+ki_g*(totalerror)+kd_g*(error-lasterror);
    lasterror=error;//pidterm=abs(pidterm);
    
return(constrain((pidterm),-50,35));

}
