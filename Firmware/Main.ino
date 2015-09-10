////////////////////////////////////////////////////
//
// Date: 16/08/15
// Author: Luis H. Porras
// 
// Inverted Pendulum Robot
// Description: Basic inverted pendulum robot based
// on different libraries such as MPU6050.h, PID.h, 
// Filter.h (which contains FilterOnePole.h) and Wire.h
// an Arduino library included in the IDE. 
// Building this robot will provide knowledge about
// signal processing ( Highpass and Lowpass filters
// combined to achieve a Complementary filter, Kalman
// filter etc.), control theory (PID controllers, 
// neural networks and genetic algorithms etc.),
// dynamical systems theory...
// Also contains comunications via bluetooth
//
// Code map:  
// This code has been divided in blocks that summerize 
// what is going on and what it is, almost all blocks
// are independet so if you comment one the code will
// work i.e., comment the comunication functions
// and the robot will run without Protocoder
// interface.
//
///////////////////////////////////////////////////


#include <Wire.h>
#include <MPU6050.h>
#include <FilterOnePole.h>
#include <PID_v1.h>

//Define motor pins
#define M11 10
#define M21 6
#define M12 9
#define M22 5
#define DRI 7

/////////////////////////////////////////////////////////////////
//                      Sensor variables                       //

MPU6050 mpu;
float apitch = 0, gpitch = 0; // Accelerometer pitch, apitch; Gyroscope Pitch, gpitch
float fapitch = 0, fgpitch = 0, fsignal = 0; // Filtered Accelerometer pitch, fapitch; Filtered Gyroscope Pitch, fgpitch
float sum, setZero;
int i;

/////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////
//                        PID variables                        //

double Setpoint, Input, Output;
double Kp=80, Ki=18.3, Kd=0;//Ziegler-Nichols method Ku = 89, Tu=2.1; PID Kp=53.4 , Ki=50.86, Kd=14.2 noisy; PD Kp=71.2, Kd=18.7
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Timers
unsigned long timer = 0;
float timeStep = 0.01;

/////////////////////////////////////////////////////////////////

/////////////////////////////////////////////////////////////////
//                          DC Motors                          //

int offset = 3;

/////////////////////////////////////////////////////////////////
  
/////////////////////////////////////////////////////////////////
//                        Communications                        //

// Command id's                            
const unsigned char CMD_STOP = 'S';   
const unsigned char CMD_PLAY = 'P';
const unsigned char CMD_FORD = 'F';
const unsigned char CMD_BACK = 'B';
const unsigned char CMD_STAY = 'Y'; 
const unsigned char CMD_PLUS = 'I';
const unsigned char CMD_MINUS = 'O';
const unsigned char CMD_KP = 'T';
const unsigned char CMD_KI = 'E';
const unsigned char CMD_KD = 'D';  
const char CMD_END = '\r';   // Character end of frame \r is chosen for working with gtkterm correctly in Linux. If the arduino terminal is used, select the option "carriage return"                          

// Command receiver state
const int WAITING_CMD_ID = 1;
const int WAITING_END = 2;
int state = WAITING_CMD_ID; // Initial state

//Command manage
String inputString = "";        // A string to hold incoming data
int cmd;
bool cmd_ok = false;

bool play = false;
int constantSelector = 1; // 1->Kp, 2->Ki, 3->Kd

/////////////////////////////////////////////////////////////////


void setup() 
{
  Serial.begin(19200);

  /////////////////////////////////////////////////////////////////
  //                        BT communications                    //
  
  // Get and execute the commands
  while(play == false)
  {
    getCommands();
    Serial.println("Waiting start...");
    delay(100);
  }
  Serial.println("Bluethooh connected");

  /////////////////////////////////////////////////////////////////

  /////////////////////////////////////////////////////////////////
  //                      Initialize MPU6050                     //

  while(!mpu.begin(MPU6050_SCALE_250DPS, MPU6050_RANGE_2G))
  {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  // Calibrate gyroscope. The calibration must be at rest.
  // If you don't want calibrate, comment this line.
  mpu.calibrateGyro();
        
  for( i=0; i<10; i++){
      
    Vector normAccel = mpu.readNormalizeAccel();
    sum += (atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI; // Signe '-' in this operation has been deleted because of the position of the IMU
          
  }
        
  setZero = sum / i;

  /////////////////////////////////////////////////////////////////
    
  /////////////////////////////////////////////////////////////////
  //                         PID settings                        //
    
  Setpoint = 0;
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(15);
  myPID.SetMode(AUTOMATIC);
    
  /////////////////////////////////////////////////////////////////
     
  /////////////////////////////////////////////////////////////////
  //                         DC Motors                           //   
    
  digitalWrite(DRI, HIGH);
  motorWrite(M11, M12, 0, 0);
  motorWrite(M21, M22, 0, 0);
  
  /////////////////////////////////////////////////////////////////
    
}

void loop()
{
  if(play == true)
  {
    timer = millis();

    /////////////////////////////////////////////////////////////////
    //                Data adquisition and filtering               //

    // Read normalized values
    Vector normAccel = mpu.readNormalizeAccel();
    Vector norm = mpu.readNormalizeGyro();

    // Calculate angle
    apitch = (atan2(normAccel.XAxis, sqrt(normAccel.YAxis*normAccel.YAxis + normAccel.ZAxis*normAccel.ZAxis))*180.0)/M_PI; // Signe '-' in this operation has been deleted because of the position of the IMU
    gpitch = norm.YAxis * timeStep;

    fsignal = 0.98 * (fsignal + gpitch) + 0.02 * (apitch-setZero);

    /////////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////////////
    //                        Calculate PID                        //

    Input = fsignal;
    myPID.Compute();

    //Serial.print(Output);
    //Serial.print("\n");

    /////////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////////////
    //                           Write Output                      //

      if( Output < -10 && Input < 45)
      {
        digitalWrite(DRI, HIGH);
        motorWrite(M11, M12, 0, (int)-Output - offset);
        motorWrite(M21, M22, 1, (int)-Output);
      }
      else if ( Output > 10 && Input > -45)
      {
          digitalWrite(DRI, HIGH);
        motorWrite(M11, M12, 1, (int)Output - offset);
        motorWrite(M21, M22, 0, (int)Output);
      }
      else
      {
        digitalWrite(DRI, LOW);
      }

    /////////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////////////
    //                          Debug                              //

    //Serial.print(Input);
    //Serial.print(" ");
    //Serial.print((int)Output);
    //Serial.print("\n");
      
    /////////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////////////
    //                   Protocoder communications                 //
    getCommands();
    //Serial.println(play);
    /////////////////////////////////////////////////////////////////
    
    delay((timeStep*1000) - (millis() - timer)); //Sensor Time Step
  }
  else
  {
    /////////////////////////////////////////////////////////////////
    //                   Protocoder communications                 //
    getCommands();
    //Serial.println(play);
    /////////////////////////////////////////////////////////////////
    
  }  
}

/////////////////////////////////////////////////////////////////
//                          Functions                          //


void motorWrite(int P1, int P2, int dir, int vel)
{
  if(dir == 1)
  {
    digitalWrite(P1, LOW);
    analogWrite(P2, 255 - vel);
  }
  else 
  {
    digitalWrite(P2, HIGH);
    analogWrite(P1, vel);
  }
  
}

void changeConstant(bool add, int selector)
{
  if(add)
  {
    if(selector == 1)
      Kp += 5;
    else if(selector == 2)
      Ki += 1;
    else
      Kd += 1;
  }
  else
  {
    if(selector == 1)
      Kp -= 5;
    else if(selector == 2)
      Ki -= 1;
    else
      Kd -= 1;
  }
  if(Kp == 0)
  {
     motorWrite(M11, M12, 0, 0);
     motorWrite(M21, M22, 0, 0);
  }
  myPID.SetTunings(Kp, Ki, Kd);
    
}

void getCommands()
{
  //Serial.println("getCommands");
  
  // Read the incoming commands
  while (Serial.available())
  {
    // Read the received char
    char inChar = (char)Serial.read();
     
    // Depending on the state
    switch(state)
    {    
      // Read the cmd ID
      case WAITING_CMD_ID:
      
         //inputString = "";
         cmd = inChar;
           
         // The next state depends on the command id
         switch(cmd)
         {
           case CMD_STOP:
           case CMD_PLAY:
           case CMD_FORD:
           case CMD_BACK:
           case CMD_STAY:
           case CMD_PLUS:
           case CMD_MINUS:
           case CMD_KP:
           case CMD_KI:
           case CMD_KD:
             state = WAITING_END;
             //Serial.println("Waiting end");
             break;
                
           // If the command is unknow, move to the initial state  
           default:
             state = WAITING_CMD_ID;
             Serial.println("Wrong command");
         }
         break;  
         
       // Read the end of the frame
       case WAITING_END:
         
         // The next state is the initial, in any case
         state = WAITING_CMD_ID;
         
         // End of frame received correctly
         if (inChar == CMD_END) {
           cmd_ok = true;             // The command is ok!
           Serial.println("Command received");
           runCommand();
         }  
         else {
           cmd_ok = false;   // Invalid frame. It will be ignored
         }
         break;
          
    } // End switch
      
  } // End While
    
}

void runCommand()
{
  // If the frame received is ok... process!
  if (cmd_ok)
  {
    //Serial.println("OK");
    switch(cmd)
    {  
      // Command play
      case CMD_PLAY:
        play = true;
        break;
        
      // Command stop
      case CMD_STOP:
        play = false;
        motorWrite(M11, M12, 0, 0);
        motorWrite(M21, M22, 0, 0);
        break;

      // Command move fordward Setpoint
      case CMD_FORD:
        Setpoint += 0.05;
        break;

      // Command move backward Setpoint
      case CMD_BACK:
        Setpoint -= 0.05;
        break;

      // Command set Setpoint to 0
      case CMD_STAY:
        Setpoint = 0;
        break;

      // Command add to a selected constant
      case CMD_PLUS:
        changeConstant(1, constantSelector);
        break;
        
      // Command extract to a selected constant
      case CMD_MINUS:
        changeConstant(0, constantSelector);
        break;

      // Command select kp
      case CMD_KP:
        constantSelector = 1;
        break;

      // Command select ki
      case CMD_KI:
        constantSelector = 2;
        break;

      // Command select kd
      case CMD_KD:
        constantSelector = 3;
        break;

    }
    
    cmd_ok = false;
  }  
}
/////////////////////////////////////////////////////////////////



