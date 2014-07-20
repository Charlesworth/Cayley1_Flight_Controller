/*Master Control Program (MCP)
Written by Charles Cochrane
For use with Cayley 1 quad rotor aircraft*/

//Include libraries
#include <Wire.h>
#include <Servo.h>
#include <LSM303.h>
#include <L3G.h>

//Define global variables
double RawPitch, RawRoll, RawYaw;//roll = x, pitch = y, yaw = z
double OutputRoll, OutputPitch;//The PID output variables
int control;    //Communication integer for Xbee data output
int IO = 0;     //Flag for indicating aircraft PWR state
int SerialCount = 0;//Counter to keep Xbee comms at 2Hz

void setup(){  //The Setup function, run once at PWR ON of arduino
    Prime_LEDs();      //Initiate LEDs to accept Write_LEDs() command
    Write_LEDs("011"); //Light the yellow and red LEDs
    Prime_Pan_Tilt_Range();//Initaite the range finder to output range
    
    Serial.begin(9600);//Begin serial comms, define the Baud rate as 9600
    Serial.println("***********Cayley 1***********");
    Serial.println("**Autonimous Quadrotor Robot**");
    
    Prime_Motors();    //Train the ESCs for use with the motors  
    Wire.begin();      //Initaite the wire library for I2C comms
    IO = 0;            //Start quad rotor in powered down state
    
    Serial.println("*************Ready*************");
    Instructions();    //Print user flight operation instructions
}

void loop(){  //Function will loop continously until Arduino PWR down
    Read_Xbee();  //Read the Xbee and output user inputs
    
    if(IO == 0){  //If in PWR down mode
        O();          //execute function 0()
    }
    if(IO == 1){  //If in PWR on mode
        Write_LEDs("100");//Turn green LED on
        I();          //execute function I()
    }
}

void O(){    //PWR down mode
    Write_LEDs("001");//Turn on red LED
    Off_Motors();   //keep propellers motionless
}

void I(){    //PWR on mode
    while(SerialCount < 51){//run loop 50 time
        loop_IMU();        //Read Roll Pitch and Yaw from IMU
        
        PID_Compute_Roll();//PID stablisation on X axis
        PID_Compute_Pitch();//PID stablisation on Y axis
        Write_Motors();    //Write PID and user values to ESCs
        
        SerialCount++;     //Increment SerialCount
    } 
    SerialCount = 0;  //When loop ends reset SerialCount = 0
  
    //Output Range-finder range over serial
    Read_Pan_Tilt_Range();
    
    //Output orientation data over serial
    Serial.print("roll = ");
    Serial.print(RawRoll);
    Serial.print(", pitch = ");
    Serial.print(RawPitch);
    Serial.print(", yaw = ");
    Serial.println(RawYaw);     
}
