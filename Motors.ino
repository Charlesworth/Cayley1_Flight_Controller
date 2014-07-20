//Define variables
Servo motor1, motor2, motor3, motor4;
int rotspeed = 0;
int Train_Count = 11;
int motor1speed, motor2speed, motor3speed, motor4speed;
int UserInput[3];

void Prime_Motors(){  //Used to train ESCs and leave them in a ready but non-moving state
    motor1.attach(40);      //Define the Arduino outputs
    motor2.attach(38);      //which the motors are 
    motor3.attach(34);      //attached to
    motor4.attach(36); 
  
    while (Train_Count > 1){//Loop 10 times to train a 0 servo signal to the ESCs
        motor1.write(0);          //output a servo
        motor2.write(0);          //signal of 0
        motor3.write(0);          //to the motors
        motor4.write(0);
        Train_Count--;            //decrement the counter
        delay(1000);              //wait for 1 second
    };
    
    while (rotspeed <= 55){ //while the motors servo signal is smaller than 55
        rotspeed = rotspeed + 5;  //increment the speed by 5
        motor1.write(rotspeed);   //write the new speed to the motors
        motor2.write(rotspeed);
        motor3.write(rotspeed);
        motor4.write(rotspeed);
        delay(1000);              //wait for 1 second
    };
    //the motors are now trained and ready
} 

void Off_Motors(){ //Write servo value of 55, giving no propeller spin
    motor1.write(55);
    motor2.write(55);
    motor3.write(55);
    motor4.write(55);
}

void Write_Motors(){ /*here we take the IMU roll, pitch and yaw values 
  and the user input movement and produce the motor output*/
  
    //Please Note, hover occurs at a servo input of 72
    if(SerialCount == 1){//input the user controls at a 2Hz rate
        switch (control){    //use switch statements to determine input
            case 0:             //if W
              UserInput[0] = -2;
              UserInput[2] = 2;
            break;
            case 1:             //if A
              UserInput[1] = 2;
              UserInput[3] = -2;
            break;
            case 2:             //if S
              UserInput[0] = 2;
              UserInput[2] = -2;
            break;
            case 3:             //if D
              UserInput[1] = -2;
              UserInput[3] = 2;
            break;
            case 4:             //if Q
              UserInput[0] = -2;
              UserInput[1] = 2;
              UserInput[2] = -2;
              UserInput[3] = 2;
            break; 
            case 5:             //if E
              UserInput[0] = 2;
              UserInput[1] = -2;
              UserInput[2] = 2;
              UserInput[3] = -2;
            break;
            case 6:             //if R
              UserInput[0] = 4;
              UserInput[1] = 4;
              UserInput[2] = 4;
              UserInput[3] = 4;
            break;
            case 7:             //if F
              UserInput[0] = -4;
              UserInput[1] = -4;
              UserInput[2] = -4;
              UserInput[3] = -4;
            break;       
        }
    }
  
    //combine hover input with the user and PID inputs for each motor
    motor1speed = 72 + UserInput[0] + OutputPitch;
    motor2speed = 72 + UserInput[1] + OutputRoll;
    motor3speed = 72 + UserInput[2] - OutputPitch; 
    motor4speed = 72 + UserInput[3] - OutputRoll;
    
    //keeps the 4 motor inputs above 65 so as they always spin during I()
    if (motor1speed < 65){motor1speed = 65;}
    if (motor2speed < 65){motor2speed = 65;}
    if (motor3speed < 65){motor3speed = 65;}
    if (motor4speed < 65){motor4speed = 65;}    
    
    //write the combined servo inputs to the 4 ESCs  
    motor1.write(motor1speed);
    motor2.write(motor2speed);
    motor3.write(motor3speed);
    motor4.write(motor4speed);
  
    //if the user inputs have been added, clear their values
    if(SerialCount == 1){
          UserInput[0] = 0;
          UserInput[1] = 0;
          UserInput[2] = 0;
          UserInput[3] = 0;
    }
}
