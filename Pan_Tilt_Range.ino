int sensorPin = 0;         //analog pin 0 for range finder input
int Pan, Tilt;             //Pan and Tilt servo position values
Servo PanServo, TiltServo; //define as servos

void Prime_Pan_Tilt_Range(){ 
    PanServo.attach(22);     //attach a servo to digital pin 22
    TiltServo.attach(24);    //attach a servo to digital pin 24
    int Pan = 90;            //set the initial value to 90, centering the mount
    int Tilt = 125;          //max value = 125 (downwards facing)
    PanServo.write(Pan);     //write 90 to the pan servo
    TiltServo.write(Tilt);   //write 125 to tilt servo
} 

void Write_Pan_Tilt_Range(){
    switch (control){        //using switch to match control to each case
       case 8:                 //when control int = 8
         Tilt = 0;               //mount is facing forward
         Pan = 90;
       break;
       case 9:                //when control int = 9    
         Tilt = 0;               //mount is facing leftward
         Pan = 0;  
       break;
       case 10:               //when control int = 10   
         Tilt = 125;             //mount is facing downwards (altitude)
         Pan = 90;               //mount pan is centered
       break;
       case 11:               //when control int = 11    
         Tilt = 0;               //mount is facing rightward
         Pan = 180;
       break;
       default:               //if command is not recognised by a case 
       break;                 //don't change position
    }
    PanServo.write(Pan);   //write the resultant pan/tilt
    TiltServo.write(Tilt); //values to the servos
} 

void Read_Pan_Tilt_Range(){ 
    int Distance = analogRead(sensorPin); //min 125, max 300
    Distance = Distance * 2.59;          //converts to cm
    switch (Pan){                        //use switch to select serial output
         case 180:                           //if pan is facing right
           Serial.print("Right Dist = ");      //output the Right Distance
           Serial.println(Distance);
         break;
         case 0:                             //if pan is facing left
           Serial.print("Left Dist = ");       //output left distance
           Serial.println(Distance);  
         break;
         case 90:                            //if pan is centered
           if(Tilt == 125){                    //if camera is downward facing
               Serial.print("Altitude = ");      //output altitude
               Serial.println(Distance);
           }else{                              //else
               Serial.print("Forward Dist = ");  //output forward distance
               Serial.println(Distance);
           }
         break;                              
         default:                            //if Pan is not recognised   
         break;                              //output nothing
    }
}
