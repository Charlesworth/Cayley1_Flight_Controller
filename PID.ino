//Define variables
unsigned long lastTimePitch, lastTimeRoll;
double Input, Setpoint;
double errSumPitch, lastErrPitch, errSumRoll, lastErrRoll;
double kp = 0.0;    //Kp gain co-efficient
double ki = 0.0;    //Ki gain co-efficient
double kd = 0.0;    //Kd gain co-efficient

void PID_Compute_Roll(){//PID stablisation for X axis
     Input = RawRoll;      //System input = Roll
     Setpoint = 0;         //Reference = 0 degrees
     
     
     unsigned long now = millis();//Time now
     
     //How long since we last calculated
     double timeChange = (double)(now - lastTimeRoll);
    
     double error = Setpoint - Input;//Compute error
     errSumRoll += (error * timeChange);//Compute intergration
     double dErr = (error - lastErrRoll) / timeChange * 50;//compute differential
    
     OutputRoll = kp * error + ki * errSumRoll + kd * dErr;//Compute PID Output
     
     //set the output upper and lower limits of the system output
     if(OutputRoll > 5){
         OutputRoll = 5;
     }
     else if(OutputRoll < -5){ 
         OutputRoll = -5;
     }
  
     lastErrRoll = error; //Store the current error for next PID loop
     lastTimeRoll = now;  //Store the current time for next PID loop
} 

void PID_Compute_Pitch(){
     Input = RawPitch;      //System input = Pitch
     Setpoint = 0;         //Reference = 0 degrees
     
     unsigned long now = millis();//Time now
     
     //How long since we last calculated
     double timeChange = (double)(now - lastTimePitch);
  
     double error = Setpoint - Input;//Compute error
     errSumPitch += (error * timeChange);//Compute intergration
     double dErr = (error - lastErrPitch) / timeChange;//compute differential
    
     OutputPitch = kp * error + ki * errSumPitch + kd * dErr;//Compute PID Output
     
     //set the output upper and lower limits of the system
     if(OutputPitch > 5){
         OutputPitch = 5;
     }
     else if(OutputPitch < -5){ 
         OutputPitch = -5;
     }
  
    
     lastErrPitch = error; //Store the current error for next PID loop
     lastTimePitch = now;  //Store the current time for next PID loop
} 
  
void PID_Reset(){//reset all of the variables for next PID initialization
     lastErrRoll = 0;
     lastTimeRoll = 0;
     errSumRoll = 0;
     lastErrPitch = 0;
     lastTimePitch = 0;
     errSumPitch = 0;
}
