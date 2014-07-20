//Define variables
int led1 = 48;                  //pin 48 = Green
int led2 = 50;                  //pin 50 = Yellow
int led3 = 52;                  //pin 52 = Red

void Prime_LEDs(){              // initialize the digital pin as an output for the LEDs   
    pinMode(led3, OUTPUT);  
    pinMode(led2, OUTPUT);  
    pinMode(led1, OUTPUT);    
}

void Write_LEDs(char LED[]){    //LED[] is a 3 binary digit array to control each LED
  if(LED[0] == '1'){              //LED[0] controls the green LED, if 1
    digitalWrite(led1, HIGH);       //turn green LED on (HIGH is the voltage level)
  }else{                          //else
    digitalWrite(led1, LOW);        //turn green LED off (LOW is the voltage level)
  }
  
  if(LED[1] == '1'){              //LED[1] controls the yellow LED, if 1
    digitalWrite(led2, HIGH);       //turn yellow LED on (HIGH is the voltage level)
  }else{                          //else
    digitalWrite(led2, LOW);        //turn yellow LED off (LOW is the voltage level)
  }
  
  if(LED[2] == '1'){              //LED[2] controls the red LED, if 1
    digitalWrite(led3, HIGH);       //turn red LED on (HIGH is the voltage level)
  }else{                          //else
    digitalWrite(led3, LOW);        //turn red LED off (LOW is the voltage level)
  }
}
