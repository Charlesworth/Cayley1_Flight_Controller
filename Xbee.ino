void Read_Xbee() {
  if(Serial.available()){  //is there anything to read on the serial RX0 pin?
	char getData = Serial.read();  //if yes, write the char to getData

        switch (getData){  //using switch statements map key presses to actions
        case 'w':             //if W
          control = 0;          //control int = 0
        break;
        case 'a':             //if A
  	  control = 1;          //control int = 1
        break;
        case 's':             //if S
          control = 2;          //control int = 2
        break;
        case 'd':             //if D
          control = 3;          //control int = 3
        break;
        case 'q':             //if Q
          control = 4;          //control int = 4
        break;
        case 'e':             //if E
          control = 5;          //control int = 5
        break;
        case 'r':             //if R
          control = 6;          //control int = 6
        break;
        case 'f':             //if F
          control = 7;          //control int = 7
        break;
        case 'i':             //if I
          control = 8;          //control int = 8
          Write_Pan_Tilt_Range();//move the Pan/Tilt Mount
        break;
        case 'j':             //if J
          control = 9;          //control int = 9
          Write_Pan_Tilt_Range();//move the Pan/Tilt Mount
        break;
        case 'k':             //if K
          control = 10;          //control int = 10
          Write_Pan_Tilt_Range();//move the Pan/Tilt Mount
        break;
        case 'l':             //if L
          control = 11;          //control int = 11
          Write_Pan_Tilt_Range();//move the Pan/Tilt Mount
        break;
        case '0':             //if 0
          Serial.println("*************POWER DOWN**************");
          //PID_Reset();
  	  IO = 0;              //int I0 = 0, signaling power down
          Instructions();       //control instructions are printed on serial
        break;
        case '1':             //if 1
          setup_IMU();          //Initialize and Calibrate the IMU
          IO = 1;               //int I0 = 1, signaling power on
        break;
        default:              //if command not recognised
          Serial.println("*********CMD NOT RECOGNISED**********");
        }
  }
}

void Instructions(){ //control instructions printed over serial during 0()
  Serial.println("Commands:");
  Serial.println("1/0 -> PWR ON/OFF");
  Serial.println("W/S -> Move Forward/Back");
  Serial.println("A/D -> Move Left/Right");
  Serial.println("Q/E -> Turn Left/Right");
  Serial.println("R/F -> Move Up/Down");
  Serial.println("I/J/L -> Range Forward/Left/Right");
  Serial.println("K -> Altitude");
}