#include <LiquidCrystal.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Math.h>


#define F_CPU 16000000
#define BAUD 9600
#define BRC ((F_CPU/16/BAUD) -1)
#define TX_BUFFER_SIZE 128
#define RX_BUFFER_SIZE 128


char rxBuffer[RX_BUFFER_SIZE];
uint8_t rxReadPos = 0;
uint8_t rxWritePos =0;
char txBuffer[TX_BUFFER_SIZE];
uint8_t serialReadPos =0;
uint8_t serialWritePos = 0;




LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

double incomingByte;
volatile unsigned long int milliseconds = 0;


/** Button inputs **/
enum Buttons {
  btnNONE,
  btnRIGHT,
  btnUP,
  btnDOWN,
  btnLEFT,
  btnSELECT
};
Buttons whatbuttons; // Current button pressed


enum Mode {
  mainMODE,
  controlMODE,
  sweepMODE,
  wallMODE,
  navMODE
};
Mode currentMode = mainMODE;

enum mainMenuState {
  mainState,
  controlState,
  sweepState,
  wallState,
  navState
};
mainMenuState currentMenuState;



boolean startup = true;

ISR(TIMER2_OVF_vect) { //Chapter 16
  //Register size = 64
  // CLK = 62500 Hz
  //Timer pertick = 1/CLK = 0.016ms
  // from 0 to 64 = 64 * 0.016ms = 1ms

  milliseconds += 1; //increment every ms

}



void setup() {
  // put your setup code here, to run once:
  timer_Init();
  sei(); //enable global interrupt

  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  lcd.print("12878930");
  lcd.setCursor(0,1);
  lcd.print("Main Menu");
  //serial_Init();
  Serial.begin(9600);
  Serial.setTimeout(300);
  
  ADC_Init();  

  PrintMessage("CMD_START");
  
}



void loop() {
  switch(currentMode) {
    case mainMODE: 
      mainModeOperation();
      break;
    case controlMODE:
      controlModeOperation();
      break;
    case sweepMODE:
      sweepModeOperation();
      break;
    case wallMODE:
      wallModeOperation();
      break;
    case navMODE:
      navModeOperation();
      break;
  }

  whatbuttons = readLCDButtons();
}


void PrintMessage(String message)
{
  Serial.print(message);
  Serial.write(13); //carriage return character (ASCII 13, or '\r')
  Serial.write(10); //newline character (ASCII 10, or '\n')
  mydelay(40);
}


void timer_Init() {
  //Timer2 register A = normal operation
  TCCR2A &=~(1<<WGM20);
  TCCR2A &=~(1<<WGM21);
  TCCR2A &=~(1<<COM2B0);
  TCCR2A &=~(1<<COM2B1);
  TCCR2A &=~(1<<COM2A0);
  TCCR2A &=~(1<<COM2A1);

  // Prescaler for 64
  TCCR2B &= ~(1<<CS20);
  TCCR2B &= ~(1<<CS21);
  TCCR2B |= (1<<CS22);

  TIMSK2 |= (1<<TOIE2); //Enable Overflow interrupt
  TIFR2 |= (1<<TOV2);
}

volatile unsigned long int mymillis() {
  return milliseconds;
}

void mydelay(volatile long unsigned int delayTime) {
 
  volatile long unsigned int count = mymillis();
  while(mymillis() <= (delayTime + count)) {

  }
}


Buttons readLCDButtons() {
  static int inputButton; 
  inputButton = myAnalogRead(0);
  mydelay(150); //DEBOUNCE
  // read analog 0 with registers
  if(inputButton > 1000) {
    return btnNONE;
  }
  if(inputButton < 50) {
    return btnRIGHT;
  }
  if(inputButton < 250) {
    return btnUP;
  }
  if(inputButton < 450) {
    return btnDOWN;
  }
  if(inputButton < 650) {
    return btnLEFT;
  }
  if(inputButton < 850) {
    return btnSELECT;
  }

  return btnNONE; // when all others fail, return this
}


void ADC_Init() {
  ADCSRA |= (1<<ADEN); // Enable ADC
  ADMUX |= (1<<REFS0); // Internal Vcc 5v
}

int myAnalogRead(int Pin) 
{
  if(Pin == 1) {
    ADMUX |= Pin; //Multiplexer  for which pin to read from
  }
  else if(Pin == 0) {
    ADMUX = 0;
    ADMUX |= (1<<REFS0); // Internal Vcc 5v 
  }
  ADCSRA |= (1<<ADSC); // start conversion
  // wait for conversion to complete
  while (!(ADCSRA &(1<<ADIF))); // becomes while(0) when the conversion is complete
  ADCSRA |= (1<<ADIF);
  return ADC; 
}


void mainModeOperation() {
  if(startup) {
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("12878930");
    lcd.setCursor(0,1);
    stateToText();
    startup = false;
  }


  switch(whatbuttons) {
    case btnDOWN: 
      if(currentMenuState == navState) {
        currentMenuState = mainState;
      }
      else {
        currentMenuState = currentMenuState + 1;
      }
      startup = true;
      break;

    case btnSELECT:
      stateToMode();
      startup = true;
      break;
      
  }
}


void stateToMode() {
  switch(currentMenuState) {
    case mainState:
      break;
    case controlState:
      currentMode = controlMODE;
      break;
    case sweepState:
      currentMode = sweepMODE;
      break;
    case wallState:
      currentMode = wallMODE;
      break;
    case navState:
      currentMode = navMODE;
      break;
  }
  
}


void stateToText() {
  switch(currentMenuState) {
    case mainState:
      lcd.print("Main Menu");
      break;
    case controlState:
      lcd.print("Control");
      break;
    case sweepState:
      lcd.print("Sweep");
      break;
    case wallState:
      lcd.print("Wall follow");
      break;
    case navState:
      lcd.print("Navigation");
      break;
  } 
}
void controlModeOperation() {
  mydelay(100);
  switch(whatbuttons) {
    case btnLEFT:
      PrintMessage("CMD_ACT_ROT_0_10");
      break;
    case btnRIGHT:
      PrintMessage("CMD_ACT_ROT_1_10");
      break;
    case btnUP:
      PrintMessage("CMD_ACT_LAT_1_0.5\r\n");
      break;
    case btnDOWN:
      PrintMessage("CMD_ACT_LAT_0_0.5\r\n");
      break;
    case btnSELECT:
      currentMode = mainMODE;
      currentMenuState = mainState;
      startup = true;
      break;
  }
}

void sweepModeOperation() {
  static boolean finished = false;
  int minimumAngle = 0;
  float minimum;
  String currentString;
  static float currentValue = 0;
  float irValue;
  int i = 0;
  int cutString;
  if(!finished) {
    if(whatbuttons == btnUP) {
      PrintMessage("CMD_SEN_ROT_0");
      for(i = 0; i <= 72;i++ ) {
        PrintMessage("CMD_SEN_ROT_" + (String) (360 - (i*5)));
        PrintMessage("CMD_SEN_IR");
        mydelay(30);
        currentString = Serial.readString();
        cutString = currentString.length();
        currentString.remove(cutString-2);
        irValue = currentString.toFloat();
        mydelay(20);
        if(irValue != irValue) { // if NaN
          irValue = 5; // more than sensor range
        }
        if((irValue <= minimum) ||(i == 0)) {
          minimumAngle = i*5;
          minimum = irValue;
          lcd.clear();
          lcd.print(minimum);
        }
        
      }
      finished = true;
      minimumAngle = 359 - minimumAngle;
      for(int j = 0; j <= minimumAngle; j++) {
        PrintMessage("CMD_ACT_ROT_0_1");
      }
      PrintMessage("CMD_SEN_ROT_0");
    }
    else  if(whatbuttons == btnSELECT) {
      startup = true;
      finished = false;
      currentMode = mainMODE;
      currentMenuState = mainState;
    }

  }
  else {
    if(whatbuttons == btnSELECT) {
      startup = true;
      finished = false;
      currentMode = mainMODE;
      currentMenuState = mainState;
    }
  }
  
}

void wallModeOperation() {
  static boolean finished = false;
  static int minimumAngle = 0;
  static float minimum;
  String currentString;
  static float currentValue = 0;
  static float irValue;
  static float d1, d2, d1d2, dwall, dfront;
  static float alpha = 15;
  static int angle = 0;
  int cutString;
  static float error, oldError;
  static float anglePosition = 0;
  static boolean stopped = false;
  static boolean firstCal = true;
  static boolean badReading = false;
  if(!stopped) {
    if(!finished) {
      PrintMessage("CMD_SEN_ROT_" + (String) (360 - (angle*5)));
      PrintMessage("CMD_SEN_IR");
      currentString = Serial.readString();
      cutString = currentString.length();
      currentString.remove(cutString-2);
      irValue = currentString.toFloat();
      
      if(irValue != irValue) { // if NaN
        irValue = 5; // more than sensor range
        badReading = false;
      }
      else if(irValue == 0) {
        badReading = true;
      }
      else {
        badReading = false;
      }
      
      if((irValue <= minimum) ||(angle == 0)) {
        minimumAngle = angle*5;
        minimum = irValue;
      }
      if(angle == 72) {
        minimumAngle = 359 - minimumAngle;
        for(int j = 0; j <= (minimumAngle) ; j++) {
          PrintMessage("CMD_ACT_ROT_0_1");
          mydelay(40);
        }
        mydelay(200);
        PrintMessage("CMD_ACT_ROT_1_90");
        anglePosition = 0;
        finished = true;
      }
      if(!badReading) {
        angle++;
      }
    }
    else {
        PrintMessage("CMD_SEN_ROT_90");
        mydelay(20);
        PrintMessage("CMD_SEN_IR");
        mydelay(20);
        currentString = Serial.readString();
        cutString = currentString.length();
        currentString.remove(cutString-2);     
        d1 = currentString.toFloat();
        PrintMessage("CMD_SEN_ROT_75");
        mydelay(20);
        PrintMessage("CMD_SEN_IR");
        mydelay(20);
        currentString = Serial.readString();
        cutString = currentString.length();
        currentString.remove(cutString-2);
        d2 = currentString.toFloat();
        if((d1 == d1) && (d2 == d2) && ((d1 != 0) || (d2 != 0))) {
          d1d2 = sqrt(d1*d1+ d2*d2 - 2*d1*d2*cos((alpha*3.14)/180));
          //angleError = (asin(d1*sin(30*3.14/180)/d1d2))*180/3.14;
          dwall = d2*d1*sin((alpha*3.14)/180)/d1d2; 
          if(dwall != 0) {
            if(dwall > 2) {
              error = dwall - 2; 
              if((error <= 0.15)) {
                if(anglePosition < -8 && (error <= oldError)) {
                  PrintMessage("CMD_ACT_ROT_1_" + (String) (error*14));
                  anglePosition = anglePosition + error*18;              
                }
                else if((anglePosition > 0) || (oldError <= error)) {
                  PrintMessage("CMD_ACT_ROT_0_" + (String) (error*10));
                  anglePosition = anglePosition - error*8;              
                }
    
              }
              else if(anglePosition > -14) {
                PrintMessage("CMD_ACT_ROT_0_" + (String) (error*8));
                anglePosition = anglePosition - error*12;;          
              }
            }
            else {
              error = 2 -  dwall;
              if (error <= 0.15){
                if(anglePosition > 8 && (error <= oldError)) {
                  PrintMessage("CMD_ACT_ROT_0_" + (String) (error*14));
                  anglePosition = anglePosition - error*18;              
                }
                else if(anglePosition < 0 || (oldError <= error)) {
                  PrintMessage("CMD_ACT_ROT_1_" + (String) (error*10));
                  anglePosition = anglePosition + error*8;               
                }
    
              }
              else if(anglePosition < 14) {
                PrintMessage("CMD_ACT_ROT_1_" + (String) (error*8));
                anglePosition = anglePosition + error*12;          
              }
            }   
          }
//          lcd.clear();
//          lcd.setCursor(0,0);
//          lcd.print(d1);
//          lcd.print(" ");
//          lcd.print(d2);
//          lcd.print(" ");
//          lcd.print(anglePosition);
//          lcd.setCursor(0,1);
//          lcd.print(dwall);
          PrintMessage("CMD_SEN_ROT_0");
          PrintMessage("CMD_SEN_IR");
          currentString = Serial.readString();
          cutString = currentString.length();
          currentString.remove(cutString-2);     
          dfront = currentString.toFloat();
          if((dfront == dfront) && (dfront != 0)) {
            if(dfront <= 2.4) {
              PrintMessage("CMD_ACT_ROT_1_90");
            } 
          }
          PrintMessage("CMD_ACT_LAT_1_0.5");
          oldError = error;      
        }
    }
  }
  switch(whatbuttons) {
    case btnUP:
      firstCal = true;
      stopped = true;
      minimumAngle = minimum = anglePosition = angle = 0;
      break;
    case btnSELECT:
      firstCal = true;
      stopped = false;
      PrintMessage("CMD_SEN_ROT_0");
      finished = false;
      minimumAngle = minimum = anglePosition = angle = 0;
      currentMode = mainMODE;
      currentMenuState = mainState;
      startup = true;
  }


  
}

void navModeOperation() {
  float leftCorner, rightCorner;
  float leftAngle, rightAngle;
  float oldLeft, oldRight;
  float maxLeft, maxRight;
  float leftDistance, rightDistance;
  float frontDistance;
  String currentString;
  int cutString;
  static boolean foundGoal = false;
  static int reachGoal = 0;
  int collisions;
  static float distanceGoal;
  static float A,B,C;
  static float angleGoal;
  boolean wallWarning = false;
  static float newDistanceGoal;


  if(!foundGoal) {
    PrintMessage("CMD_SEN_PING");
    currentString = Serial.readString();
    cutString = currentString.length();
    currentString.remove(cutString-2);     
    distanceGoal = currentString.toFloat();
    if(distanceGoal != 0) {
      foundGoal = true;
    }
    PrintMessage("CMD_SEN_COLL");
//    mydelay(50);
//    currentString = Serial.readString();
//    cutString = currentString.length();
//    currentString.remove(cutString-2);     
//    collisions = currentString.toFloat();
    lcd.clear();
    lcd.print(distanceGoal);
    PrintMessage("CMD_SEN_ROT_0");
    mydelay(30);
    PrintMessage("CMD_SEN_IR");
    mydelay(50);
    currentString = Serial.readString();
    cutString = currentString.length();
    currentString.remove(cutString-2);     
    frontDistance = currentString.toFloat();
    if(frontDistance != frontDistance) {
      PrintMessage("CMD_SEN_ROT_20");
      PrintMessage("CMD_SEN_IR");
      currentString = Serial.readString();
      cutString = currentString.length();
      currentString.remove(cutString-2);     
      leftCorner = currentString.toFloat();
  
      PrintMessage("CMD_SEN_ROT_340");
      PrintMessage("CMD_SEN_IR");
      currentString = Serial.readString();
      cutString = currentString.length();
      currentString.remove(cutString-2);     
      rightCorner = currentString.toFloat();
        if((rightCorner != rightCorner) && (leftCorner != leftCorner)) {
          PrintMessage("CMD_ACT_LAT_1_2");
        }
        else if((rightCorner != rightCorner) && (leftCorner == leftCorner)) {
          if(leftCorner < 2) {
            PrintMessage("CMD_ACT_ROT_1_10");
          }
          PrintMessage("CMD_ACT_LAT_1_1");
        }
        else if((rightCorner == rightCorner) && (leftCorner != leftCorner)) {
          if(rightCorner < 2) {
            PrintMessage("CMD_ACT_ROT_0_10");
          }
          PrintMessage("CMD_ACT_LAT_1_1");
        }
        else if((rightCorner > 0.75) && (leftCorner > 0.75)) {
          PrintMessage("CMD_ACT_LAT_1_1");
        }
        else if((rightCorner > 0.75) && (leftCorner < 0.75)) {
          PrintMessage("CMD_ACT_ROT_1_90");
        }
        else if((rightCorner < 0.75) && (leftCorner > 0.75)) {
          PrintMessage("CMD_ACT_ROT_0_90");
        }
      
    }
    else {
      if(frontDistance >= 2) {
        PrintMessage("CMD_SEN_ROT_20");
        PrintMessage("CMD_SEN_IR");
        currentString = Serial.readString();
        cutString = currentString.length();
        currentString.remove(cutString-2);     
        leftCorner = currentString.toFloat();

        PrintMessage("CMD_SEN_ROT_340");
        PrintMessage("CMD_SEN_IR");
        currentString = Serial.readString();
        cutString = currentString.length();
        currentString.remove(cutString-2);     
        rightCorner = currentString.toFloat();
        if((rightCorner != rightCorner) && (leftCorner != leftCorner)) {
          PrintMessage("CMD_ACT_LAT_1_" + (String)(frontDistance - 1.25));
        }
        else if((rightCorner != rightCorner) && (leftCorner == leftCorner)) {
          if(leftCorner <2) {
            PrintMessage("CMD_ACT_ROT_1_10");
          }
          PrintMessage("CMD_ACT_LAT_1_" + (String)(frontDistance - 1.25));
        }
        else if((rightCorner == rightCorner) && (leftCorner != leftCorner)) {
          if(rightCorner < 2) {
            PrintMessage("CMD_ACT_ROT_0_10");
          }
          
          PrintMessage("CMD_ACT_LAT_1_" + (String)(frontDistance - 1.25));
        }
        else if((rightCorner > 0.75) && (leftCorner > 0.75)) {
          PrintMessage("CMD_ACT_LAT_1_" + (String)(frontDistance - 1.25));
        }
        else if((rightCorner > 0.75) && (leftCorner < 0.75)) {
          PrintMessage("CMD_ACT_ROT_1_30");
        }
        else if((rightCorner < 0.75) && (leftCorner > 0.75)) {
          PrintMessage("CMD_ACT_ROT_0_30");
        }
      }
      else {
        for(int i = 1; i < 9; i++) {
          PrintMessage("CMD_SEN_ROT_" + (String) (i*10));
          mydelay(30);
          PrintMessage("CMD_SEN_IR");
          mydelay(30);
          currentString = Serial.readString();
          cutString = currentString.length();
          currentString.remove(cutString-2);     
          leftDistance = currentString.toFloat(); 
 

          if(leftDistance != leftDistance) {
//            PrintMessage("CMD_ACT_ROT_0_" + (String) (i*10));
//            PrintMessage("CMD_ACT_LAT_1_2");
            leftAngle = i*10;
            i = 8;
          }
          else {
            if(leftDistance > oldLeft) {
              maxLeft = leftDistance;
              leftAngle = i*10;
            }
            oldLeft = leftDistance;
          }
          
        }

        for(int j = 1; j < 9; j++) {

            PrintMessage("CMD_SEN_ROT_" + (String) (360-(j*10)));
            mydelay(30);
            PrintMessage("CMD_SEN_IR");
            mydelay(30);
            currentString = Serial.readString();
            cutString = currentString.length();
            currentString.remove(cutString-2);     
            rightDistance = currentString.toFloat(); 

          if(rightDistance != rightDistance) {
//            PrintMessage("CMD_ACT_ROT_1_" + (String) (j*10));
//            PrintMessage("CMD_ACT_LAT_1_2");
            rightAngle = j*10;
            j = 8;
          }
          else {
            if(rightDistance > oldRight) {
              maxRight = rightDistance;
              rightAngle = j*10;
            }
            oldRight = rightDistance;
          }
        }

        if((rightDistance == rightDistance) && (leftDistance == leftDistance) && (leftDistance != 0) && (rightDistance != 0)) {
          if((frontDistance > maxRight) && (frontDistance > maxLeft) && (frontDistance > 1)) {
            PrintMessage("CMD_ACT_LAT_1_" + (String) (frontDistance -1));
            mydelay(30);
          }
          if(maxRight > maxLeft) {
            if(maxRight < 1.75) {
              PrintMessage("CMD_ACT_ROT_1_180");
              mydelay(30);
            }
            else {
              PrintMessage("CMD_ACT_ROT_1_" + (String) rightAngle);
              mydelay(50);
              PrintMessage("CMD_ACT_LAT_1_" + (String) (maxRight -1));
              mydelay(50);
            }
            
          }
          else {
            if(maxLeft < 1.75) {
              PrintMessage("CMD_ACT_ROT_1_180");
              mydelay(30);
            }
            else {
              PrintMessage("CMD_ACT_ROT_0_" + (String) leftAngle);
              mydelay(50);
              PrintMessage("CMD_ACT_LAT_1_" + (String) (maxLeft -1));
              mydelay(50);
            }
          }
        }
        else if((rightDistance != rightDistance) || (leftDistance != leftDistance)){
          if(rightAngle > leftAngle) {
            PrintMessage("CMD_ACT_ROT_0_" + (String) (leftAngle));
          }
          else  {
            PrintMessage("CMD_ACT_ROT_1_" + (String) (rightAngle));
          }
        }
      }
    }   
  }
  else {
    lcd.clear();
    lcd.print("within 5m");
    lcd.print(reachGoal);
    switch(reachGoal) {
      case 0:
        A = distanceGoal;
        PrintMessage("CMD_SEN_ROT_0");
        mydelay(30);
        PrintMessage("CMD_SEN_IR");
        mydelay(50);
        currentString = Serial.readString();
        cutString = currentString.length();
        currentString.remove(cutString-2);     
        frontDistance = currentString.toFloat();
        if(frontDistance != frontDistance || (frontDistance > 1)) {
          PrintMessage("CMD_SEN_ROT_20");
          mydelay(30);
          PrintMessage("CMD_SEN_IR");
          mydelay(30);
          currentString = Serial.readString();
          cutString = currentString.length();
          currentString.remove(cutString-2);     
          leftCorner = currentString.toFloat();
          PrintMessage("CMD_SEN_ROT_340");
          mydelay(30);
          PrintMessage("CMD_SEN_IR");
          mydelay(30);
          currentString = Serial.readString();
          cutString = currentString.length();
          currentString.remove(cutString-2);     
          rightCorner = currentString.toFloat();
          if((rightCorner != rightCorner) && (leftCorner != leftCorner)) {
            //PrintMessage("CMD_ACT_LAT_1_0.5");
          }
          else if((rightCorner != rightCorner) && (leftCorner == leftCorner)) {
            if(leftCorner < 2) {
              PrintMessage("CMD_ACT_ROT_1_10");
            }
            //PrintMessage("CMD_ACT_LAT_1_0.5");
          }
          else if((rightCorner == rightCorner) && (leftCorner != leftCorner)) {
            if(rightCorner < 2) {
              PrintMessage("CMD_ACT_ROT_0_10");
            }
            //PrintMessage("CMD_ACT_LAT_1_0.5");
          }
          else if((rightCorner > 0.75) && (leftCorner > 0.75)) {
            //PrintMessage("CMD_ACT_LAT_1_0.5");
          }
          else if((rightCorner > 0.75) && (leftCorner < 0.75)) {
            PrintMessage("CMD_ACT_ROT_1_90");
            wallWarning = true;
          }
          else if((rightCorner < 0.75) && (leftCorner > 0.75)) {
            PrintMessage("CMD_ACT_ROT_0_90");
            wallWarning = true;
            mydelay(30);
          }
          PrintMessage("CMD_SEN_PING");
          mydelay(30);
          currentString = Serial.readString();
          cutString = currentString.length();
          currentString.remove(cutString-2);     
          distanceGoal = currentString.toFloat();
          if(distanceGoal == 0) {
            foundGoal = false;
          }
          else if(!wallWarning) {
            mydelay(0);
            PrintMessage("CMD_ACT_LAT_1_0.5");
            mydelay(30);
            C = distanceGoal;
            B = 0.52;
            angleGoal = acos((B*B+C*C-A*A)/(2*B*C));
            angleGoal = 180 - (angleGoal *(180/3.14));
            lcd.print(" ");
            lcd.print(angleGoal);
            lcd.setCursor(0,1);
            lcd.print(A);
            lcd.print(" ");
            lcd.print(B);
            lcd.print(" ");
            lcd.print(C);
            mydelay(1000);
            if((angleGoal != 0) && (angleGoal == angleGoal)) {
              PrintMessage("CMD_ACT_ROT_1_" + (String) (angleGoal));
              reachGoal++;
            }
            
          }
        }
        else {
          PrintMessage("CMD_ACT_ROT_0_20");
          mydelay(200);
        }
        break;
      case 1:
        PrintMessage("CMD_SEN_IR");
        mydelay(50);
        currentString = Serial.readString();
        cutString = currentString.length();
        currentString.remove(cutString-2);     
        frontDistance = currentString.toFloat();
        if((frontDistance != frontDistance) || (frontDistance > 1.5)) {
          PrintMessage("CMD_ACT_LAT_1_0.5");
          mydelay(50);
          PrintMessage("CMD_SEN_PING");
          mydelay(50);
          currentString = Serial.readString();
          cutString = currentString.length();
          currentString.remove(cutString-2);     
          newDistanceGoal = currentString.toFloat();
          if(newDistanceGoal > distanceGoal && (newDistanceGoal == newDistanceGoal) && (newDistanceGoal == 0)) {
            mydelay(50);
            PrintMessage("CMD_ACT_ROT_0_180");
            mydelay(50);
            PrintMessage("CMD_ACT_LAT_1_0.5");
            reachGoal++;
          }
          else if(newDistanceGoal < distanceGoal){
            reachGoal++;
          }
        }
        else {
          PrintMessage("CMD_ACT_ROT_0_20");
          mydelay(200);
          reachGoal = 0;
        }
        break;
//      case 2:
//        break;
    }
  }
  
  switch(whatbuttons) {
    btnSELECT:
      startup = true;
      currentMode = mainMODE;
      currentMenuState = mainState;
      break;
      
  }

   


}
