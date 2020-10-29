#include <LiquidCrystal.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <Math.h>

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

int counter = 0;
boolean start = false;

boolean startup = true; 
boolean completed = false;
float currentAngle = 0; // to keep track of the direction of the robot 

int firePosition = 99; // if not 99, it means that the fire is found before the water and will tell us where
boolean startOp = true;
boolean back = true;
boolean foundTheFire = false;

enum Objective {
  idle,
  W,
  F,
  H,
  C
};
Objective currentObjective = idle;

enum objectiveCompletion {
  GoalFinding, //explore key positions for goals 
  GoalFound, // go within 2m of goal to find which goal it is 
  GoalReaching, //reach the goal whether fire or water
  GoalReached // get ready for next mission and show completion
};
objectiveCompletion waterCompletion = GoalFinding;
objectiveCompletion fireCompletion = GoalFinding;

int completion;

ISR(TIMER2_OVF_vect) { //Chapter 16
  //Register size = 64
  // CLK = 62500 Hz
  //Timer pertick = 1/CLK = 0.016ms
  // from 0 to 64 = 64 * 0.016ms = 1ms

  milliseconds += 1; //increment every ms
  
  counter++;
  if(start && currentObjective != C && counter >= 150) {
    counter = 0;
    printClock(mymillis()/60000, (mymillis() / 1000) % 60); //Convert the values to the print the clock
  }
}


void setup() {
  // put your setup code here, to run once:
  timer_Init();
  sei(); //enable global interrupt

  lcd.begin(16, 2);
  printClock(0, 0);
  Serial.begin(9600);
  Serial.setTimeout(350); // how long serial readString takes before timeout
  
  ADC_Init();  

  PrintMessage("CMD_START"); // Start the "robot"
}





void loop() {
  // put your main code here, to run repeatedly:
  if(start) {    
//    if(currentObjective != C) {
//      printClock(mymillis()/60000, (mymillis() / 1000) % 60); //Convert the values to the print the clock
//    }

    switch(currentObjective) {
      case W:
        waterOperation();
        break;
      case F:
        fireOperation();
        break;
      case H:
        homeOperation();
        break;
      case C:
        if(!completed) {
          printClock(mymillis()/60000, (mymillis() / 1000) % 60);
          completed = true;
        }
        completeOperation();
        break;
    }
  }

  
  if(startup) {
    whatbuttons = readLCDButtons();
    if(whatbuttons == btnSELECT) {
      start = true;
      milliseconds = 0;
      startup = false;
      currentObjective = W;
      //turnRight(180);
    }
  }
}



void PrintMessage(String message)
{
  Serial.print(message);
  Serial.write(13); //carriage return character (ASCII 13, or '\r')
  Serial.write(10); //newline character (ASCII 10, or '\n')
  mydelay(40);
}

void printClock(int minutes, int seconds) { // print clock for startup mode
  lcd.setCursor(0,0);
  lcd.print("12878930"); 
  lcd.setCursor(0,1);
  if(minutes <10) {
    lcd.print('0');
  }
  lcd.print(minutes);
  lcd.print(':');
  if(seconds < 10) {
    lcd.print('0');
  }
  lcd.print(seconds);
  modeToText();
}

void modeToText() {
  lcd.print(" ");
  switch(currentObjective) {
    case idle:
      break;
    case W:
      lcd.print("W");
      break;
    case F: 
      lcd.print("F");
      break;
    case H:
      lcd.print("H");
      break;
    case C: 
      lcd.print("C");
      break;
  }
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
  mydelay(175); //DEBOUNCE
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

uint8_t keyPosition = 0; 

void findGoal() {
  switch(keyPosition) {
    case 0: //start position at 2 4 0  
      if(withinPerimeter(4, distanceToGoal())) {
        if(waterCompletion == GoalFinding) {
          waterCompletion = GoalFound;
        }
        else if(fireCompletion == GoalFinding && waterCompletion == GoalReached) {
          fireCompletion = GoalFound;
        }
      }
      else {
        keyPosition++;
      }
      break;
    case 1:  // position at 2 11 270
      turnLeft(90);
      forward(5);
      forward(2);
      mydelay(1000);
      if(withinPerimeter(3, distanceToGoal())) {
        if(waterCompletion == GoalFinding) {
          waterCompletion = GoalFound;
        }
        else if(fireCompletion == GoalFinding && waterCompletion == GoalReached) {
          fireCompletion = GoalFound;
        }
      }
      else {
        keyPosition++;
      }
      break;
    case 2: // 6 8 0
      backward(3);
      turnRight(90);
      forward(4);
      if(withinPerimeter(2.5, distanceToGoal())) {
        if(waterCompletion == GoalFinding) {
          waterCompletion = GoalFound;
        }
        else if(fireCompletion == GoalFinding && waterCompletion == GoalReached) {
          fireCompletion = GoalFound;
        }
      }
      else {
        keyPosition++;
      }
      break;
      
    case 3: // position at 7 4 0
      backward(2);
      turnRight(90);
      forward(4);
      turnLeft(90);
      forward(3);
      if(withinPerimeter(2, distanceToGoal())) {
        if(waterCompletion == GoalFinding) {
          waterCompletion = GoalFound;
        }
        else if(fireCompletion == GoalFinding && waterCompletion == GoalReached) {
          fireCompletion = GoalFound;
        }
      }
      else {
        keyPosition++;
      }
      break;
    case 4: // position 10 4 0
      forward(3);
      if(withinPerimeter(3, distanceToGoal())) {
        if(waterCompletion == GoalFinding) {
          waterCompletion = GoalFound;
        }
        else if(fireCompletion == GoalFinding && waterCompletion == GoalReached) {
          fireCompletion = GoalFound;
        }
      }
      else {
        keyPosition++;
      }
      break;
    case 5: // position at 10 8 270
      turnLeft(90);
      forward(4);
      if(withinPerimeter(2, distanceToGoal())) {
        if(waterCompletion == GoalFinding) {
          waterCompletion = GoalFound;
        }
        else if(fireCompletion == GoalFinding && waterCompletion == GoalReached) {
          fireCompletion = GoalFound;
        }
      }
      else {
        keyPosition++;
      }
      break;
    case 6: // position at 10 13 270
      forward(5);
      if(withinPerimeter(2, distanceToGoal())) {
        if(waterCompletion == GoalFinding) {
          waterCompletion = GoalFound;
        }
        else if(fireCompletion == GoalFinding && waterCompletion == GoalReached) {
          fireCompletion = GoalFound;
        }
      }
      else {
        keyPosition++;
      }
      break;
    case 7: // 7 13 180 
      turnLeft(90);
      forward(3);
      if(withinPerimeter(3, distanceToGoal())) {
        if(waterCompletion == GoalFinding) {
          waterCompletion = GoalFound;
        }
        else if(fireCompletion == GoalFinding && waterCompletion == GoalReached) {
          fireCompletion = GoalFound;
        }
      }
      else {
        keyPosition++;
      }
      break;
    case 8: // position at 4 16 180
      turnRight(90);
      forward(2);
      turnLeft(90);
      forward(4);
      if(withinPerimeter(3, distanceToGoal())) {
        if(waterCompletion == GoalFinding) {
          waterCompletion = GoalFound;
        }
        else if(fireCompletion == GoalFinding && waterCompletion == GoalReached) {
          fireCompletion = GoalFound;
        }
      }
      else {
        keyPosition++;
      }
      break;
    case 9: // position at 4 18.5 270
      backward(1);
      turnRight(90);
      forward(3.5);
      if(withinPerimeter(3, distanceToGoal())) {
        if(waterCompletion == GoalFinding) {
          waterCompletion = GoalFound;
        }
        else if(fireCompletion == GoalFinding && waterCompletion == GoalReached) {
          fireCompletion = GoalFound;
        }
      }
      else {
        keyPosition++;
      }
      break;
    case 10: // position at 7 18 0
      backward(0.5);
      turnRight(90);
      forward(3);
      if(withinPerimeter(3, distanceToGoal())) {
        if(waterCompletion == GoalFinding) {
          waterCompletion = GoalFound;
        }
        else if(fireCompletion == GoalFinding && waterCompletion == GoalReached) {
          fireCompletion = GoalFound;
        }
      }
      else {
        keyPosition++;
      }
      break;
    case 11: // position at 10 18 0
      //turnRight(90);
      forward(3);
      if(withinPerimeter(3, distanceToGoal())) {
        if(waterCompletion == GoalFinding) {
          waterCompletion = GoalFound;
        }
        else if(fireCompletion == GoalFinding && waterCompletion == GoalReached) {
          fireCompletion = GoalFound;
        }
      }
      else {
        keyPosition++;
      }
      break;
    case 12: // position at 14 18 0
      forward(4);
      if(withinPerimeter(2, distanceToGoal())) {
        if(waterCompletion == GoalFinding) {
          waterCompletion = GoalFound;
        }
        else if(fireCompletion == GoalFinding && waterCompletion == GoalReached) {
          fireCompletion = GoalFound;
        }
      }
      else {
        keyPosition++;
      }
      break;
    case 13: // position at 19 16.5 90
      turnLeft(90);
      forward(0.5);
      turnRight(90);
      forward(4);
      turnRight(90);
      forward(2);
      if(withinPerimeter(2.25, distanceToGoal())) {
        if(waterCompletion == GoalFinding) {
          waterCompletion = GoalFound;
        }
        else if(fireCompletion == GoalFinding && waterCompletion == GoalReached) {
          fireCompletion = GoalFound;
        }
      }
      else {
        keyPosition++;
      }
      break;
    case 14: // position at 14 14.5 180
      forward(2);
      turnRight(90);
      forward(4);
      if(withinPerimeter(2.5, distanceToGoal())) {
        if(waterCompletion == GoalFinding) {
          waterCompletion = GoalFound;
        }
        else if(fireCompletion == GoalFinding && waterCompletion == GoalReached) {
          fireCompletion = GoalFound;
        }
      }
      else {
        keyPosition++;
      }
      break;
    case 15: // position at 14 11 0
      forward(1);
      turnLeft(90);
      forward(2.5);
      turnLeft(90);
      forward(1);
      if(withinPerimeter(3, distanceToGoal())) {
        if(waterCompletion == GoalFinding) {
          waterCompletion = GoalFound;
        }
        else if(fireCompletion == GoalFinding && waterCompletion == GoalReached) {
          fireCompletion = GoalFound;
        }
      }
      else {
        keyPosition++;
      }
      break;
    case 16: // position at 11 18 0
      turnRight(90);
      forward(1);
      turnLeft(90);
      forward(4);    
      if(withinPerimeter(3, distanceToGoal())) {
        if(waterCompletion == GoalFinding) {
          waterCompletion = GoalFound;
        }
        else if(fireCompletion == GoalFinding && waterCompletion == GoalReached) {
          fireCompletion = GoalFound;
        }
      }
      else {
        keyPosition++;
      }
      break;
    case 17: // position at 18 8 90  
      turnRight(90);
      forward(3);
      if(withinPerimeter(2.5, distanceToGoal())) {
        if(waterCompletion == GoalFinding) {
          waterCompletion = GoalFound;
        }
        else if(fireCompletion == GoalFinding && waterCompletion == GoalReached) {
          fireCompletion = GoalFound;
        }
      }
      else {
        keyPosition++;
      }
      break;
    case 18: // position at 18 4 90
      forward(4);    
      if(withinPerimeter(3, distanceToGoal())) {
        if(waterCompletion == GoalFinding) {
          waterCompletion = GoalFound;
        }
        else if(fireCompletion == GoalFinding && waterCompletion == GoalReached) {
          fireCompletion = GoalFound;
        }
      }
      else {
        keyPosition++;
      }
      break;
    case 19: // position at 14 4 180
      turnRight(90);
      forward(4);   
      if(withinPerimeter(2.5, distanceToGoal())) {
        if(waterCompletion == GoalFinding) {
          waterCompletion = GoalFound;
        }
        else if(fireCompletion == GoalFinding && waterCompletion == GoalReached) {
          fireCompletion = GoalFound;
        }
      }
      else {
        keyPosition++;
      }
      break;
    case 20: // position at 14 7 270
      turnRight(90);
      forward(3);    
      if(withinPerimeter(2, distanceToGoal())) {
        if(waterCompletion == GoalFinding) {
          waterCompletion = GoalFound;
        }
        else if(fireCompletion == GoalFinding && waterCompletion == GoalReached) {
          fireCompletion = GoalFound;
        }
        
      }
      else {
        keyPosition++;
      }
      break;
    case 21:
      if(currentObjective == F) {
        firePosition = 0;
        fireCompletion = GoalFound;
      }
  }
  completion = goalAchieved();
  if(completion == 1 && waterCompletion != GoalReached) {
    waterCompletion = GoalReached;
    keyPosition++;
  }
  else if(completion == 2 && currentObjective == W) {
    firePosition = keyPosition - 1;
  }
  else if(completion == 2 && currentObjective == F) {
    fireCompletion = GoalReached;
  }
  
}

float sensorValue() {
  float value;
  do {
  PrintMessage("CMD_SEN_IR");
  String currentString = Serial.readString();
  int cutString = currentString.length();
  currentString.remove(cutString-2);  
  value = currentString.toFloat(); 
  }while(value == 0);
  if(value != value) {
    return 5;
  }
  else {
    return value;
  }
  
}

float receiveGoalDistance() {
    String currentString = Serial.readString();
    int cutString = currentString.length();
    currentString.remove(cutString-2);  
    float value = currentString.toFloat();
    return value;
    
}

float distanceToGoal() {
  float distanceGoal;
  float tempGoal;
  do {
    PrintMessage("CMD_SEN_PING");
    mydelay(100);  
    distanceGoal = receiveGoalDistance();

    PrintMessage("CMD_SEN_PING");
    mydelay(100);    
    tempGoal = receiveGoalDistance();
  } while(distanceGoal != tempGoal);
  return distanceGoal;
}


boolean withinPerimeter(float perimeter, float goalDistance) { //check for each keyposition if the goal is within their reach (and not behind walls)
  if(goalDistance == 0) {
    return false;
  }
  else if(goalDistance <= perimeter) {
    return true;
  }
  else {
    return false;
  }
}

int goalAchieved() {
  PrintMessage("CMD_SEN_GOAL");
  String currentString = Serial.readString();
  int value = currentString.toInt();
  //lcd.print(value);
  return value;
}

int goalNumber() {
  PrintMessage("CMD_SEN_ID");
  String currentString = Serial.readString();
  int value = currentString.toInt();
  return value;
  
}

int secondMinAngle;

int minimumAngle;

void checkGoalWater() {
  int whichGoal;
  float frontDistance;
  float minimum; 
  int minimumIndex;
  

  static int exploredAngle = 0;
  static float distanceGoals[5];
  static boolean goalIdentified = false;
  switch(exploredAngle) {
    case 0: //check if within 2 m now 
      distanceGoals[0] = distanceToGoal();
      if(withinPerimeter(2, distanceGoals[0])) {
        whichGoal = goalNumber();
        if(whichGoal == 1) {
          goalIdentified = true;
        }
        else  if(whichGoal == 2) {
          firePosition = keyPosition;
          keyPosition++;
          waterCompletion = GoalFinding;
          exploredAngle = 0;
          goalIdentified = false;
          return;
        }
      }
      exploredAngle++;
      break;
    case 1: // move to one direction
      frontDistance = sensorValue();
      if(frontDistance > 2.2) {
        forward(2);
      }
      else if(frontDistance > 1.11) {
        forward(1);
      }
      distanceGoals[1] = distanceToGoal();
      if(withinPerimeter(2, distanceGoals[1])) {
        whichGoal = goalNumber();
        if(whichGoal == 1) {
          goalIdentified = true;
        }
        else  if(whichGoal == 2)  {
          if(frontDistance > 2.2) {
            backward(2);
          }
          else if(frontDistance > 1.11) {
            backward(1);
          }
          firePosition = keyPosition;
          keyPosition++;
          waterCompletion = GoalFinding;
          exploredAngle = 0;
          goalIdentified = false;
          return;
        }
      }
      if(frontDistance > 2.2) {
        backward(2);
      }
      else if(frontDistance > 1.11) {
        backward(1);
      }
      exploredAngle++;
      break;
    case 2:
      turnRight(90);
      frontDistance = sensorValue();
      if(frontDistance > 2.2) {
        forward(2);
      }
      else if(frontDistance > 1.11) {
        forward(1);
      }
      distanceGoals[2] = distanceToGoal();
      if(withinPerimeter(2, distanceGoals[2])) {
        whichGoal = goalNumber();
        if(whichGoal == 1) {
          goalIdentified = true;
        }
        else  if(whichGoal == 2.2)  {
          if(frontDistance > 2) {
            backward(2);
          }
          else if(frontDistance > 1.11) {
            backward(1);
          }
          turnLeft(90);
          firePosition = keyPosition;
          keyPosition++;
          waterCompletion = GoalFinding;
          exploredAngle = 0;
          goalIdentified = false;
          return;
        }
      }

      exploredAngle++;
      if(frontDistance > 2.2) {
        backward(2);
      }
      else if(frontDistance > 1.11) {
        backward(1);
      }

      break;
    case  3:
      turnRight(90);
      frontDistance = sensorValue();
      if(frontDistance > 2.2) {
        forward(2);
      }
      else if(frontDistance > 1.11) {
        forward(1);
      }
      distanceGoals[3] = distanceToGoal();
      if(withinPerimeter(2, distanceGoals[3])) {
        whichGoal = goalNumber();
        if(whichGoal == 1) {
          goalIdentified = true;
        }
        else  if(whichGoal == 2.2) {
          if(frontDistance > 2) {
            backward(2);
          }
          else if(frontDistance > 1.11) {
            backward(1);
          }
          turnRight(180);
          firePosition = keyPosition;
          keyPosition++;
          waterCompletion = GoalFinding;
          exploredAngle = 0;
          goalIdentified = false;
          return;
        }
      }
      exploredAngle++;
      if(frontDistance > 2.2) {
        backward(2);
      }
      else if(frontDistance > 1.11) {
        backward(1);
      }
      break;
    case  4:
      turnRight(90);
      frontDistance = sensorValue();
      if(frontDistance > 2.2) {
        forward(2);
      }
      else if(frontDistance > 1.11) {
        forward(1);
      }
      distanceGoals[4] = distanceToGoal();
      if(withinPerimeter(2, distanceGoals[4])) {
        whichGoal = goalNumber();
        if(whichGoal == 1) {
          goalIdentified = true;
        }
        else  if(whichGoal == 2)  {
          if(frontDistance > 2.2) {
            backward(2);
          }
          else if(frontDistance > 1.11) {
            backward(1);
          }
          turnRight(90);
          firePosition = keyPosition;
          keyPosition++;
          waterCompletion = GoalFinding;
          exploredAngle = 0;
          goalIdentified = false;
          return;
        }
      }
      if(frontDistance > 2.2) {
        backward(2);
      }
      else if(frontDistance > 1.11) {
        backward(1);
      }
      turnRight(90);
      exploredAngle++;
      break;
  }
  
  completion = goalAchieved();
  if(completion == 1) {
    switch(exploredAngle) {
      case 3:
        turnLeft(90);
        break;
      case 4:
        turnRight(180);
        break;
    }
    waterCompletion = GoalReached;
    keyPosition++;
    return;
  }

  if(goalIdentified && exploredAngle == 5) {
    //find minimum out of all the arrays
    minimum = distanceGoals[1];
    minimumAngle = 0;
    for(int i = 2; i < 5; i++){
      if(minimum > distanceGoals[i]) {
        minimumAngle = (i -1)*90;
        minimum = distanceGoals[i];
        minimumIndex = i;
      }
    }
    if(minimumIndex != 0) {
      if(minimumIndex == 1) {
        if(distanceGoals[4] > distanceGoals[2]) {
          secondMinAngle = 90;
        }
        else {
          secondMinAngle = 270;
        }
      }
      else if(minimumIndex == 4){
        if(distanceGoals[3] > distanceGoals[1]) {
          secondMinAngle = 0;
        }
        else {
          secondMinAngle = 180;
        }
      }
      else {
        if(distanceGoals[minimumIndex +1] > distanceGoals[minimumIndex -1]) {
          secondMinAngle = minimumAngle -90;
          if(minimumAngle == 0) {
            secondMinAngle = 270;
          }
        }
        else {
          secondMinAngle = minimumAngle  +90;
        }
      }
    }
//    lcd.print(minimumAngle);
//    lcd.print(" ");
//    lcd.print(secondMinAngle);
    waterCompletion = GoalReaching;
    keyPosition++;

  }

}

void reachGoal() {
  static int directionScan; // 0 is CCW and 1 os CW
  float frontDistance = 0;
  float goalDistance = 0;
  switch(minimumAngle) {
    case 0:
      if(secondMinAngle == 90) { //travel every 30 degrees to check for the goal
        for(int i = 0; i <= 3; i++) {
          frontDistance = sensorValue();
          // check for distance to wall and travel
          if(frontDistance > 3.4) {
            forward(3);
          }
          else if(frontDistance > 2.2) {
            forward(2);
          }
          else if(frontDistance > 1.1) {
            forward(1);
          }
          else if(frontDistance > 0.55) {
            forward(0.55);
          }

          // coming back
          if(frontDistance > 3.4) {
            backward(3);
          }
          else if(frontDistance > 2.2) {
            backward(2);
          }
          else if(frontDistance > 1.11) {
            backward(1);
          }
          else if(frontDistance > 0.55) {
            backward(0.55);
          }
          if(goalAchieved() == 1 && currentObjective == W) {
            waterCompletion = GoalReached;
            switch(i) {
              case 1:
                turnLeft(30);
                break;
              case 2:
                turnLeft(60);
                break;
              case 3:
                turnLeft(90);
                break;
            }
            return;
          }
          else if(goalAchieved() == 2 && currentObjective == F) {
            fireCompletion = GoalReached;
            switch(i) {
              case 1:
                turnLeft(30);
                break;
              case 2:
                turnLeft(60);
                break;
              case 3:
                turnLeft(90);
                break;
            }
            return;
          }
          if(i != 3) {
            turnRight(30); 
          }
          
        }
        turnLeft(90);
        if(currentObjective == W) {
          waterCompletion = GoalReached;
        }
        else if(currentObjective == F) {
          fireCompletion = GoalReached;
        }
        return;
      }
      else if(secondMinAngle == 270) {
        for(int i = 0; i <= 3; i++) {
          frontDistance = sensorValue();
          // check for distance to wall and travel
          if(frontDistance > 3.4) {
            forward(3);
          }
          else if(frontDistance > 2.2) {
            forward(2);
          }
          else if(frontDistance > 1.11) {
            forward(1);
          }
          else if(frontDistance > 0.55) {
            forward(0.55);
          }

          // coming back
          if(frontDistance > 3.4) {
            backward(3);
          }
          else if(frontDistance > 2.2) {
            backward(2);
          }
          else if(frontDistance > 1.11) {
            backward(1);
          }
          else if(frontDistance > 0.55) {
            backward(0.55);
          }
          if(goalAchieved() == 1 && currentObjective == W) {
            waterCompletion = GoalReached;
            switch(i) {
              case 1:
                turnRight(30);
                break;
              case 2:
                turnRight(60);
                break;
              case 3:
                turnRight(90);
                break;
            }
            return;
          }
          else if(goalAchieved() == 2 && currentObjective == F) {
            fireCompletion = GoalReached;
            switch(i) {
              case 1:
                turnRight(30);
                break;
              case 2:
                turnRight(60);
                break;
              case 3:
                turnRight(90);
                break;
            }
            return;
          }
          if(i != 3) {
            turnLeft(30); 
          } 
        }
        turnRight(90);
        if(currentObjective == W) {
          waterCompletion = GoalReached;
        }
        else if(currentObjective == F) {
          fireCompletion = GoalReached;
        }
        return;
      }
      break;
    case 90:
      if(secondMinAngle == 180) {
        turnRight(90);
        for(int i = 0; i <= 3; i++) {
          frontDistance = sensorValue();
          // check for distance to wall and travel
          if(frontDistance > 3.4) {
            forward(3);
          }
          else if(frontDistance > 2.2) {
            forward(2);
          }
          else if(frontDistance > 1.11) {
            forward(1);
          }
          else if(frontDistance > 0.55) {
            forward(0.55);
          }

          // coming back
          if(frontDistance > 3.4) {
            backward(3);
          }
          else if(frontDistance > 2.2) {
            backward(2);
          }
          else if(frontDistance > 1.11) {
            backward(1);
          }
          else if(frontDistance > 0.55) {
            backward(0.55);
          }
          if(goalAchieved() == 1 && currentObjective == W) {
            waterCompletion = GoalReached;
            switch(i) {
              case 1:
                turnLeft(30);
                break;
              case 2:
                turnLeft(60);
                break;
              case 3:
                turnLeft(90);
                break;
            }
            turnLeft(90);
            return;
          }
          else if(goalAchieved() == 2 && currentObjective == F) {
            fireCompletion = GoalReached;
            switch(i) {
              case 1:
                turnLeft(30);
                break;
              case 2:
                turnLeft(60);
                break;
              case 3:
                turnLeft(90);
                break;
            }
            turnLeft(90);
            return;
          }
          if(i != 3) {
            turnRight(30); 
          }
        }
        turnLeft(180);
        if(currentObjective == W) {
          waterCompletion = GoalReached;
        }
        else if(currentObjective == F) {
          fireCompletion = GoalReached;
        }
        return;
      }
      else if(secondMinAngle == 0) {
        for(int i = 0; i <= 3; i++) {
          frontDistance = sensorValue();
          // check for distance to wall and travel
          if(frontDistance > 3.4) {
            forward(3);
          }
          else if(frontDistance > 2.2) {
            forward(2);
          }
          else if(frontDistance > 1.11) {
            forward(1);
          }
          else if(frontDistance > 0.55) {
            forward(0.55);
          }

          // coming back
          if(frontDistance > 3.4) {
            backward(3);
          }
          else if(frontDistance > 2.2) {
            backward(2);
          }
          else if(frontDistance > 1.11) {
            backward(1);
          }
          else if(frontDistance > 0.55) {
            backward(0.55);
          }
          if(goalAchieved() == 1 && currentObjective == W) {
            waterCompletion = GoalReached;
            switch(i) {
              case 1:
                turnLeft(30);
                break;
              case 2:
                turnLeft(60);
                break;
              case 3:
                turnLeft(90);
                break;
            }
            return;
          }
          else if(goalAchieved() == 2 && currentObjective == F) {
            fireCompletion = GoalReached;
            switch(i) {
              case 1:
                turnLeft(30);
                break;
              case 2:
                turnLeft(60);
                break;
              case 3:
                turnLeft(90);
                break;
            }
            return;
          }
          if(i != 3) {
            turnRight(30); 
          }
        }
        turnLeft(90);
        if(currentObjective == W) {
          waterCompletion = GoalReached;
        }
        else if(currentObjective == F) {
          fireCompletion = GoalReached;
        }
        return;
      }
      break;
    case 180:
      if(secondMinAngle == 270) {
        turnLeft(90);
        for(int i = 0; i <= 3; i++) {
          frontDistance = sensorValue();
          // check for distance to wall and travel
          if(frontDistance > 3.4) {
            forward(3);
          }
          else if(frontDistance > 2.2) {
            forward(2);
          }
          else if(frontDistance > 1.11) {
            forward(1);
          }
          else if(frontDistance > 0.55) {
            forward(0.55);
          }

          // coming back
          if(frontDistance > 3.4) {
            backward(3);
          }
          else if(frontDistance > 2.2) {
            backward(2);
          }
          else if(frontDistance > 1.11) {
            backward(1);
          }
          else if(frontDistance > 0.55) {
            backward(0.55);
          }
          if(goalAchieved() == 1 && currentObjective == W) {
            waterCompletion = GoalReached;
            switch(i) {
              case 1:
                turnRight(30);
                break;
              case 2:
                turnRight(60);
                break;
              case 3:
                turnRight(90);
                break;
            }
            turnRight(90);
            return;
          }
          else if(goalAchieved() == 2 && currentObjective == F) {
            fireCompletion = GoalReached;
            switch(i) {
              case 1:
                turnRight(30);
                break;
              case 2:
                turnRight(60);
                break;
              case 3:
                turnRight(90);
                break;
            }
            turnRight(90);
            return;
          }
          if(i != 3) {
            turnLeft(30); 
          }
        }
        turnRight(180);
        if(currentObjective == W) {
          waterCompletion = GoalReached;
        }
        else if(currentObjective == F) {
          fireCompletion = GoalReached;
        }
        return;
      }
      else if(secondMinAngle == 90) {
        turnRight(90);
        for(int i = 0; i <= 3; i++) {
          frontDistance = sensorValue();
          // check for distance to wall and travel
          if(frontDistance > 3.4) {
            forward(3);
          }
          else if(frontDistance > 2.2) {
            forward(2);
          }
          else if(frontDistance > 1.11) {
            forward(1);
          }
          else if(frontDistance > 0.55) {
            forward(0.5);
          }

          // coming back
          if(frontDistance > 3.4) {
            backward(3);
          }
          else if(frontDistance > 2.2) {
            backward(2);
          }
          else if(frontDistance > 1.11) {
            backward(1);
          }
          else if(frontDistance > 0.55) {
            backward(0.5);
          }
          if(goalAchieved() == 1 && currentObjective == W) {
            waterCompletion = GoalReached;
            switch(i) {
              case 1:
                turnLeft(30);
                break;
              case 2:
                turnLeft(60);
                break;
              case 3:
                turnLeft(90);
                break;
            }
            turnLeft(90);
            return;
          }
          else if(goalAchieved() == 2 && currentObjective == F) {
            fireCompletion = GoalReached;
            switch(i) {
              case 1:
                turnLeft(30);
                break;
              case 2:
                turnLeft(60);
                break;
              case 3:
                turnLeft(90);
                break;
            }
            turnLeft(90);
            return;
          }
          if(i != 3) {
            turnRight(30); 
          }
        }
        turnLeft(180);
        if(currentObjective == W) {
          waterCompletion = GoalReached;
        }
        else if(currentObjective == F) {
          fireCompletion = GoalReached;
        }
        return;
      }
      break;
    case 270:
      if(secondMinAngle == 180) {
        turnLeft(90);
        for(int i = 0; i <= 3; i++) {
          frontDistance = sensorValue();
          // check for distance to wall and travel
          if(frontDistance > 3.4) {
            forward(3);
          }
          else if(frontDistance > 2.2) {
            forward(2);
          }
          else if(frontDistance > 1.11) {
            forward(1);
          }
          else if(frontDistance > 0.55) {
            forward(0.55);
          }

          // coming back
          if(frontDistance > 3.4) {
            backward(3);
          }
          else if(frontDistance > 2.2) {
            backward(2);
          }
          else if(frontDistance > 1.11) {
            backward(1);
          }
          else if(frontDistance > 0.55) {
            backward(0.55);
          }
          if(goalAchieved() == 1 && currentObjective == W) {
            waterCompletion = GoalReached;
            switch(i) {
              case 1:
                turnRight(30);
                break;
              case 2:
                turnRight(60);
                break;
              case 3:
                turnRight(90);
                break;
            }
            turnRight(90);
            return;
          }
          else if(goalAchieved() == 2 && currentObjective == F) {
            fireCompletion = GoalReached;
            switch(i) {
              case 1:
                turnRight(30);
                break;
              case 2:
                turnRight(60);
                break;
              case 3:
                turnRight(90);
                break;
            }
            turnRight(90);
            return;
          }
          if(i != 3) {
            turnLeft(30); 
          }
        }
        turnRight(180);
        if(currentObjective == W) {
          waterCompletion = GoalReached;
        }
        else if(currentObjective == F) {
          fireCompletion = GoalReached;
        }
        return;
      }
      else if(secondMinAngle == 0) {
        for(int i = 0; i <= 3; i++) {
          frontDistance = sensorValue();
          // check for distance to wall and travel
          if(frontDistance > 3.4) {
            forward(3);
          }
          else if(frontDistance > 2.2) {
            forward(2);
          }
          else if(frontDistance > 1.11) {
            forward(1);
          }
          else if(frontDistance > 0.55) {
            forward(0.55);
          }

          // coming back
          if(frontDistance > 3.4) {
            backward(3);
          }
          else if(frontDistance > 2.2) {
            backward(2);
          }
          else if(frontDistance > 1.11) {
            backward(1);
          }
          else if(frontDistance > 0.55) {
            backward(0.55);
          }
          if(goalAchieved() == 1 && currentObjective == W) {
            waterCompletion = GoalReached;
            switch(i) {
              case 1:
                turnRight(30);
                break;
              case 2:
                turnRight(60);
                break;
              case 3:
                turnRight(90);
                break;
            }
            return;
          }
          else if(goalAchieved() == 2 && currentObjective == F) {
            fireCompletion = GoalReached;
            switch(i) {
              case 1:
                turnRight(30);
                break;
              case 2:
                turnRight(60);
                break;
              case 3:
                turnRight(90);
                break;
            }
            return;
          }
          if(i != 3) {
            turnLeft(30); 
          }
        }
        turnRight(90);
        if(currentObjective == W) {
          waterCompletion = GoalReached;
        }
        else if(currentObjective == F) {
          fireCompletion = GoalReached;
        }
        return;
      }
      break;
  }
  
}


void checkGoalFire() {
  int whichGoal;
  float frontDistance;
  float minimum; 
  int minimumIndex;
  

  static int exploredAngle = 0;
  static float distanceGoals[5];
  static boolean goalIdentified = false;
  switch(exploredAngle) {
    case 0: //check if within 2 m now 
      distanceGoals[0] = distanceToGoal();
      if(withinPerimeter(2, distanceGoals[0])) {
        whichGoal = goalNumber();
        if(whichGoal == 2) {
          goalIdentified = true;
        }
        else  if(whichGoal == 1) {
          //firePosition = keyPosition;
          keyPosition++;
          //waterCompletion = GoalFinding;
          return;
        }
      }
      exploredAngle++;
      break;
    case 1: // move to one direction
      frontDistance = sensorValue();
      if(frontDistance > 2.5) {
        forward(2);
      }
      else if(frontDistance > 1.11) {
        forward(1);
      }
      distanceGoals[1] = distanceToGoal();
      if(withinPerimeter(2, distanceGoals[1])) {
        whichGoal = goalNumber();
        if(whichGoal == 2) {
          goalIdentified = true;
        }
        else  if(whichGoal == 1)  {
          if(frontDistance > 2.5) {
            backward(2);
          }
          else if(frontDistance > 1.11) {
            backward(1);
          }
          keyPosition++;
          fireCompletion = GoalFinding;
          return;
        }
      }
      if(frontDistance > 2.5) {
        backward(2);
      }
      else if(frontDistance > 1.11) {
        backward(1);
      }
      exploredAngle++;
      break;
    case 2:
      turnRight(90);
      frontDistance = sensorValue();
      if(frontDistance > 2.5) {
        forward(2);
      }
      else if(frontDistance > 1.11) {
        forward(1);
      }
      distanceGoals[2] = distanceToGoal();
      if(withinPerimeter(2, distanceGoals[1])) {
        whichGoal = goalNumber();
        if(whichGoal == 2) {
          goalIdentified = true;
        }
        else  if(whichGoal == 1)  {
          if(frontDistance > 2.5) {
            backward(2);
          }
          else if(frontDistance > 1.11) {
            backward(1);
          }
          turnLeft(90);
          keyPosition++;
          fireCompletion = GoalFinding;
          return;
        }
      }
      exploredAngle++;
      if(frontDistance > 2.5) {
        backward(2);
      }
      else if(frontDistance > 1.11) {
        backward(1);
      }

      break;
    case  3:
      turnRight(90);
      frontDistance = sensorValue();
      if(frontDistance > 2.5) {
        forward(2);
      }
      else if(frontDistance > 1.11) {
        forward(1);
      }
      distanceGoals[3] = distanceToGoal();
      if(withinPerimeter(2, distanceGoals[3])) {
        whichGoal = goalNumber();
        if(whichGoal == 2) {
          goalIdentified = true;
        }
        else  if(whichGoal == 2.5) {
          if(frontDistance > 2) {
            backward(2);
          }
          else if(frontDistance > 1.11) {
            backward(1);
          }
          turnRight(180);
          keyPosition++;
          fireCompletion = GoalFinding;
          return;
        }
      }
      exploredAngle++;
      if(frontDistance > 2.5) {
        backward(2);
      }
      else if(frontDistance > 1.11) {
        backward(1);
      }
      break;
    case  4:
      turnRight(90);
      frontDistance = sensorValue();
      if(frontDistance > 2.5) {
        forward(2);
      }
      else if(frontDistance > 1.11) {
        forward(1);
      }
      distanceGoals[4] = distanceToGoal();
      if(withinPerimeter(2, distanceGoals[4])) {
        whichGoal = goalNumber();
        if(whichGoal == 2) {
          goalIdentified = true;
        }
        else  if(whichGoal == 1)  {
          if(frontDistance > 2.5) {
            backward(2);
          }
          else if(frontDistance > 1.11) {
            backward(1);
          }
          turnRight(90);
          keyPosition++;
          fireCompletion = GoalFinding;
          return;
        }
      }
      if(frontDistance > 2.5) {
        backward(2);
      }
      else if(frontDistance > 1.11) {
        backward(1);
      }
      turnRight(90);
      exploredAngle++;
      break;
  }
  completion = goalAchieved();
  if(completion == 2) {
    switch(exploredAngle) {
      case 3:
        turnLeft(90);
        break;
      case 4:
        turnRight(180);
        break;
    }
    fireCompletion = GoalReached;
    //keyPosition++;
    return;
  }

  if(goalIdentified && exploredAngle == 5) {
    //find minimum out of all the arrays
    minimum = distanceGoals[1];
    minimumAngle = 0;
    for(int i = 2; i < 5; i++){
      if(minimum > distanceGoals[i]) {
        minimumAngle = (i -1)*90;
        minimum = distanceGoals[i];
        minimumIndex = i;
      }
    }
    if(minimumIndex != 0) {
      if(minimumIndex == 1) {
        if(distanceGoals[4] > distanceGoals[2]) {
          secondMinAngle = 90;
        }
        else {
          secondMinAngle = 270;
        }
      }
      else if(minimumIndex == 4){
        if(distanceGoals[3] > distanceGoals[1]) {
          secondMinAngle = 0;
        }
        else {
          secondMinAngle = 180;
        }
      }
      else {
        if(distanceGoals[minimumIndex +1] > distanceGoals[minimumIndex -1]) {
          secondMinAngle = minimumAngle -90;
          if(minimumAngle == 0) {
            secondMinAngle = 270;
          }
        }
        else {
          secondMinAngle = minimumAngle  +90;
        }
      }
    }
//    lcd.print(minimumAngle);
//    lcd.print(" ");
//    lcd.print(secondMinAngle);
    fireCompletion = GoalReaching;

  }

}


void waterOperation() {
  switch(waterCompletion) {
    case GoalFinding:
      findGoal();
      break;
    case GoalFound:
      checkGoalWater();
      break;
    case GoalReaching:
      reachGoal();
      break;
    case GoalReached:
      currentObjective = F;
      break;
  }
}


void fireOperation() {
  switch(fireCompletion) {
    case GoalFinding:
      if(firePosition != 99) {
        if(back) {
          keyPosition--;
          back = false;
        }
        backToFire();
      }
      else {
        findGoal();
      }
      break;
    case GoalFound:
      checkGoalFire();
      break;
    case GoalReaching:
       reachGoal();
      break;
    case GoalReached:
      currentObjective = H;
      break;
  }
}


void backToFire() {
  //static boolean startOp = true;
  foundTheFire = true;
  switch(keyPosition) {
    case 20:
      if(keyPosition == firePosition) {
        fireCompletion = GoalFound;
        return;
      }
      if(startOp) { // change the angle to "face" home
        turnLeft(180);
        startOp = false;
      }
      forward(3);
      keyPosition--;
      break;
    case 19:
      if(keyPosition == firePosition) {
        fireCompletion = GoalFound;
        return;
      }
      if(startOp) { // change the angle to "face" home
        turnLeft(90);
        startOp = false;
      }    
      turnLeft(90);
      forward(4);
      keyPosition--;
      break;
    case 18:
      if(keyPosition == firePosition) {
        fireCompletion = GoalFound;
        return;
      }
      if(startOp) { // change the angle to "face" home
        turnLeft(90);
        startOp = false;
      }
      turnLeft(90);
      forward(4);
      keyPosition--;
      break;
    case 17:
      if(keyPosition == firePosition) {
        fireCompletion = GoalFound;
        return;
      }
      if(startOp) { // change the angle to "face" home
        turnLeft(180);
        startOp = false;
      }
      forward(3);
      keyPosition--;
      break;
    case 16:
      if(keyPosition == firePosition) {
        fireCompletion = GoalFound;
        return;
      }
      if(startOp) { // change the angle to "face" home
        turnLeft(90);
        startOp = false;
      }
      turnLeft(90);
      forward(4);
      turnRight(90);
      forward(1);
      turnLeft(90);
      keyPosition--;
      break;
    case 15:
      if(keyPosition == firePosition) {
        fireCompletion = GoalFound;
        return;
      }
      if(startOp) { // change the angle to "face" home
        turnLeft(180);
        startOp = false;
      }
      forward(1);
      turnRight(90);
      forward(2.5);
      turnRight(90);
      forward(1);
      keyPosition--;
      break;
    case 14:
      if(keyPosition == firePosition) {
        fireCompletion = GoalFound;
        return;
      }
      if(startOp) { // change the angle to "face" home
        turnLeft(180);
        startOp = false;
      }
      forward(4);
      turnLeft(90);
      forward(2);
      keyPosition--;
      break;
    case 13:
      if(keyPosition == firePosition) {
        fireCompletion = GoalFound;
        return;
      }
      if(startOp) { // change the angle to "face" home
        turnLeft(180);
        startOp = false;
      }
      forward(2);
      turnLeft(90);
      forward(4);
      turnLeft(90);
      forward(0.5);
      turnRight(90);
      keyPosition--;
      break;
    case 12:
      if(keyPosition == firePosition) {
        fireCompletion = GoalFound;
        return;
      }
      if(startOp) { // change the angle to "face" home
        turnLeft(180);
        startOp = false;
      }
      forward(4);
      keyPosition--;
      break;
    case 11:
      if(keyPosition == firePosition) {
        fireCompletion = GoalFound;
        return;
      }
      if(startOp) { // change the angle to "face" home
        turnLeft(180);
        startOp = false;
      }
      forward(3);
      //turnLeft(90);
      keyPosition--;
      break;
    case 10:
      if(keyPosition == firePosition) {
        fireCompletion = GoalFound;
        return;
      }
      if(startOp) { // change the angle to "face" home
        turnRight(180);
        startOp = false;
      }
      forward(3);
      turnRight(90);
      forward(0.5);
      keyPosition--;
      break;
    case 9:
      if(keyPosition == firePosition) {
        fireCompletion = GoalFound;
        return;
      }
      if(startOp) { // change the angle to "face" home
        //turnRight(180);
        startOp = false;
      }
      backward(3.5);
      turnLeft(90);
      forward(1);
      keyPosition--;
      break;
    case 8:
      if(keyPosition == firePosition) {
        fireCompletion = GoalFound;
        return;
      }
      if(startOp) { // change the angle to "face" home
        startOp = false;
      }
      backward(4);
      turnLeft(90);
      forward(2);
      keyPosition--;
      break;
    case 7:
      if(keyPosition == firePosition) {
        fireCompletion = GoalFound;
        return;
      }
      if(startOp) { // change the angle to "face" home
        turnRight(180);
        forward(3);
        keyPosition--;
        startOp = false;
      }
      else {
        turnLeft(90);
        forward(3);
        keyPosition--;
      }
      break;
    case 6:
      if(keyPosition == firePosition) {
        fireCompletion = GoalFound;
        return;
      }
      if(startOp) { // change the angle to "face" home
        turnLeft(180);
        forward(5);
        keyPosition--;
        startOp = false;
      }
      else {
        turnRight(90);
        forward(5);
        keyPosition--;
      }
      break;
    case 5:
      if(keyPosition == firePosition) {
        fireCompletion = GoalFound;
        return;
      }
      if(startOp) { // change the angle to "face" home
        turnLeft(180);
        startOp = false;
      }
      forward(4);
      keyPosition--;
      break;
    case 4:
      if(keyPosition == firePosition) {
        fireCompletion = GoalFound;
        return;
      }
      if(startOp) { // change the angle to "face" home
        turnRight(90);
        startOp = false;
      }
      turnRight(90);
      forward(3);
      keyPosition--;
      break;
    case 3:
      if(keyPosition == firePosition) {
        fireCompletion = GoalFound;
        return;
      }
      if(startOp) { // change the angle to "face" home
        turnRight(180);
        startOp = false;
      }
      forward(3);
      turnRight(90);
      forward(4);
      turnLeft(90);
      backward(2);
      keyPosition--;
      break;
    case 2:
      if(keyPosition == firePosition) {
        fireCompletion = GoalFound;
        return;
      }
      if(startOp) { // change the angle to "face" home
        turnLeft(180);
        startOp = false;
      }
      forward(4);
      turnRight(90);
      forward(2);
      keyPosition--;
      break;
    case 1:
      if(keyPosition == firePosition) {
        fireCompletion = GoalFound;
        return;
      }
      backward(5);
      backward(2);
      keyPosition = 0;
//      if(startOp) { // change the angle to "face" home
//        backward(5);
//        backward(1);
//        keyPosition = 0;
//        startOp = false;
//      }
      break;
    case 0:
      if(keyPosition == firePosition) {
        fireCompletion = GoalFound;
        return;
      }
      //currentObjective = C;
      break;

      
  }
}

void homeOperation() {
  //static boolean startOp = true;
  switch(keyPosition) {
    case 20:
      if(startOp) { // change the angle to "face" home
        turnLeft(180);
        startOp = false;
      }
      forward(3);
      keyPosition--;
      break;
    case 19:
      if(startOp) { // change the angle to "face" home
        turnLeft(90);
        startOp = false;
      }    
      turnLeft(90);
      forward(4);
      keyPosition--;
      break;
    case 18:
      if(startOp) { // change the angle to "face" home
        turnLeft(90);
        startOp = false;
      }
      turnLeft(90);
      forward(4);
      keyPosition--;
      break;
    case 17:
      if(startOp) { // change the angle to "face" home
        turnLeft(180);
        startOp = false;
      }
      forward(3);
      keyPosition--;
      break;
    case 16:
      if(startOp) { // change the angle to "face" home
        turnLeft(90);
        startOp = false;
      }
      turnLeft(90);
      forward(5);
      turnRight(90);
      forward(5);
      forward(2);
      keyPosition = 12;
      break;
    case 15:
      if(foundTheFire) {
        forward(1);
        turnRight(90);
        forward(5);
        forward(1);
        keyPosition = 12;
        startOp = false;
        foundTheFire = false;
      }
      if(startOp) { // change the angle to "face" home
        turnLeft(180);
        forward(1);
        turnRight(90);
        forward(5);
        forward(1);
        keyPosition = 12;
        startOp = false;
      }
      break;
    case 14:
      if(foundTheFire) {
        backward(1);
        turnLeft(90);
        forward(3.5);
        keyPosition = 12;
        startOp = false;
        foundTheFire = false;
      }
      if(startOp) { // change the angle to "face" home
        forward(1);
        turnRight(90);
        forward(3.5);
        keyPosition = 12;
        startOp = false;
      }
      break;
    case 13:
      if(foundTheFire) {
        forward(2);
        turnLeft(90);
        forward(5);
        turnRight(90);
        backward(0.5);
        keyPosition--;
        foundTheFire = false;
        startOp = false;
      }
      if(startOp) { // change the angle to "face" home
        backward(2);
        turnRight(90);
        forward(5);
        turnRight(90);
        backward(0.5);
        keyPosition--;
        startOp = false;
      }
      break;
    case 12:
      if(foundTheFire) {
        backward(1);
        turnLeft(90);
        startOp = false;
        foundTheFire = false;
      }
      if(startOp) { // change the angle to "face" home
        backward(1);
        turnLeft(90);
        startOp = false;
      }
      turnLeft(90);
      forward(3);
      keyPosition--;
      break;
    case 11:
      if(startOp) { // change the angle to "face" home
        turnLeft(180);
        startOp = false;
      }
      forward(3);
      turnLeft(90);
      keyPosition--;
      break;
    case 10:
      if(startOp) { // change the angle to "face" home
        turnRight(90);
        startOp = false;
      }
      forward(5);
      keyPosition = 7;
      break;
    case 9:
      if(startOp) { // change the angle to "face" home
        startOp = false;
      }
      backward(3.5);
      turnRight(90);
      forward(3);
      turnRight(90);
      forward(2);
      keyPosition = 7;
      break;
    case 8:
      if(startOp) { // change the angle to "face" home
        backward(3);
        turnLeft(90);
        forward(3);
        keyPosition--;
        startOp = false;
      }
      break;
    case 7:
      if(startOp) { // change the angle to "face" home
        turnLeft(90);   
        startOp = false;
      }
      forward(5);
      turnRight(90);
      forward(1);
      keyPosition = 2;
      break;
    case 6:
      if(foundTheFire) {
        backward(3);
        turnRight(90);
        foundTheFire = false;
        startOp = false;
      }
      if(startOp) { // change the angle to "face" home
        turnLeft(90);
        forward(3);
        turnLeft(90);
        keyPosition = 7;
        startOp = false;
      }
      break;
    case 5:
      if(foundTheFire) {
        //turnLeft(180);
        startOp = false;
      }
      if(startOp) { // change the angle to "face" home
        turnLeft(180);
        startOp = false;
      }
      forward(4);
      keyPosition--;
      break;
    case 4:
      if(startOp) { // change the angle to "face" home
        turnRight(90);
        startOp = false;
      }
      turnRight(90);
      forward(2);
      keyPosition--;
      break;
    case 3:
      if(startOp) { // change the angle to "face" home
        turnRight(180);
        startOp = false;
      }
      forward(5);
      keyPosition = 0;
      break;
    case 2:
      if(startOp) { // change the angle to "face" home
        turnLeft(180);
        startOp = false;
      }
      forward(4);
      turnLeft(90);
      forward(4);
      keyPosition = 0;
      break;
    case 1:
      //if(startOp) { // change the angle to "face" home
      backward(5);
      backward(2);
      keyPosition = 0;
      startOp = false;
      //}
      break;
    case 0:
      currentObjective = C;
      break;

      
  }
}

void completeOperation() {
  PrintMessage("CMD_CLOSE");
}



void forward(float distance) {
  if(distance== 0.5)
  {
    distance= (distance/1.04) + 0.001;
  }
  else if(distance== 1)
  {
    distance= (distance/1.04) + 0.002;
  }
  else if(distance== 1.5)
  {
    distance= (distance/1.04) + 0.002;
  }
  else if(distance== 2)
  {
    distance= (distance/1.04) + 0.003;
  }
  else if(distance== 2.5)
  {
    distance= (distance/1.04) + (0.05/1.04);
  }
  else if(distance== 3)
  {
    distance= (distance/1.04) + (0.062/1.04);
  }
  else if(distance== 3.5)
  {
    distance= (distance/1.04) + (0.063/1.04);
  }
  else if(distance== 4)
  {
    distance= (distance/1.04) + (0.073/1.04);
  }
  else if(distance== 4.5)
  {
    distance= (distance/1.04) + (0.083/1.04);
  }
  else if(distance== 5)
  {
    distance= (distance/1.04) + (0.094/1.04);
  }
  else
  {
    return;
  }
  PrintMessage("CMD_ACT_LAT_1_" + (String) distance);
}

void backward(float distance) {
  if(distance== 0.5)
  {
    distance= (distance/1.04) + 0.001;
  }
  else if(distance== 1)
  {
    distance= (distance/1.04) + 0.002;
  }
  else if(distance== 1.5)
  {
    distance= (distance/1.04) + 0.002;
  }
  else if(distance== 2)
  {
    distance= (distance/1.04) + 0.003;
  }
  else if(distance== 2.5)
  {
    distance= (distance/1.04) + (0.05/1.04);
  }
  else if(distance== 3)
  {
    distance= (distance/1.04) + (0.062/1.04);
  }
  else if(distance== 3.5)
  {
    distance= (distance/1.04) + (0.063/1.04);
  }
  else if(distance== 4)
  {
    distance= (distance/1.04) + (0.073/1.04);
  }
  else if(distance== 4.5)
  {
    distance= (distance/1.04) + (0.083/1.04);
  }
  else if(distance== 5)
  {
    distance= (distance/1.04) + (0.094/1.04);
  }
  else
  {
    return;
  }
  PrintMessage("CMD_ACT_LAT_0_" + (String) distance);
}

void turnLeft(float angle) {
  for(int i=0; i<angle*2; i++)
  {
    PrintMessage("CMD_ACT_ROT_0_0.5");
  }
  currentAngle = currentAngle - angle;
}

void turnRight(float angle) {
  for(int i=0; i<angle*2; i++)
  {
    PrintMessage("CMD_ACT_ROT_1_0.5");
  }
  currentAngle = currentAngle + angle;
}
