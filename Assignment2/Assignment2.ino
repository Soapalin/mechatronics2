#include <LiquidCrystal.h>
#include <avr/io.h>
#include <avr/interrupt.h>


LiquidCrystal lcd(8, 9, 4, 5, 6, 7); // Set up LCD

volatile unsigned long int milliseconds = 0;
volatile long unsigned int timer;
boolean blinking = true;


/*** IR sensor variables ***/
volatile float irHistory[5];
volatile int irValue =0;

/*** MOTOR VARIABLES ***/
int Steps;
int Direction = 0;
int StepTime = 2;
int previousStepTime = 0;
int motorSpeed = 2;
boolean startMotor = false;
int wheelSize = 20;
int stepSet = 100;
int stepCount = 0;

enum Mode {
  startupMODE, 
  debugMODE,
  irMODE,
  cmMODE,
  pmMODE,
  setMODE,
  driveMODE
};

Mode currentMode;

enum debugState {
  IR,
  CM,
  PM,
  SET,
  E
};

debugState currentDebugState = IR;

enum cmState {
  starting,
  exiting
};

cmState currentCMState = starting;


enum Buttons {
  btnNONE,
  btnRIGHT,
  btnUP,
  btnDOWN,
  btnLEFT,
  btnSELECT
};

int inputButton; 
Buttons whatbuttons;
Buttons buttonHistory[5] = { btnNONE, btnNONE, btnNONE, btnNONE, btnNONE };
Buttons debugSequence[5] = { btnLEFT, btnLEFT, btnUP, btnRIGHT, btnSELECT };



ISR(TIMER2_OVF_vect) { //Chapter 16
  //Register size = 256
  // CLK = 62500 Hz
  //Timer pertick = 1/CLK = 0.016ms
  // from 0 to 255 = 256 * 0.016ms = 4.096ms
  //to turn on and off every 250 ms, you do 250/4.096 = 61ish 
  milliseconds += 4; //4.096 milliseconds for prescaler 256 to fill up
  if(startMotor) { // CM Mode only atm
    stepperMotor(motorSpeed);
  }
}


void setup() {
  Serial.begin(9600);
  timer_Init();
  lcd_Init();
  ADC_Init();
  stepperMotor_Init();
  currentMode = startupMODE;

}

void loop() {
  switch(currentMode) {
    case startupMODE: 
      printClock(mymillis()/60000, (mymillis() / 1000) % 60);
      if(checkDebugSequence()) {
        currentMode = debugMODE;

      }
      else if ((whatbuttons == btnSELECT) && !(checkDebugSequence())) {
        currentMode = driveMODE;
      }
      break;
    case debugMODE:
      debugModeOperation();
      break;
    case irMODE:
      irModeOperation();
      break;
    case cmMODE:
      cmModeOperation();
      break;
    case pmMODE:
      pmModeOperation();
      break;
    case setMODE:
      setModeOperation();
      break;
    case driveMODE:
      driveModeOperation();
      break;
    default:
      break;
  }

  buttonSlidingWindow();



}



void timer_Init() {
  //Timer2 register A = normal operation
  TCCR2A &=~(1<<WGM20);
  TCCR2A &=~(1<<WGM21);
  TCCR2A &=~(1<<COM2B0);
  TCCR2A &=~(1<<COM2B1);
  TCCR2A &=~(1<<COM2A0);
  TCCR2A &=~(1<<COM2A1);

  // Prescaler for 256
  TCCR2B &= ~(1<<CS20);
  TCCR2B |= (1<<CS21);
  TCCR2B |= (1<<CS22);

  TIMSK2 |= (1<<TOIE2); //Enable Overflow interrupt
  sei(); //enable global interrupt
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

void printClock(int minutes, int seconds) {
  //lcd.clear();
  lcd.setCursor(0,1);
  lcd.print("ID:12878930"); 
  lcd.setCursor(0,0);
  if(minutes <10) {
    lcd.print('0');
  }
  lcd.print(minutes);
  lcd.print(':');
  if(seconds < 10) {
    lcd.print('0');
  }
  lcd.print(seconds);
}

void lcd_Init() {
  lcd.begin(16,2);
  lcd.clear();
}

Buttons readLCDButtons() {
  inputButton = myAnalogRead(0);
  // read analog 0 with registers
  if(inputButton > 1000) {
    mydelay(150);
    return btnNONE;
  }
  if(inputButton < 50) {
    mydelay(150);
    return btnRIGHT;
  }
  if(inputButton < 250) {
    mydelay(150);
    return btnUP;
  }
  if(inputButton < 450) {
    mydelay(150);
    return btnDOWN;
  }
  if(inputButton < 650) {
    mydelay(150);
    return btnLEFT;
  }
  if(inputButton < 850) {
    mydelay(150);
    return btnSELECT;
  }

  return btnNONE; // when all others fail, return this
}

void buttonSlidingWindow() {
  whatbuttons = readLCDButtons();
  if(whatbuttons != btnNONE) {
    for(int i = 0; i < 4; i++) {
      buttonHistory[i] = buttonHistory[i+1];
    }
    buttonHistory[4] = whatbuttons;
  }
}

boolean checkDebugSequence() {
  for(int i = 0; i <= 4; i++) {
    if(buttonHistory[i] != debugSequence[i]) {
      return false;
    }
  }
  return true;
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


void stepperMotor_Init() {
  DDRB |= (1<<DDB5); // Change data direction of pin 13 to output
  DDRB |= (1<<DDB4); // Change data direction of pin 12 to output
  DDRB |= (1<<DDB3); // Change data direction of pin 11 to output
  DDRD |= (1<<DDD3); // Change data direction of pin 3 to output
  
}

void stepperMotor(int xw) {
  for(int x =0; x< xw; x++) 
  {
    switch(Steps) 
    {
      case 0: //1000
        PORTB |= (1<<PORTB5); 
        PORTB &= ~(1<<PORTB4);
        PORTB &= ~(1<<PORTB3);
        PORTD &= ~(1<<PORTD3);   
        break;
      case 1: //1100
        PORTB |= (1<<PORTB5); 
        PORTB |= (1<<PORTB4);
        PORTB &= ~(1<<PORTB3);
        PORTD &= ~(1<<PORTD3);  
        break;
      case 2: //0100
        PORTB &= ~(1<<PORTB5); 
        PORTB |= (1<<PORTB4);
        PORTB &= ~(1<<PORTB3);
        PORTD &= ~(1<<PORTD3);    
        break;
      case 3: //0110
        PORTB &= ~(1<<PORTB5); 
        PORTB |= (1<<PORTB4);
        PORTB |= (1<<PORTB3);
        PORTD &= ~(1<<PORTD3);  
        break;
      case 4: //0010
        PORTB &= ~(1<<PORTB5); 
        PORTB &= ~(1<<PORTB4);
        PORTB |= (1<<PORTB3);
        PORTD &= ~(1<<PORTD3);   
        break;
      case 5: //0011
        PORTB &= ~(1<<PORTB5); 
        PORTB &= ~(1<<PORTB4);
        PORTB |= (1<<PORTB3);
        PORTD |= (1<<PORTD3);  
        break;
      case 6: //0001
        PORTB &= ~(1<<PORTB5); 
        PORTB &= ~(1<<PORTB4);
        PORTB &= ~(1<<PORTB3);
        PORTD |= (1<<PORTD3);  
        break;
      case 7: //1001
        PORTB |= (1<<PORTB5); 
        PORTB &= ~(1<<PORTB4);
        PORTB &= ~(1<<PORTB3);
        PORTD |= (1<<PORTD3); 
        break;
      default: //0000
        PORTB &= ~(1<<PORTB5); 
        PORTB &= ~(1<<PORTB4);
        PORTB &= ~(1<<PORTB3);
        PORTD &= ~(1<<PORTD3); 
        break;
    }
    setDirection();
  } 
}

void setDirection() {
  if(Direction ==1) {
    Steps++;
  }
  else {
    Steps--;
  }
  if(Steps > 7) {
    Steps = 0;
  }
  if(Steps < 0) {
    Steps = 7;
  }
}

void debugModeOperation() {
  lcd.setCursor(0,0);
  lcd.print("DEBUG Mode");

  if(blinking) {
    lcd.setCursor(0,1);
    lcd.print("IR CM PM SET E");
    timer = mymillis();
    blinking = false;
  }
  
  if(mymillis() >= (timer + 1000)) {
    blinking = true;
    lcd.setCursor(0,1);
    switch(currentDebugState) {
      case IR:
        lcd.print("   CM PM SET E");
        break;
      case CM:
        lcd.print("IR    PM SET E");
        break;
      case PM:
        lcd.print("IR CM    SET E");
        break;
      case SET:
        lcd.print("IR CM PM     E");
        break;
      case E:
        lcd.print("IR CM PM SET  ");
        break;
      default:
        break;
    }
  }

  switch(whatbuttons) {
    case btnLEFT:
      currentDebugState = currentDebugState - 1;
      if(currentDebugState < 0) {
        currentDebugState = 0;
      }
      break;
    case btnRIGHT: 
      currentDebugState = currentDebugState +1;
      if(currentDebugState > 4) {
        currentDebugState = 4;
      }
      break;
    case btnSELECT:
      debugModeSelection();
      blinking = true;
      currentDebugState = IR;
      lcd.clear();
      break;
  }
  
}

void debugModeSelection() {
  switch(currentDebugState) {
    case IR:
      currentMode = irMODE;
      break;
    case CM:
      currentMode = cmMODE;
      break;
    case PM:
      currentMode = pmMODE;
      break;
    case SET:
      currentMode = setMODE;
      break;
    case E:
      currentMode = startupMODE;
      break;
    default:
      break;
  }
}

enum driveModeState {
  idle,
  CW,
  CCW
};



void driveModeOperation() {
  static driveModeState currentDriveState = idle; 
  static int averaging =0;
  static volatile float sum =0;
  static volatile float sensorValue =0;
  static long unsigned int previousTime = 0; 
  static float revolutions = 0;
  static int stepRemaining = 0;
  static boolean startupCondition = true;
  
  
  if(startupCondition) {
    lcd.clear();
    startupCondition = false;
  }
  lcd.setCursor(0,0);
  lcd.print("Drive Mode");

  switch(currentDriveState) {
    case idle:
      if(averaging < 5) {
        if(mymillis() - previousTime >= 200) {
          averaging++;
          sensorValue = myAnalogRead(1);
          mydelay(50);
          sensorValue = ((sensorValue*5/1023)*(-53.039))+139.67;
          if(sensorValue < 0) {
            sensorValue = 0;
          }
          //Serial.println(sensorValue);
          sum += sensorValue; // found from excel line of best fit + datasheet
          previousTime = mymillis();
        }
      }
      else if(averaging >= 5){
          irValue = sum/averaging;
          lcd.setCursor(0,1);
          lcd.print("               ");
          lcd.setCursor(0,1);
          lcd.print(irValue);
          lcd.print(" ");
          revolutions = (float) irValue/ (float) wheelSize;
          lcd.print(revolutions, 1);
          lcd.print(" ");
          stepRemaining = revolutions * 4096;
          lcd.print(stepRemaining);
          averaging =0;
          sum = 0;
      }
      break;
    case CW:
      if(stepRemaining != 0) {
        stepperMotor(1);
        stepRemaining--;
        lcd.setCursor(0,1);
        lcd.print("              ");
        lcd.setCursor(0,1);
        lcd.print(irValue);
        lcd.print(" ");
        lcd.print(revolutions, 1);
        lcd.print(" ");
        lcd.print(stepRemaining);        
      }
      break;
    case CCW:
      if(stepRemaining != 0) {
        stepperMotor(1);
        stepRemaining--;
        lcd.setCursor(0,1);
        lcd.print("              ");
        lcd.setCursor(0,1);
        lcd.print(irValue);
        lcd.print(" ");
        lcd.print(revolutions, 1);
        lcd.print(" ");
        lcd.print(stepRemaining);        
      }

      break;
  }
  
  switch(whatbuttons) {
    case btnSELECT:
      currentMode = startupMODE;
      Direction = 0;
      currentDriveState = idle;
      startupCondition = true;
      lcd.clear();
      break;
    case btnUP:
      Direction =0;
      currentDriveState = CW;
      stepRemaining = revolutions*4096;
      break;
    case btnDOWN:
      Direction = 1;
      currentDriveState = CCW;
      stepRemaining = revolutions*4096;
      break;
    default:
      break;
  }
}


void irModeOperation() {
  static int averaging = 0;
  static volatile float sum = 0;
  static volatile float sensorValue = 0;
  static long unsigned int previousTime = 0;
  
  lcd.setCursor(0,0);
  lcd.print("IR Mode");

  if(averaging < 5) {
    if(mymillis() - previousTime >= 200) {
      averaging++;
      sensorValue = myAnalogRead(1);
      mydelay(50);
      sensorValue = ((sensorValue*5/1023)*(-53.039))+139.67;
      if(sensorValue < 0) {
        sensorValue = 0;
      }
      //Serial.println(sensorValue);
      sum += sensorValue; // found from excel line of best fit + datasheet
      previousTime = mymillis();
    }
  }
  else if(averaging >= 5){
      irValue = sum/averaging;
      lcd.setCursor(0,1);
      lcd.print("       ");
      lcd.setCursor(0,1);
      lcd.print(irValue);
      averaging =0;
      sum = 0;
  }
  

  if(whatbuttons == btnSELECT) {
    lcd.clear();
    averaging =0;
    sum = 0;
    currentMode = debugMODE;
  }

  
}


void setModeOperation() {
  lcd.setCursor(0,0);
  lcd.print("SETTINGS Mode");
  lcd.setCursor(0,1);
  lcd.print("Wheel: ");
  lcd.setCursor(7,1);
  lcd.print(wheelSize);
  lcd.setCursor(10, 1);
  lcd.print("cm");

  switch(whatbuttons) {
    case btnUP:
      wheelSize += 10;
      if(wheelSize > 90) {
        wheelSize = 90;
      }
      break;
    case btnDOWN:
      wheelSize -= 10;
      if(wheelSize < 10) {
        wheelSize = 10;
      }
      break;
    case btnSELECT:
      lcd.clear();
      currentMode = debugMODE;
      break;
  }  
}

void pmModeOperation() {
  static boolean pmStart = false;
  lcd.setCursor(0,0);
  lcd.print("PM Mode");
  lcd.setCursor(0,1);
  lcd.print(stepSet);
  lcd.setCursor(4, 1);
  lcd.print(stepSet);
  
  if(!pmStart) {
    switch(whatbuttons) {
      case btnUP:
        stepSet += 100;
        if(stepSet > 900) {
          stepSet = 900;
        }
        break;
      case btnDOWN:
        stepSet -= 100;
        if(stepSet < 100) {
          stepSet = 100;
        }
        break;
      case btnLEFT:
        stepSet = 100;
        break;
      case btnRIGHT:
        pmStart = true;
        break;
      case btnSELECT:
        lcd.clear();
        currentMode = debugMODE;
    }  
  }
  else {
    static int stepRemaining = stepSet;
    if(stepRemaining != 0) {
      stepperMotor(1);
      stepCount++;
    }
    
    stepRemaining = stepSet - stepCount;
    
    lcd.setCursor(4,1);
    lcd.print(stepRemaining);
    if((stepRemaining) < 100) {
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("PM Mode");
      lcd.setCursor(0,1);
      lcd.print(stepSet);
      lcd.setCursor(4, 1);
      lcd.print(stepRemaining);   
    }
    if(stepRemaining == 0)
    if(whatbuttons == btnSELECT) {
      lcd.clear();
      pmStart = false;
      currentMode = debugMODE;
    }
    
  }

  
}


void cmModeOperation() {
  lcd.setCursor(0,0);
  lcd.print("CM Mode");
  if(blinking) {
    lcd.setCursor(0,1);
    lcd.print("Start Exit");
    timer = mymillis();
    blinking = false;
  }

  if(mymillis() >= (timer + 1000) && startMotor == false) {
    blinking = true;
    lcd.setCursor(0,1);
    switch(currentCMState) {
      case starting:
        lcd.print("      Exit");
        break;
      case exiting:
        lcd.print("Start     ");
        break;
    }
  }
  
  if(!startMotor) {
    switch(whatbuttons) {
      case btnLEFT:
        currentCMState = starting;
        break;
      case btnRIGHT:
        currentCMState = exiting;
        break;
      case btnSELECT:
        if(currentCMState == exiting) {
          blinking = true;
          currentCMState = starting;
          lcd.clear();
          currentMode = debugMODE;
        }
        else {
          startMotor = true;
          blinking = false;
          lcd.clear();
          lcd.setCursor(0,0);
          lcd.print("CM Mode");

        }
        break;
    }
  }
  else {
     lcd.setCursor(0,1);
    if(Direction == 0) {
      lcd.print("CW");
      lcd.print(" Speed ");
      lcd.print(motorSpeed);
    }
    else {
      lcd.print("CCW");
      lcd.print(" Speed ");
      lcd.print(motorSpeed);
    } 
    switch(whatbuttons) {
      case btnLEFT: //anticlockwise
        Direction = 1;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("CM Mode");       
        break;
      case btnRIGHT: // clockwise
        Direction = 0;
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("CM Mode");
        break;
      case btnUP:

        motorSpeed++;
        if(motorSpeed > 3) {
          motorSpeed = 3;
        }
        break;
      case btnDOWN:
        motorSpeed--;
        if(motorSpeed < 1) {
          motorSpeed = 1;
        }
        break;
      case btnSELECT:
        motorSpeed = 2;
        Direction = 0;
        startMotor = false;
        blinking = true;
        break;
    }
    
  }

}
