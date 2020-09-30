#include <LiquidCrystal.h>
#include <avr/io.h>
#include <avr/interrupt.h>


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

//ISR(USART_TX_vect) {
//  if(serialReadPos != serialWritePos) {
//    UDR0 = txBuffer[serialReadPos];
//    serialReadPos++;
//
//    if(serialReadPos >= TX_BUFFER_SIZE) {
//      serialReadPos =0;
//    }
//  }
//}
//
//ISR(USART_RX_vect) {
//  rxBuffer[rxWritePos] = UDR0;
//
//  rxWritePos++;
//
//  if(rxWritePos >= RX_BUFFER_SIZE) {
//    rxWritePos = 0;
//  }
//}

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
}

void serial_Init() {
  //Using USART UART
  UBRR0H = (BRC >> 8);
  UBRR0L = BRC;

  //TX ENABLE
  UCSR0B = (1 << TXEN0) | (1 << TXCIE0) | (1 << RXEN0) | (1 << RXCIE0);
  UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);

  //RX ENABLE
  //  UCSR0B = (1 << RXEN0) | (1 << RXCIE0);


}

char getChar() {
  char ret = '\0';

  if(rxReadPos != rxWritePos) {
    ret = rxBuffer[rxReadPos];

    rxReadPos++;

    if(rxReadPos >= RX_BUFFER_SIZE) {
      rxReadPos = 0;
    }
  }

  return ret;
}



void appendSerial(char c) {
 txBuffer[serialWritePos] = c;
 serialWritePos++;

 if(serialWritePos >= TX_BUFFER_SIZE) {
  serialWritePos = 0;
 }

}

void serialWrite(char c[]) {
  for(uint8_t i = 0; i < strlen(c); i++) {
    appendSerial(c[i]);
  }

  if(UCSR0A & (1 << UDRE0)) {
    UDR0 = 0;
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
        mydelay(30);
        PrintMessage("CMD_SEN_IR");
        mydelay(30);
        currentString = Serial.readString();
        cutString = currentString.length();
        currentString.remove(cutString-2);
        irValue = currentString.toFloat();
        
        if(irValue != irValue) { // if NaN
          irValue = 5; // more than sensor range
        }
        if((irValue <= minimum) ||(i == 0)) {
          minimumAngle = i*5;
          minimum = irValue;
        }
        //lcd.print(irValue[i]);
      }
      finished = true;
      minimumAngle = 359 - minimumAngle;
      for(int j = 0; j <= minimumAngle; j++) {
        mydelay(20);
        PrintMessage("CMD_ACT_ROT_0_1");
      }
      PrintMessage("CMD_SEN_ROT_0");
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


enum Direction {
  LEFT,
  RIGHT
};


void wallModeOperation() {
  static boolean finished = false;
  static int minimumAngle = 0;
  static float minimum;
  String currentString;
  static float currentValue = 0;
  static float irValue;
  static int angle = 0;
  int distance = 0;
  int cutString;
  static Direction currentDirection = RIGHT;
  static boolean robotPosition;
  if(!finished) {
    PrintMessage("CMD_SEN_ROT_" + (String) (360 - (angle*5)));
    PrintMessage("CMD_SEN_IR");
    currentString = Serial.readString();
    cutString = currentString.length();
    currentString.remove(cutString-2);
    irValue = currentString.toFloat();
    
    if(irValue != irValue) { // if NaN
      irValue = 5; // more than sensor range
    }
    if((irValue <= minimum) ||(angle == 0)) {
      minimumAngle = angle*5;
      minimum = irValue;
    }
    if(angle == 72) {
      minimumAngle = 359 - minimumAngle;
      for(int j = 0; j <= minimumAngle; j++) {
        mydelay(20);
        PrintMessage("CMD_ACT_ROT_0_1");
      }
      PrintMessage("CMD_SEN_ROT_0"); 
      if(minimum >= 2) {
        PrintMessage("CMD_ACT_LAT_1_" + (String) (minimum - 2));
      }
      else {
        PrintMessage("CMD_ACT_LAT_0_" + (String) (2-minimum));
      }
      PrintMessage("CMD_ACT_ROT_0_90");
//      PrintMessage("CMD_SEN_ROT_90");
//      PrintMessage("CMD_SEN_IR");
//      currentString = Serial.readString();
//      cutString = currentString.length();
//      currentString.remove(cutString-2);
//      irValue = currentString.toFloat(); 
//      if(irValue != irValue) {
//        robotPosition = true; // true when it is in the upper or left side of the map
//      }
//      else {
//        robotPosition = false;
//      }
      finished = true;     
    }
    angle++;
  }
  else {
//    if(robotPosition) {
//      
//    }
//    else {
      switch(currentDirection) {
        case LEFT:
          PrintMessage("CMD_ACT_LAT_1_0.25");
          PrintMessage("CMD_SEN_ROT_90");
          PrintMessage("CMD_SEN_IR");
          currentString = Serial.readString();
          cutString = currentString.length();
          currentString.remove(cutString-2);
          irValue = currentString.toFloat();
          PrintMessage("CMD_ACT_ROT_0_90");
          if(irValue >= 2) {
            PrintMessage("CMD_ACT_LAT_1_" + (String) (irValue - 2));
          }
          else {
            PrintMessage("CMD_ACT_LAT_0_" + (String) (2- irValue));
          }
          PrintMessage("CMD_ACT_ROT_1_90");
          PrintMessage("CMD_SEN_ROT_0");
          PrintMessage("CMD_SEN_IR");
          currentString = Serial.readString();
          cutString = currentString.length();
          currentString.remove(cutString-2);
          irValue = currentString.toFloat();
          if(irValue <= 2.2) {
            PrintMessage("CMD_ACT_ROT_0_180");
            currentDirection = RIGHT;
          }
          break;
        case RIGHT:
          PrintMessage("CMD_ACT_LAT_1_0.25");
          PrintMessage("CMD_SEN_ROT_270");
          PrintMessage("CMD_SEN_IR");
          currentString = Serial.readString();
          cutString = currentString.length();
          currentString.remove(cutString-2);
          irValue = currentString.toFloat();
          PrintMessage("CMD_ACT_ROT_1_90");
          if(irValue >= 2) {
            PrintMessage("CMD_ACT_LAT_1_" + (String) (irValue - 2));
          }
          else {
            PrintMessage("CMD_ACT_LAT_0_" + (String) (2- irValue));
          }
          PrintMessage("CMD_ACT_ROT_0_90");
          PrintMessage("CMD_SEN_ROT_0");
          PrintMessage("CMD_SEN_IR");
          currentString = Serial.readString();
          cutString = currentString.length();
          currentString.remove(cutString-2);
          irValue = currentString.toFloat();
          if(irValue <= 2.2) {
            PrintMessage("CMD_ACT_ROT_0_180");
            currentDirection = LEFT;
          }
          break;
      }      
    //}

  }
  switch(whatbuttons) {
    case btnUP:
      minimumAngle = minimum = currentValue = angle = 0;
      break;
    case btnSELECT:
      finished = false;
      minimumAngle = minimum = currentValue = angle = 0;
      currentMode = mainMODE;
      currentMenuState = mainState;
      startup = true;
  }
  
}

void navModeOperation() {
  
}
