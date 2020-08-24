#include <avr/io.h>
#include <avr/interrupt.h>


volatile unsigned long int milliseconds = 0;
volatile float sensorValue = 0;

ISR(TIMER2_OVF_vect) { //Chapter 16
  //Register size = 256
  // CLK = 62500 Hz
  //Timer pertick = 1/CLK = 0.016ms
  // from 0 to 255 = 256 * 0.016ms = 4.096ms
  //to turn on and off every 250 ms, you do 250/4.096 = 61ish 
  milliseconds += 4; //4.096 milliseconds for prescaler 256 to fill up
}
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  timer_Init();
  ADC_Init();

}

void loop() {
  // put your main code here, to run repeatedly:
  sensorValue = myAnalogRead(1);
  sensorValue = ((sensorValue*5/1023)*(-53.039))+139.67;
  Serial.println(sensorValue);
  
  mydelay(500);

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


void ADC_Init() {
  ADCSRA = (1<<ADEN); // Enable ADC
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
