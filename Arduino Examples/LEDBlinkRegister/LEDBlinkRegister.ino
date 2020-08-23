#include <avr/io.h>


int count = 0; 


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  DDRB |= (1<<DDB5); // Pin 9 direction output
  
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

  

}

void loop() {
  // put your main code here, to run repeatedly:
  //Register size = 256
  // CLK = 62500 Hz
  //Timer pertick = 1/CLK = 0.016ms
  // from 0 to 255 = 256 * 0.016ms = 4.096ms
  //to turn on and off every 250 ms, you do 250/4.096 = 61ish


  if(TCNT2 >= 255) {
    count++;
    TCNT2 =0;

    if(count >= 244) {
      PORTB ^=(1<<PORTB5); //XOR the pin = HIGH OR LOW (TOGGLE)
      count =0;
    }
  }

  
}
