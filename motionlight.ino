#include <avr/io.h>
#include <avr/interrupt.h>

// light parameters
byte low  = 2;
byte high = 2047;

// pwm controls
#define PWM_rollover  2047
#define PWM_delay     127
unsigned int PWM_dc = low;  

// pin definitions
#define MOTION_PIN 0x01
#define LED_PIN    0x02

// LDR definitions
#define DAYLIGHT   100
#define NIGHTLIGHT 10

// state machine
#define STATE_LOW      0x01
#define STATE_TRANSIT  0x02
#define STATE_HIGH     0x04
#define STATE_DAYLIGHT 0x08
byte cur_state = STATE_DAYLIGHT;

// timeout control
unsigned long then     = 0;
unsigned long interval = 300;

void setup() 
{
  DDRB &= ~MOTION_PIN; // read pins
  DDRB |= LED_PIN;     // write pins
  PWM_setup();         // setup timer0 interrupts
}

void loop() 
{
  static unsigned long start = millis();
  unsigned long cur_millis = millis();
  unsigned long cur_seconds = cur_millis / 1000;
  static unsigned long old_seconds = cur_seconds;
  boolean motion = (PINB & MOTION_PIN);
  static int lightValue = analogRead(0);
  int new_dc;

  if ( cur_seconds - old_seconds > 5 )
  {
    lightValue = analogRead(0);
    old_seconds = cur_seconds;
  }

  if (motion)
    then = cur_seconds;
  
  switch(cur_state)
  {
    case STATE_LOW:
      if ( lightValue >= DAYLIGHT )
      {
        cur_state = STATE_DAYLIGHT;
      }
      else if (motion)
      {
        start = cur_millis;
        cur_state = STATE_TRANSIT;
      }
      break;
      
    case STATE_TRANSIT:
      new_dc = millis_to_dc( cur_millis - start );
      if ( new_dc >= high )
        cur_state = STATE_HIGH;
      else
        PWM_set_dc( new_dc );

    case STATE_HIGH:
      if ( lightValue >= DAYLIGHT )
      {
        cur_state = STATE_DAYLIGHT;
      }
      else if ( (cur_seconds - then) > interval )
      {
        cur_state = STATE_LOW;
        PWM_set_dc( low );
      }
      break;

    case STATE_DAYLIGHT:
      if ( lightValue <= NIGHTLIGHT )
      {
        cur_state = STATE_LOW;
        PWM_setup();
        PWM_set_dc( low );
      }
      else if ( PWM_dc > 0 ) // shutdown PWM functions only once
      {
        PWM_set_dc( 0 );
        PWM_shutdown();
      }
      break;
  }
  
  delay(20);
}

unsigned long millis_to_dc( unsigned long mils )
{
  float seconds = mils / 1000.0;
  return long(0.00035 * seconds * seconds * seconds) + low;  
}

void PWM_set_dc( unsigned int dc )
{
  if( dc < 0 )
    PWM_dc = 0;
  else if( dc > PWM_rollover )
    PWM_dc = PWM_rollover;
  else
    PWM_dc = dc;
}

void PWM_setup()
{
  cli();
  TCNT0   = 0;            // clear the counter
  OCR0A   = PWM_delay;    // set compare value
  TIMSK  |= _BV(OCIE0A);  // enable interrupt for compare match A   
  TCCR0A  = _BV(WGM01);   // CTC mode
  TCCR0B  = _BV(CS01);    // clk/8 prescaling
  sei();
}

void PWM_shutdown()
{
  cli();
  TCNT0  = 0;              // reset counter
  TIMSK &= ~(_BV(OCIE0A)); // disable interrupt
  PORTB &= ~LED_PIN;       // turn off LED
  sei();
}

ISR(TIMER0_COMPA_vect)
{
  static unsigned int cur_tick = 0;
  if( cur_tick > PWM_rollover ) 
    cur_tick = 0;

  if( cur_tick == 0 && PWM_dc > 0 )
    PORTB |= LED_PIN;   // turn LED on at beginning of duty cycle

  if( cur_tick == PWM_dc )
    PORTB &= ~LED_PIN;  // turn LED off at END of duty cycle

  cur_tick++;
}

