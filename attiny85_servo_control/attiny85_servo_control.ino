
/*
  Motorized hourglass 
  
  INCOMING SIGNALS TO Attiny85:
  -----------------------------
  RELEASE_PIN: Incoming active high to release the butterfly latch. Connected to a pin change interrupt on the ATtiny85.
  LOCK_PIN:  Incoming active high to lock the butterfly latch. Connected to a pin change interrupt on the ATtiny85.
  
  OUTGOING SIGNALS FROM Attiny85:
  -----------------------------
  SERVO_OUT: Signal that rotates the servo 180 or 1 degrees to release or lock the butterfly latch.
  TEST_LED: Connected to an LED for debug/trobleshooting purposes. (Not required for the core operation)

  OPERATION: Most of the time the code does not do anything other than waiting for an interrupt either on RELEASE_PIN or LOCK_PIN.
  An incoming HIGH level on RELEASE_PIN, activates the SERVO_OUT to rotate servo motor to open the butterfly latch to release the butterflies.
  An incoming HIGH level on LOCK_PIN, activates the SERVO_OUT to rotate servo motor to close the butterfly latch to lock the butterflies in position.

  The assumption is RELEASE_PIN and LOCK_PIN inputs will never be activated at the same time by the user. This type of operation will result in unpredictable
  SERVO_OUT behavior.

  by circuitapps
  November 2023
*/

#include <avr/io.h>
#include "Servo8Bit.h"

// PORT B DEFINITIONS
#define PB5_PIN1 PB5
#define PB4_PIN3 PB4
#define PB3_PIN2 PB3
#define PB2_PIN7 PB2
#define PB1_PIN6 PB1
#define PB0_PIN5 PB0

#define RELEASE_PIN PB3_PIN2  // Incoming signal for releasing the butterfly latch
#define LOCK_PIN PB4_PIN3  // Incoming signal for locking the butterfly latch
#define LATCH_SERVO_OUT PB1_PIN6  // This is the signal going to release latch servo motor
#define FLAP_SERVO_OUT PB0_PIN5  // This is the signal going to release latch servo motor
//#define TEST_LED PB0_PIN5  // Used for visual testing with LED (for debug purposes only)

#define PC_INT_VECTOR PCINT0_vect  // On ATtiny85, there is ONE interrupt vector (i.e., one interrupt service routine) that gets called when ANY of the active interrupt pins are triggered !

#define RELEASE_POSITION 1
#define LOCK_POSITION 0

#define TRUE 1
#define FALSE 0

// Butterfly wing flapper parameters
#define IDLE_WING_ANGLE 5  // when the butterfly is sitting idle without wing flapping
#define MAX_WING_ANGLE_DEG 90  // maximum wing span
#define MIN_WING_ANGLE_DEG 20  // minimum wing span
#define MAX_DELAY_BETWEEN_WING_RESTORE_MSEC  400  // maximum wait time after opening the wings
#define MIN_DELAY_BETWEEN_WING_RESTORE_MSEC  75  // minimum wait time after opening the wings
#define MAX_DELAY_BETWEEN_FLAPS_MSEC  3000  // maximum wait until next flap
#define MIN_DELAY_BETWEEN_FLAPS_MSEC  1000  // minimum wait until next flap

//#define HIGH 1  already defined in Arduino header
//#define LOW 0   already defined in Arduino header

Servo8Bit latch_servo_obj;  // create a global latch servo object. 
Servo8Bit flap_servo_obj;  // create a global wing flap servo object. 
bool next_latch_position;  // Controls how the servo will move next.
bool current_latch_position;  // Keeps track of the existing latch position.
bool run_servo;  // Flag set by ISR and reset in main loop as par5t of servo control.

void pin_interrupt_config()
{
  cli();  // disable GLOBAL interrupts during set up (USE WITH CAUTION as timing functions such as millis(), micros(), delay() get disrupted)
  // Three pin interrupts are enabled below
  PCMSK |= (1 << RELEASE_PIN) | (1 << LOCK_PIN);  // RELEASE_PIN input and LOCK_PIN pins drive an interrupt
  GIFR  |= (1 << PCIF);  // clear any outstanding interrupts
  GIMSK &= ~(1 << PCIE);  // Pin Change Interrupts are DISABLED
  //pinMode(TEST_LED, OUTPUT); // LED pin is set as output
  pinMode(RELEASE_PIN, INPUT);  // Hourglass clock output accepted as input
  pinMode(LOCK_PIN, INPUT);  // Hourglass D10 signal accepted as input
  sei();  // enable GLOBAL interrupts after set up (last line of set up) (USE WITH CAUTION as timing functions such as millis(), micros(), delay() get enabled)
}

void enable_PC_interrupts()
{// Only enables Pin Change interrupt
  GIMSK |= (1 << PCIE);
}

void disable_PC_interrupts()
{// Only disables Pin Change interrupt
  GIMSK &= ~(1 << PCIE);
}

void latch_servo_rotate(void)
{
  if(current_latch_position != next_latch_position)
  {// Current and next positions are different. Servo activation required.
    if(next_latch_position == LOCK_POSITION)
    {// Locking the latch
      latch_servo_obj.write(179);  // 179 degree movement
    }
    else
    {// Releasing the latch
      latch_servo_obj.write(1);  // 1 degree movement (minimum is 0 degrees)
    }

    // Update current latch position before next round.
    current_latch_position = next_latch_position;
  }

}

void flap_wings(void)
{
  long flap_angle = random(MIN_WING_ANGLE_DEG, MAX_WING_ANGLE_DEG);
  unsigned long flap_delay = (unsigned long)random(MIN_DELAY_BETWEEN_WING_RESTORE_MSEC, MAX_DELAY_BETWEEN_WING_RESTORE_MSEC);

  flap_servo_obj.write(flap_angle);
  delay(flap_delay);
  flap_servo_obj.write(IDLE_WING_ANGLE);

}

ISR(PC_INT_VECTOR)
{// Pin change interrupt routine

  if( digitalRead(LOCK_PIN) == HIGH)
  { // Butterfly latch needs to move to release position
      run_servo = TRUE;  // signal to main loop to enable servo rotation
      next_latch_position = LOCK_POSITION;  // servo needs to move to lock position
      //digitalWrite(TEST_LED, HIGH);  // for troubleshooting if necessary
  }

  if( digitalRead(RELEASE_PIN) == HIGH)
  { // Butterfly latch needs to move to release position
      run_servo = TRUE;  // signal to main loop to enable servo rotation
      //digitalWrite(TEST_LED, LOW);  // for troubleshooting if necessary
      next_latch_position = RELEASE_POSITION;  // servo needs to move to release position
  }

}

// the setup function runs once when you press reset or power the board
void setup()
{
  // At start up, latch is moved to lock position by default.
  next_latch_position = LOCK_POSITION;  // Controls how the servo will move next.
  current_latch_position = RELEASE_POSITION;  // Needs to be different to next_latch_position
  run_servo = FALSE;  // Main loop will not activate the servo at startup.

  flap_servo_obj.attach(FLAP_SERVO_OUT);    //attach the wing flap servo to pin PB0 (pin 5)

  latch_servo_obj.attach(LATCH_SERVO_OUT);  //attach the latch servo to pin PB1 (pin 6)
  latch_servo_rotate();  // servo position reset. Internally checks next_latch_position and current_latch_position variables
  delay(150);  // servo positioning delay
  pin_interrupt_config();  // set up interrupts
  enable_PC_interrupts();  // enable pin change interrupts
}


// the loop function runs over and over again forever
void loop()
{

  if(run_servo == TRUE)
  {// ISR requested servo activation.
    disable_PC_interrupts();  // disable pin change interrupts while activating the servo.
    latch_servo_rotate();  // rotation direction will be checked internally
    
    delay(150);  // This is to give time to the servo to activate and complete its rotation. Critically important to include here !

    run_servo = FALSE;  // one rotation executed. Flag inverted.
    enable_PC_interrupts();  // enable interrupt to wait for stabilization before the rotation
  }
  else
  {// Flap butterfly wings using another servo
    flap_wings();
    unsigned long flap_delay = (unsigned long)random(MIN_DELAY_BETWEEN_FLAPS_MSEC, MAX_DELAY_BETWEEN_FLAPS_MSEC);
    delay(flap_delay);  // wait before next flap event
  }

}
