/*
 * This example demonstrates usage of interrupt by detecting a button press.
 *
 * Setup: Connect first button between pin defined in BUTTON1 and GND
 *        Similarly connect second button between pin defined in BUTTON2 and GND.
 *        If you do not have a button simply connect a wire to those buttons
 *        - touching GND pin with other end of the wire will behave same as pressing the connected button.
 *        Wen using the bare wire be careful not to touch any other pin by accident.
 *
 * Note: There is no de-bounce implemented and the physical connection will normally
 *       trigger many more button presses than actually happened.
 *       This is completely normal and is not to be considered a fault.
 */

#include <Arduino.h>
#include <FunctionalInterrupt.h>
#include "esp_system.h"

#define PotentiometerPin 4
#define ZeroCrossPin1 22
#define ZeroCrossPin2 23
#define RedLedPin 21
#define YellowLedPin 20
#define GreenLedPin 19
#define HeatingElementPin 6
#define UserButtonPin 3

hw_timer_t *timer = NULL;

void ARDUINO_ISR_ATTR pump_start_isr()
{
  Serial.println("Starting the pump");
  // Turn on the pump
  digitalWrite(GreenLedPin, HIGH);
}

class SmokeMachine
{
public:
  SmokeMachine(uint8_t risingPin, uint8_t fallingPin)
  : risingPin(risingPin)
  , fallingPin(fallingPin)
  {
    pinMode(risingPin, INPUT_PULLUP);
    pinMode(fallingPin, INPUT_PULLUP);
  };

  void begin()
  {
    attachInterrupt(risingPin, std::bind(&SmokeMachine::main_isr, this), RISING);
    attachInterrupt(fallingPin, std::bind(&SmokeMachine::pump_stop_isr, this), FALLING);
    Serial.printf("Started SmokeMachine interrupt on pins %d and %d\n", risingPin, fallingPin);
  }

  ~SmokeMachine()
  {
    detachInterrupt(risingPin);
    detachInterrupt(fallingPin);
  }

  bool check_temperature()
  {
    // Read temperature from sensor
    temperature = analogRead(HeatingElementPin);

    // Check if temperature is above threshold
    if (temperature > threshold)
    {
      // Serial.printf("Temperature is above threshold: %u\n", temperature);
      return true;
    }
    else
    {
      // Serial.printf("Temperature is below threshold: %u\n", temperature);
      return false;
    }
  }

  void start_heater()
  {
    // Turn on heater
    digitalWrite(RedLedPin, HIGH);
    heater_on = true;
  }

  void stop_heater()
  {
    // Turn off heater
    digitalWrite(RedLedPin, LOW);
    heater_on = false;
  }

  int get_pump_delay()
  {
    int delayValue = 0;
    // Read the potentiometer value
    potValue = analogRead(PotentiometerPin);

    // The max potentiometer value depends on the voltage read when the potmeter 
    // is turned to max. 
    int potMax = 3665;

    // Map the potentiometer value to one of 4 values.
    delayValue = map(potValue, 0, potMax, 0, 3);

    // set delay between 500 and 2000 milliseconds
    // Serial.printf("Potmeter reads: %u -> delay: %u\n", potValue, (delayValue+1)*500);
    return (delayValue+1)*500;


  void ARDUINO_ISR_ATTR pump_stop_isr()
  {
    Serial.println("Stopping the pump");
    // Turn off the pump
    digitalWrite(GreenLedPin, LOW);
  }

  void ARDUINO_ISR_ATTR main_isr()
  {
    pressed = true;
    numberKeyPresses++;
    // Serial.printf("isr activated");
    // Check if temperature is above threshold so machine is ready
    bool temp_ready = check_temperature();

    if (temp_ready)
    {
      // Turn on a status LED to indicate the machine is ready
      digitalWrite(YellowLedPin, HIGH);

      // turn off heater if is on
      if (heater_on)
      {
        stop_heater();
      }
    }
    else
    {
      // Turn off the status LED to indicate the machine is not ready
      digitalWrite(YellowLedPin, LOW);

      // Turn on heater if is off
      if (!heater_on)
      {
        start_heater();
      }
    }

    // Serial.printf("user button state is: %u\n", digitalRead(UserButtonPin));
    // Check if the button is pressed
    if (digitalRead(UserButtonPin) == LOW)
    {
      // If the button is pressed and the machine is ready, turn on the machine
      Serial.println("User pressing button.");

      if (temp_ready)
      {
        int pump_delay = get_pump_delay();
        // start timer based ISR to turn on the pump later
        timerRestart(timer);
        timerAlarm(timer, pump_delay, false, 0);
      }
    }
  }

  void checkPressed()
  {
    if (pressed)
    {
      Serial.printf("SmokeMachine with risingPin on pin %u has been pressed %lu times\n", risingPin, numberKeyPresses);
      pressed = false;
    }
  }

private:
  const uint8_t risingPin;
  const uint8_t fallingPin;
  volatile uint32_t numberKeyPresses;
  volatile bool pressed;
  uint16_t temperature = 0;
  uint16_t potValue = 0;
  uint16_t threshold = 500;
  bool heater_on = false;
};

SmokeMachine machine(ZeroCrossPin1, ZeroCrossPin2);

void setup()
{
  Serial.begin(115200);
  pinMode(RedLedPin, OUTPUT);
  pinMode(YellowLedPin, OUTPUT);
  pinMode(GreenLedPin, OUTPUT);

  pinMode(UserButtonPin, INPUT_PULLUP);

  Serial.println("Starting Functional Interrupt example.");
  machine.begin();

  timer = timerBegin(1000);
  timerAttachInterrupt(timer, &pump_start_isr);

  Serial.println("Setup done.");
}

void loop()
{
  // machine.checkPressed();
}
