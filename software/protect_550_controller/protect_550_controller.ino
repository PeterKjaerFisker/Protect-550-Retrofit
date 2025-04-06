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

#define UserButtonPin 3
#define VolumeCtrlPin 5
#define TemperatureSensePin 6
#define ZeroCrossPin1 22
#define ZeroCrossPin2 23
#define GreenLedPin 19
#define YellowLedPin 20
#define RedLedPin 21

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
  
  bool check_temperature(bool currently_ready = false)
  {
    // Read temperature from sensor
    temperature = analogRead(TemperatureSensePin);

    // Check if temperature is above threshold and machine is not ready
    // Then set it to ready
    if ((temperature > heat_threshold) && !currently_ready)
    {
      // Serial.printf("Temperature is above threshold: %u\n", temperature);
      return true;
    }
    else if ((temperature < pump_threshold) && currently_ready)
    {
      // Serial.printf("Temperature is below threshold: %u\n", temperature);
      return false;
    }
    else
    {
      // Serial.printf("Temperature is below threshold: %u\n", temperature);
      return currently_ready;
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
    potValue = analogRead(VolumeCtrlPin);

    // The max potentiometer value depends on the voltage read when the potmeter 
    // is turned to max. 
    int potMax = 3665;

    // Map the potentiometer value to one of 4 values.
    // The +1 is to get 4 equal areas for the 0-3665 range.
    delayValue = map(potValue, 0, potMax + 1, 0, 4);

    // set delay between 500 and 2000 milliseconds
    // Serial.printf("Potmeter reads: %u -> delay: %u\n", potValue, (delayValue+1)*500);
    return (delayValue+1)*500;
  }

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
    ready_state = check_temperature(ready_state);
    digitalWrite(YellowLedPin, ready_state ? HIGH : LOW);

    // turn off heater if is on
    if (temperature > heat_threshold)
    {
      stop_heater();
    }
    else
    {
      start_heater();
    }

    // Serial.printf("user button state is: %u\n", digitalRead(UserButtonPin));
    // Check if the button is pressed
    if (digitalRead(UserButtonPin) == LOW)
    {
      // If the button is pressed and the machine is ready, turn on the machine
      Serial.println("User pressing button.");

      if (ready_state == true)
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
  uint16_t heat_threshold = 2500;
  uint16_t pump_threshold = 1500;
  bool heater_on = false;
  bool ready_state = false;
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
