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

#define ZeroCrossPin 23
#define RedLedPin 4
#define YellowLedPin 5
#define GreenLedPin 8
#define HeatingElementPin 6
#define UserButtonPin 3

hw_timer_t *timer = NULL;

void ARDUINO_ISR_ATTR pump_stop_isr()
{
  Serial.println("Stopping the pump");
  // Turn off the pump
  digitalWrite(GreenLedPin, LOW); 
}

class SmokeMachine
{
public:
  SmokeMachine(uint8_t reqPin) : PIN(reqPin)
  {
    pinMode(PIN, INPUT_PULLUP);
  };

  void begin()
  {
    attachInterrupt(PIN, std::bind(&SmokeMachine::isr, this), RISING);
    Serial.printf("Started SmokeMachine interrupt on pin %d\n", PIN);
  
  }

  ~SmokeMachine()
  {
    detachInterrupt(PIN);
  }

  bool check_temperature()
  {
    // Read temperature from sensor
    // temperature = random(0, 500);
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

  void start_pump()
  {
    // Turn on the pump
    digitalWrite(GreenLedPin, HIGH);
  }

  void ARDUINO_ISR_ATTR isr()
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
        // Turn on the pump
        start_pump();

        // start timer based ISR to turn off the pump later
        timerRestart(timer);
        timerAlarm(timer, 1500, false, 0);
      }
    }
  }


  void checkPressed()
  {
    if (pressed)
    {
      Serial.printf("SmokeMachine on pin %u has been pressed %lu times\n", PIN, numberKeyPresses);
      pressed = false;
    }
  }

private:
  const uint8_t PIN;
  volatile uint32_t numberKeyPresses;
  volatile bool pressed;
  uint16_t temperature = 0;
  uint16_t threshold = 500;
  bool heater_on = false;
};

SmokeMachine machine(ZeroCrossPin);

void setup()
{
  Serial.begin(115200);
  pinMode(RedLedPin, OUTPUT);
  pinMode(YellowLedPin, OUTPUT);
  pinMode(GreenLedPin, OUTPUT);

  // pinMode(HeatingElementPin, INPUT_PULLDOWN);
  pinMode(UserButtonPin, INPUT_PULLUP);
  
  Serial.println("Starting Functional Interrupt example.");
  machine.begin();

  timer = timerBegin(1000);
  timerAttachInterrupt(timer, &pump_stop_isr);
  // timerAlarm(timer, 10, false, 0);

  
  Serial.println("Setup done.");
}

void loop()
{
  // machine.checkPressed();
}
