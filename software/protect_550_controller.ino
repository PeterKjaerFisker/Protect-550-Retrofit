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

#define ZeroCrossPin 20

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
    temperature = random(0, 500);

    // Check if temperature is above threshold
    if (temperature > threshold)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  void start_heater()
  {
    // Turn on heater
    // digitalWrite(11, HIGH);
    heater_on = true;
  }

  void stop_heater()
  {
    // Turn off heater
    // digitalWrite(11, LOW);
    heater_on = false;
  }

  void ARDUINO_ISR_ATTR isr()
  {
    // Check if temperature is above threshold so machine is ready
    bool temp_ready = check_temperature();

    if (temp_ready)
    {
      // Turn on a status LED to indicate the machine is ready
      // digitalWrite(13, HIGH);
      // turn off heater if is on
      if (heater_on)
      {
        stop_heater();
      }
    }
    else
    {
      // Turn off the status LED to indicate the machine is not ready
      // digitalWrite(13, LOW);

      // Turn on heater if is off
      if (!heater_on)
      {
        start_heater();
      }
    }

    // Check if the button is pressed
    if (digitalRead(PIN) == LOW)
    {
      // If the button is pressed and the machine is ready, turn on the machine
      if (temp_ready)
      {
        // Turn on the pump
        // start_pump();

        // start timer based ISR to turn off the pump later
        //
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
  uint8_t temperature = 0;
  uint8_t threshold = 250;
  bool heater_on = false;
};

SmokeMachine machine(ZeroCrossPin);

void setup()
{
  Serial.begin(115200);

  Serial.println("Starting Functional Interrupt example.");
  machine.begin();
  Serial.println("Setup done.");
}

void loop()
{
  machine.checkPressed();
}
