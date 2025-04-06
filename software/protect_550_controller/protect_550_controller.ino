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
  : mRisingPin(risingPin)
  , mFallingpin(fallingPin)
  {
    pinMode(mRisingPin, INPUT_PULLUP);
    pinMode(mFallingpin, INPUT_PULLUP);
  };

  void begin()
  {
    attachInterrupt(mRisingPin, std::bind(&SmokeMachine::main_isr, this), RISING);
    attachInterrupt(mFallingpin, std::bind(&SmokeMachine::pump_stop_isr, this), FALLING);
    Serial.printf("Started SmokeMachine interrupt on pins %d and %d\n", mRisingPin, mFallingpin);
  }

  ~SmokeMachine()
  {
    detachInterrupt(mRisingPin);
    detachInterrupt(mFallingpin);
  }
  
  bool check_temperature(bool currently_ready = false)
  {
    // Read temperature from sensor
    mMeasuredTemp = analogRead(TemperatureSensePin);

    // Check if temperature is above threshold and machine is not ready
    // Then set it to ready
    if ((mMeasuredTemp > mHeatThreshold) && !currently_ready)
    {
      // Serial.printf("Temperature is above threshold: %u\n", temperature);
      return true;
    }
    else if ((mMeasuredTemp < mPumpThreshold) && currently_ready)
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
    // start_heater();
  }
  
  void stop_heater()
  {
    // Turn off heater
    digitalWrite(RedLedPin, LOW);
    // stop_heater();
  }

  int get_pump_delay()
  {
    int delayValue = 0;
    // Read the potentiometer value
    mMeasuredVolume = analogRead(VolumeCtrlPin);

    // The max potentiometer value depends on the voltage read when the potmeter 
    // is turned to max. 
    int potMax = 3665;

    // Map the potentiometer value to one of 4 values.
    // The +1 is to get 4 equal areas for the 0-3665 range.
    delayValue = map(mMeasuredVolume, 0, potMax + 1, 0, 4);

    // set delay between 500 and 2000 milliseconds
    // Serial.printf("Potmeter reads: %u -> delay: %u\n", mMeasuredVolume, (delayValue+1)*500);
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
    mReadyState = check_temperature(mReadyState);
    digitalWrite(YellowLedPin, mReadyState ? HIGH : LOW);

    // turn off heater if is on
    if (mMeasuredTemp > mHeatThreshold)
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

      if (mReadyState == true)
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
      Serial.printf("SmokeMachine with risingPin on pin %u has been pressed %lu times\n", mRisingPin, numberKeyPresses);
      pressed = false;
    }
  }

private:
  volatile uint32_t numberKeyPresses;
  volatile bool pressed;
  
  bool mReadyState = false;
  const uint8_t mRisingPin;
  const uint8_t mFallingpin;
  uint16_t mMeasuredTemp = 0;
  uint16_t mMeasuredVolume = 0;
  uint16_t mHeatThreshold = 2500;
  uint16_t mPumpThreshold = 1500;
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
