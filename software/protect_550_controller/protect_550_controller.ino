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
#define GreenLedPin 21
#define YellowLedPin 20
#define RedLedPin 19

hw_timer_t *timer = NULL;
volatile bool pump_start_flag = false;
volatile bool pump_stop_flag = false;
volatile bool main_isr_flag = false;

// Global ISR handlers
void IRAM_ATTR pump_start_isr()
{
  pump_start_flag = true;
}

void IRAM_ATTR rising_isr_handler() 
{
  main_isr_flag = true;
}

void IRAM_ATTR falling_isr_handler() 
{
  pump_stop_flag = true;
}

class SmokeMachine
{
public:
  SmokeMachine(uint8_t risingPin, uint8_t fallingPin)
  : mRisingPin(risingPin)
  , mFallingpin(fallingPin)
  {
    pinMode(mRisingPin, INPUT_PULLDOWN);
    pinMode(mFallingpin, INPUT_PULLDOWN);
  };

  void begin()
  {
    attachInterrupt(mRisingPin, rising_isr_handler, RISING);
    attachInterrupt(mFallingpin, falling_isr_handler, FALLING);
    Serial.printf("Started SmokeMachine with interrupts on pins %d and %d\n", mRisingPin, mFallingpin);
  }

  ~SmokeMachine()
  {
    detachInterrupt(mRisingPin);
    detachInterrupt(mFallingpin);
  }

  bool check_temperature(bool currently_ready = false)
  {
    // Read temperature from sensor
    mMeasuredTemp = analogReadMilliVolts(TemperatureSensePin) * mMillivoltToCelcius;

    // Check if temperature is above threshold and machine is not ready
    // Then set it to ready
    if ((mMeasuredTemp > mHeatThreshold) && !currently_ready)
    {
      Serial.printf("Temperature is above threshold: %u\n", mMeasuredTemp);
      return true;
    }
    else if ((mMeasuredTemp < mPumpThreshold) && currently_ready)
    {
      Serial.printf("Temperature is below threshold: %u\n", mMeasuredTemp);
      return false;
    }
    else
    {
      Serial.printf("Temperature is within thresholds: %u\n", mMeasuredTemp);
      return currently_ready;
    }
  }

  void start_heater()
  {
    digitalWrite(RedLedPin, HIGH);
  }

  void stop_heater()
  {
    digitalWrite(RedLedPin, LOW);
  }

  int get_pump_delay()
  {
    int delayValue = 0;
    uint16_t measuredVolume = analogRead(VolumeCtrlPin);
    // Serial.println(mMeasuredVolume);

    // Map the potentiometer value to one of 4 values.
    // The +1 is to get 4 equal areas for the 0-3665 range.
    delayValue = map(measuredVolume, 0, mPotMax + 1, 0, 4);

    // set delay between 500 and 2000 milliseconds
    delayValue = (delayValue+1)*2;
    
    // Serial.printf("delay: %u\n", delayValue);
    return delayValue;
  }

  // Make all shared variables volatile
  volatile bool mReadyState = false;
  volatile uint16_t mMeasuredTemp = 0;
  const uint16_t mHeatThreshold = 270; // 270 degrees Celsius

private:
  const uint8_t mRisingPin;
  const uint8_t mFallingpin;
  uint16_t mPumpThreshold = 200; // 200 degrees Celsius
  uint16_t mPotMax = 3665; // Value corresponding to max volume setting on potmeter

  const double mHeatThermoGain = 231; // 10K and 220K = 221 ;)
  const uint16_t mRefTemp = 270;  // 270 degrees Celsius measured as ref
  const double mRefTempVoltage = 14.7; // 14.7 mV @ mHeatThreshold
  const double mMillivoltToCelcius = mRefTemp / ( mRefTempVoltage * mHeatThermoGain); //
};

// Global instance
SmokeMachine machine(ZeroCrossPin1, ZeroCrossPin2);

void setup()
{
  Serial.begin(115200);
  pinMode(RedLedPin, OUTPUT);
  pinMode(YellowLedPin, OUTPUT);
  pinMode(GreenLedPin, OUTPUT);

  pinMode(UserButtonPin, INPUT_PULLUP);

  analogSetAttenuation(ADC_11db);

  machine.begin();

  timer = timerBegin(1000);
  timerAttachInterrupt(timer, &pump_start_isr);

  Serial.println("Setup done.");
}

void loop()
{
  // Handle pump start event (from timer ISR)
  if (pump_start_flag) {
    digitalWrite(GreenLedPin, HIGH);
    pump_start_flag = false;
  }
  // Handle pump stop event (from falling edge ISR)
  if (pump_stop_flag) {
    digitalWrite(GreenLedPin, LOW);
    pump_stop_flag = false;
  }

  // Handle main logic from main_isr (rising edge ISR)
  if (main_isr_flag) {
    // All heavy work is done here, not in ISR!
    machine.mReadyState = machine.check_temperature(machine.mReadyState);
    digitalWrite(YellowLedPin, machine.mReadyState ? HIGH : LOW);

    // turn off heater if temp is above threshold
    if (machine.mMeasuredTemp > machine.mHeatThreshold)
    {
      machine.stop_heater();
    }
    else
    {
      machine.start_heater();
    }

    if (digitalRead(UserButtonPin) == LOW)
    {
      Serial.println("User pressing button.");

      if (machine.mReadyState == true)
      {
        Serial.println("Starting the pump.");
        int pump_delay = machine.get_pump_delay();
        // start timer based ISR to turn on the pump later
        timerRestart(timer);
        timerAlarm(timer, pump_delay, false, 0);
      }
    }
    main_isr_flag = false;
  }
}
