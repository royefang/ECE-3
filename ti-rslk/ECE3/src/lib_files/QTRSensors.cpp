#include "QTRSensors.h"
#include <Arduino.h>


void QTRSensors::setSensorPins(const uint8_t * pins, uint8_t sensorCount)
{
  if (sensorCount > QTRMaxSensors) { sensorCount = QTRMaxSensors; }

  // (Re)allocate and initialize the array if necessary.
  uint8_t * oldSensorPins = _sensorPins;
  _sensorPins = (uint8_t *)realloc(_sensorPins, sizeof(uint8_t) * sensorCount);
  if (_sensorPins == nullptr)
  {
    // Memory allocation failed; don't continue.
    free(oldSensorPins); // deallocate any memory used by old array
    return;
  }

  for (uint8_t i = 0; i < sensorCount; i++)
  {
    _sensorPins[i] = pins[i];
  }

  _sensorCount = sensorCount;

}

void QTRSensors::setTimeout(uint16_t timeout)
{
  if (timeout > 32767) { timeout = 32767; }
  _timeout = timeout;
  _maxValue = timeout; 
}


void QTRSensors::setEmitterPins(uint8_t oddEmitterPin, uint8_t evenEmitterPin)
{
  releaseEmitterPins();

  _oddEmitterPin = oddEmitterPin;
  _evenEmitterPin = evenEmitterPin;
  pinMode(_oddEmitterPin, OUTPUT);
  pinMode(_evenEmitterPin, OUTPUT);

  _emitterPinCount = 2;
}

void QTRSensors::releaseEmitterPins()
{
  if (_oddEmitterPin != QTRNoEmitterPin)
  {
    pinMode(_oddEmitterPin, INPUT);
    _oddEmitterPin = QTRNoEmitterPin;
  }

  if (_evenEmitterPin != QTRNoEmitterPin)
  {
    pinMode(_evenEmitterPin, INPUT);
    _evenEmitterPin = QTRNoEmitterPin;
  }

  _emitterPinCount = 0;
}

void QTRSensors::setDimmingLevel(uint8_t dimmingLevel)
{
  if (dimmingLevel > 31) { dimmingLevel = 31; }
  _dimmingLevel = dimmingLevel;
}

// emitters defaults to QTREmitters::All; wait defaults to true
void QTRSensors::emittersOff(QTREmitters emitters, bool wait)
{
  bool pinChanged = false;

  // Use odd emitter pin in these cases:
  // - 1 emitter pin, emitters = all
  // - 2 emitter pins, emitters = all
  // - 2 emitter pins, emitters = odd
  if (emitters == QTREmitters::All ||
      (_emitterPinCount == 2 && emitters == QTREmitters::Odd))
  {
    // Check if pin is defined and only turn off if not already off
    if ((_oddEmitterPin != QTRNoEmitterPin) &&
        (digitalRead(_oddEmitterPin) == HIGH))
    {
      digitalWrite(_oddEmitterPin, LOW);
      pinChanged = true;
    }
  }

  // Use even emitter pin in these cases:
  // - 2 emitter pins, emitters = all
  // - 2 emitter pins, emitters = even
  if (_emitterPinCount == 2 &&
      (emitters == QTREmitters::All || emitters == QTREmitters::Even))
  {
    // Check if pin is defined and only turn off if not already off
    if ((_evenEmitterPin != QTRNoEmitterPin) &&
        (digitalRead(_evenEmitterPin) == HIGH))
    {
      digitalWrite(_evenEmitterPin, LOW);
      pinChanged = true;
    }
  }

  if (wait && pinChanged)
  {
    if (_dimmable)
    {
      // driver min is 1 ms
      delayMicroseconds(1200);
    }
    else
    {
      delayMicroseconds(200);
    }
  }
}

void QTRSensors::emittersOn(QTREmitters emitters, bool wait)
{
  bool pinChanged = false;
  uint16_t emittersOnStart;

  // Use odd emitter pin in these cases:
  // - 1 emitter pin, emitters = all
  // - 2 emitter pins, emitters = all
  // - 2 emitter pins, emitters = odd
  if (emitters == QTREmitters::All ||
      (_emitterPinCount == 2 && emitters == QTREmitters::Odd))
  {
    // Check if pin is defined, and only turn on non-dimmable sensors if not
    // already on, but always turn dimmable sensors off and back on because
    // we might be changing the dimming level (emittersOnWithPin() should take
    // care of this)
    if ((_oddEmitterPin != QTRNoEmitterPin) &&
        ( _dimmable || (digitalRead(_oddEmitterPin) == LOW)))
    {
      emittersOnStart = emittersOnWithPin(_oddEmitterPin);
      pinChanged = true;
    }
  }

  // Use even emitter pin in these cases:
  // - 2 emitter pins, emitters = all
  // - 2 emitter pins, emitters = even
  if (_emitterPinCount == 2 &&
      (emitters == QTREmitters::All || emitters == QTREmitters::Even))
  {
    // Check if pin is defined, and only turn on non-dimmable sensors if not
    // already on, but always turn dimmable sensors off and back on because
    // we might be changing the dimming level (emittersOnWithPin() should take
    // care of this)
    if ((_evenEmitterPin != QTRNoEmitterPin) &&
        (_dimmable || (digitalRead(_evenEmitterPin) == LOW)))
    {
      emittersOnStart = emittersOnWithPin(_evenEmitterPin);
      pinChanged = true;
    }
  }

  if (wait && pinChanged)
  {
    if (_dimmable)
    {
      // Make sure it's been at least 300 us since the emitter pin was first set
      // high before returning. (Driver min is 250 us.) Some time might have
      // already passed while we set the dimming level.
      while ((uint16_t)(micros() - emittersOnStart) < 300)
      {
        delayMicroseconds(10);
      }
    }
    else
    {
      delayMicroseconds(200);
    }
  }
}

// assumes pin is valid (not QTRNoEmitterPin)
// returns time when pin was first set high (used by emittersSelect())
uint16_t QTRSensors::emittersOnWithPin(uint8_t pin)
{
  if (_dimmable && (digitalRead(pin) == HIGH))
  {
    // We are turning on dimmable emitters that are already on. To avoid messing
    // up the dimming level, we have to turn the emitters off and back on. This
    // means the turn-off delay will happen even if wait = false was passed to
    // emittersOn(). (Driver min is 1 ms.)
    digitalWrite(pin, LOW);
    delayMicroseconds(1200);
  }

  digitalWrite(pin, HIGH);
  uint16_t emittersOnStart = micros();

  if (_dimmable && (_dimmingLevel > 0))
  {
    noInterrupts();

    for (uint8_t i = 0; i < _dimmingLevel; i++)
    {
      delayMicroseconds(1);
      digitalWrite(pin, LOW);
      delayMicroseconds(1);
      digitalWrite(pin, HIGH);
    }

    interrupts();
  }

  return emittersOnStart;
}

void QTRSensors::emittersSelect(QTREmitters emitters)
{
  QTREmitters offEmitters;

  switch (emitters)
  {
    case QTREmitters::Odd:
      offEmitters = QTREmitters::Even;
      break;

    case QTREmitters::Even:
      offEmitters = QTREmitters::Odd;
      break;

    case QTREmitters::All:
      emittersOn();
      return;

    case QTREmitters::None:
      emittersOff();
      return;

    default: // invalid
      return;
  }

  // Turn off the off-emitters; don't wait before proceeding, but record the time.
  emittersOff(offEmitters, false);
  uint16_t turnOffStart = micros();

  // Turn on the on-emitters and wait.
  emittersOn(emitters);

  if (_dimmable)
  {
    // Finish waiting for the off-emitters emitters to turn off: make sure it's been
    // at least 1200 us since the off-emitters was turned off before returning.
    // (Driver min is 1 ms.) Some time has already passed while we waited for
    // the on-emitters to turn on.
    while ((uint16_t)(micros() - turnOffStart) < 1200)
    {
      delayMicroseconds(10);
    }
  }
}

void QTRSensors::read(uint16_t * sensorValues, QTRReadMode mode)
{
  switch (mode)
  {
    case QTRReadMode::Off:
      emittersOff();
      // fall through
    case QTRReadMode::Manual:
      readPrivate(sensorValues);
      return;

    case QTRReadMode::On:
    case QTRReadMode::OnAndOff:
      emittersOn();
      readPrivate(sensorValues);
      emittersOff();
      break;

    case QTRReadMode::OddEven:
    case QTRReadMode::OddEvenAndOff:
      // Turn on odd emitters and read the odd-numbered sensors.
      // (readPrivate takes a 0-based array index, so start = 0 to start with
      // the first sensor)
      emittersSelect(QTREmitters::Odd);
      readPrivate(sensorValues, 0, 2);

      // Turn on even emitters and read the even-numbered sensors.
      // (readPrivate takes a 0-based array index, so start = 1 to start with
      // the second sensor)
      emittersSelect(QTREmitters::Even);
      readPrivate(sensorValues, 1, 2);

      emittersOff();
      break;

    default: // invalid - do nothing
      return;
  }

  if (mode == QTRReadMode::OnAndOff ||
      mode == QTRReadMode::OddEvenAndOff)
  {
    // Take a second set of readings and return the values (on + max - off).

    uint16_t offValues[QTRMaxSensors];
    readPrivate(offValues);

    for (uint8_t i = 0; i < _sensorCount; i++)
    {
      sensorValues[i] += _maxValue - offValues[i];
      if (sensorValues[i] > _maxValue)
      {
        // This usually doesn't happen, because the sensor reading should
        // go up when the emitters are turned off.
        sensorValues[i] = _maxValue;
      }
    }
  }
}


// Reads the first of every [step] sensors, starting with [start] (0-indexed, so
// start = 0 means start with the first sensor).
// For example, step = 2, start = 1 means read the *even-numbered* sensors.
// start defaults to 0, step defaults to 1
void QTRSensors::readPrivate(uint16_t * sensorValues, uint8_t start, uint8_t step)
{
  if (_sensorPins == nullptr) { return; }

      for (uint8_t i = start; i < _sensorCount; i += step)
      {
        sensorValues[i] = _maxValue;
        // make sensor line an output (drives low briefly, but doesn't matter)
        pinMode(_sensorPins[i], OUTPUT);
        // drive sensor line high
        digitalWrite(_sensorPins[i], HIGH);
      }

      delayMicroseconds(10); // charge lines for 10 us

      {
        // disable interrupts so we can switch all the pins as close to the same
        // time as possible
        noInterrupts();

        // record start time before the first sensor is switched to input
        // (similarly, time is checked before the first sensor is read in the
        // loop below)
        uint32_t startTime = micros();
        uint16_t time = 0;

        for (uint8_t i = start; i < _sensorCount; i += step)
        {
          // make sensor line an input (should also ensure pull-up is disabled)
          pinMode(_sensorPins[i], INPUT);
        }

        interrupts(); // re-enable

        while (time < _maxValue)
        {
          // disable interrupts so we can read all the pins as close to the same
          // time as possible
          noInterrupts();

          time = micros() - startTime;
          for (uint8_t i = start; i < _sensorCount; i += step)
          {
            if ((digitalRead(_sensorPins[i]) == LOW) && (time < sensorValues[i]))
            {
              // record the first time the line reads low
              sensorValues[i] = time;
            }
          }

          interrupts(); // re-enable
        }
      }
 
}


// the destructor frees up allocated memory
QTRSensors::~QTRSensors()
{
  releaseEmitterPins();

  if (_sensorPins)            { free(_sensorPins); }

}
