/*
  StandardFirmataCustom_STM32.ino
  Customized StandardFirmata sketch with support for dynamic pin assignment for:
  - UART (HardwareSerial)
  - SPI
  - I2C
  - CAN-bus (with conditional compilation for supported STM32 boards)
*/

#include <Servo.h>
#include <Wire.h>
#include <SPI.h>
#include <Firmata.h>

// CAN Bus Library (if supported and available)
#if defined(HAL_CAN_MODULE_ENABLED) && __has_include(<STM32_CAN.h>)
#include <STM32_CAN.h>
#define CAN_SUPPORTED
#endif

// Custom SysEx Commands Definition
#define CUSTOM_I2C_CONFIG   0x10
#define CUSTOM_SPI_CONFIG   0x11
#define CUSTOM_UART_CONFIG  0x12
#define CUSTOM_SPI_TRANSFER 0x13
#define CUSTOM_CAN_CONFIG   0x14
#define CUSTOM_CAN_WRITE    0x15
#define CUSTOM_CAN_READ     0x16
#define CUSTOM_UART_WRITE   0x17

// Global dynamic peripheral pointers
HardwareSerial *customSerial = NULL;
byte current_cs_pin = 255;

#ifdef CAN_SUPPORTED
STM32_CAN *canBus = NULL;
#endif

#define I2C_WRITE                   B00000000
#define I2C_READ                    B00001000
#define I2C_READ_CONTINUOUSLY       B00010000
#define I2C_STOP_READING            B00011000
#define I2C_READ_WRITE_MODE_MASK    B00011000
#define I2C_10BIT_ADDRESS_MODE_MASK B00100000
#define I2C_END_TX_MASK             B01000000
#define I2C_STOP_TX                 1
#define I2C_RESTART_TX              0
#define I2C_MAX_QUERIES             8
#define I2C_REGISTER_NOT_SPECIFIED  -1

// the minimum interval for sampling analog input
#define MINIMUM_SAMPLING_INTERVAL   1

/*==============================================================================
 * GLOBAL VARIABLES
 *============================================================================*/

#ifdef FIRMATA_SERIAL_FEATURE
SerialFirmata serialFeature;
#endif

/* analog inputs */
int analogInputsToReport = 0; // bitwise array to store pin reporting

/* digital input ports */
byte reportPINs[TOTAL_PORTS];       // 1 = report this port, 0 = silence
byte previousPINs[TOTAL_PORTS];     // previous 8 bits sent

/* pins configuration */
byte portConfigInputs[TOTAL_PORTS]; // each bit: 1 = pin in INPUT, 0 = anything else

/* timer variables */
unsigned long currentMillis;        // store the current value from millis()
unsigned long previousMillis;       // for comparison with currentMillis
unsigned int samplingInterval = 19; // how often to run the main loop (in ms)

/* i2c data */
struct i2c_device_info {
  byte addr;
  int reg;
  byte bytes;
  byte stopTX;
};

/* for i2c read continuous more */
i2c_device_info query[I2C_MAX_QUERIES];

byte i2cRxData[64];
boolean isI2CEnabled = false;
signed char queryIndex = -1;
unsigned int i2cReadDelayTime = 0;

Servo servos[MAX_SERVOS];
byte servoPinMap[TOTAL_PINS];
byte detachedServos[MAX_SERVOS];
byte detachedServoCount = 0;
byte servoCount = 0;

boolean isResetting = false;

// Forward declarations
void setPinModeCallback(byte, int);
void reportAnalogCallback(byte analogPin, int value);
void sysexCallback(byte, byte, byte*);

/* utility functions */
void wireWrite(byte data)
{
#if ARDUINO >= 100
  Wire.write((byte)data);
#else
  Wire.send(data);
#endif
}

byte wireRead(void)
{
#if ARDUINO >= 100
  return Wire.read();
#else
  return Wire.receive();
#endif
}

/*==============================================================================
 * FUNCTIONS
 *============================================================================*/

void attachServo(byte pin, int minPulse, int maxPulse)
{
  if (servoCount < MAX_SERVOS) {
    if (detachedServoCount > 0) {
      servoPinMap[pin] = detachedServos[detachedServoCount - 1];
      if (detachedServoCount > 0) detachedServoCount--;
    } else {
      servoPinMap[pin] = servoCount;
      servoCount++;
    }
    if (minPulse > 0 && maxPulse > 0) {
      servos[servoPinMap[pin]].attach(PIN_TO_DIGITAL(pin), minPulse, maxPulse);
    } else {
      servos[servoPinMap[pin]].attach(PIN_TO_DIGITAL(pin));
    }
  } else {
    Firmata.sendString("Max servos attached");
  }
}

void detachServo(byte pin)
{
  servos[servoPinMap[pin]].detach();
  if (servoPinMap[pin] == servoCount && servoCount > 0) {
    servoCount--;
  } else if (servoCount > 0) {
    detachedServoCount++;
    detachedServos[detachedServoCount - 1] = servoPinMap[pin];
  }
  servoPinMap[pin] = 255;
}

void enableI2CPins()
{
  byte i;
  for (i = 0; i < TOTAL_PINS; i++) {
    if (IS_PIN_I2C(i)) {
      setPinModeCallback(i, PIN_MODE_I2C);
    }
  }
  isI2CEnabled = true;
  Wire.begin();
}

void disableI2CPins() {
  isI2CEnabled = false;
  queryIndex = -1;
}

void readAndReportData(byte address, int theRegister, byte numBytes, byte stopTX) {
  if (theRegister != I2C_REGISTER_NOT_SPECIFIED) {
    Wire.beginTransmission(address);
    wireWrite((byte)theRegister);
    Wire.endTransmission(stopTX);
    if (i2cReadDelayTime > 0) {
      delayMicroseconds(i2cReadDelayTime);
    }
  } else {
    theRegister = 0;
  }

  Wire.requestFrom(address, numBytes);

  if (numBytes < Wire.available()) {
    Firmata.sendString("I2C: Too many bytes received");
  } else if (numBytes > Wire.available()) {
    Firmata.sendString("I2C: Too few bytes received");
    numBytes = Wire.available();
  }

  i2cRxData[0] = address;
  i2cRxData[1] = theRegister;

  for (int i = 0; i < numBytes && Wire.available(); i++) {
    i2cRxData[2 + i] = wireRead();
  }

  Firmata.sendSysex(SYSEX_I2C_REPLY, numBytes + 2, i2cRxData);
}

void outputPort(byte portNumber, byte portValue, byte forceSend)
{
  portValue = portValue & portConfigInputs[portNumber];
  if (forceSend || previousPINs[portNumber] != portValue) {
    Firmata.sendDigitalPort(portNumber, portValue);
    previousPINs[portNumber] = portValue;
  }
}

void checkDigitalInputs(void)
{
  if (TOTAL_PORTS > 0 && reportPINs[0]) outputPort(0, readPort(0, portConfigInputs[0]), false);
  if (TOTAL_PORTS > 1 && reportPINs[1]) outputPort(1, readPort(1, portConfigInputs[1]), false);
  if (TOTAL_PORTS > 2 && reportPINs[2]) outputPort(2, readPort(2, portConfigInputs[2]), false);
  if (TOTAL_PORTS > 3 && reportPINs[3]) outputPort(3, readPort(3, portConfigInputs[3]), false);
  if (TOTAL_PORTS > 4 && reportPINs[4]) outputPort(4, readPort(4, portConfigInputs[4]), false);
  if (TOTAL_PORTS > 5 && reportPINs[5]) outputPort(5, readPort(5, portConfigInputs[5]), false);
  if (TOTAL_PORTS > 6 && reportPINs[6]) outputPort(6, readPort(6, portConfigInputs[6]), false);
  if (TOTAL_PORTS > 7 && reportPINs[7]) outputPort(7, readPort(7, portConfigInputs[7]), false);
  if (TOTAL_PORTS > 8 && reportPINs[8]) outputPort(8, readPort(8, portConfigInputs[8]), false);
  if (TOTAL_PORTS > 9 && reportPINs[9]) outputPort(9, readPort(9, portConfigInputs[9]), false);
  if (TOTAL_PORTS > 10 && reportPINs[10]) outputPort(10, readPort(10, portConfigInputs[10]), false);
  if (TOTAL_PORTS > 11 && reportPINs[11]) outputPort(11, readPort(11, portConfigInputs[11]), false);
  if (TOTAL_PORTS > 12 && reportPINs[12]) outputPort(12, readPort(12, portConfigInputs[12]), false);
  if (TOTAL_PORTS > 13 && reportPINs[13]) outputPort(13, readPort(13, portConfigInputs[13]), false);
  if (TOTAL_PORTS > 14 && reportPINs[14]) outputPort(14, readPort(14, portConfigInputs[14]), false);
  if (TOTAL_PORTS > 15 && reportPINs[15]) outputPort(15, readPort(15, portConfigInputs[15]), false);
}

void setPinModeCallback(byte pin, int mode)
{
  if (Firmata.getPinMode(pin) == PIN_MODE_IGNORE)
    return;

  if (Firmata.getPinMode(pin) == PIN_MODE_I2C && isI2CEnabled && mode != PIN_MODE_I2C) {
    disableI2CPins();
  }
  if (IS_PIN_DIGITAL(pin) && mode != PIN_MODE_SERVO) {
    if (servoPinMap[pin] < MAX_SERVOS && servos[servoPinMap[pin]].attached()) {
      detachServo(pin);
    }
  }
  if (IS_PIN_ANALOG(pin)) {
    reportAnalogCallback(PIN_TO_ANALOG(pin), mode == PIN_MODE_ANALOG ? 1 : 0);
  }
  if (IS_PIN_DIGITAL(pin)) {
    if (mode == INPUT || mode == PIN_MODE_PULLUP) {
      portConfigInputs[pin / 8] |= (1 << (pin & 7));
    } else {
      portConfigInputs[pin / 8] &= ~(1 << (pin & 7));
    }
  }
  Firmata.setPinState(pin, 0);
  switch (mode) {
    case PIN_MODE_ANALOG:
      if (IS_PIN_ANALOG(pin)) {
        if (IS_PIN_DIGITAL(pin)) {
          pinMode(PIN_TO_DIGITAL(pin), INPUT);
#if ARDUINO <= 100
          digitalWrite(PIN_TO_DIGITAL(pin), LOW);
#endif
        }
        Firmata.setPinMode(pin, PIN_MODE_ANALOG);
      }
      break;
    case INPUT:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT);
#if ARDUINO <= 100
        digitalWrite(PIN_TO_DIGITAL(pin), LOW);
#endif
        Firmata.setPinMode(pin, INPUT);
      }
      break;
    case PIN_MODE_PULLUP:
      if (IS_PIN_DIGITAL(pin)) {
        pinMode(PIN_TO_DIGITAL(pin), INPUT_PULLUP);
        Firmata.setPinMode(pin, PIN_MODE_PULLUP);
        Firmata.setPinState(pin, 1);
      }
      break;
    case OUTPUT:
      if (IS_PIN_DIGITAL(pin)) {
        if (Firmata.getPinMode(pin) == PIN_MODE_PWM) {
          digitalWrite(PIN_TO_DIGITAL(pin), LOW);
        }
        pinMode(PIN_TO_DIGITAL(pin), OUTPUT);
        Firmata.setPinMode(pin, OUTPUT);
      }
      break;
    case PIN_MODE_PWM:
      if (IS_PIN_PWM(pin)) {
        pinMode(PIN_TO_PWM(pin), OUTPUT);
        analogWrite(PIN_TO_PWM(pin), 0);
        Firmata.setPinMode(pin, PIN_MODE_PWM);
      }
      break;
    case PIN_MODE_SERVO:
      if (IS_PIN_DIGITAL(pin)) {
        Firmata.setPinMode(pin, PIN_MODE_SERVO);
        if (servoPinMap[pin] == 255 || !servos[servoPinMap[pin]].attached()) {
          attachServo(pin, -1, -1);
        }
      }
      break;
    case PIN_MODE_I2C:
      if (IS_PIN_I2C(pin)) {
        Firmata.setPinMode(pin, PIN_MODE_I2C);
      }
      break;
    case PIN_MODE_SERIAL:
#ifdef FIRMATA_SERIAL_FEATURE
      serialFeature.handlePinMode(pin, PIN_MODE_SERIAL);
#endif
      break;
    default:
      Firmata.sendString("Unknown pin mode");
  }
}

void setPinValueCallback(byte pin, int value)
{
  if (pin < TOTAL_PINS && IS_PIN_DIGITAL(pin)) {
    if (Firmata.getPinMode(pin) == OUTPUT) {
      Firmata.setPinState(pin, value);
      digitalWrite(PIN_TO_DIGITAL(pin), value);
    }
  }
}

void analogWriteCallback(byte pin, int value)
{
  if (pin < TOTAL_PINS) {
    switch (Firmata.getPinMode(pin)) {
      case PIN_MODE_SERVO:
        if (IS_PIN_DIGITAL(pin))
          servos[servoPinMap[pin]].write(value);
        Firmata.setPinState(pin, value);
        break;
      case PIN_MODE_PWM:
        if (IS_PIN_PWM(pin))
          analogWrite(PIN_TO_PWM(pin), value);
        Firmata.setPinState(pin, value);
        break;
    }
  }
}

void digitalWriteCallback(byte port, int value)
{
  byte pin, lastPin, pinValue, mask = 1, pinWriteMask = 0;

  if (port < TOTAL_PORTS) {
    lastPin = port * 8 + 8;
    if (lastPin > TOTAL_PINS) lastPin = TOTAL_PINS;
    for (pin = port * 8; pin < lastPin; pin++) {
      if (IS_PIN_DIGITAL(pin)) {
        if (Firmata.getPinMode(pin) == OUTPUT || Firmata.getPinMode(pin) == INPUT) {
          pinValue = ((byte)value & mask) ? 1 : 0;
          if (Firmata.getPinMode(pin) == OUTPUT) {
            pinWriteMask |= mask;
          } else if (Firmata.getPinMode(pin) == INPUT && pinValue == 1 && Firmata.getPinState(pin) != 1) {
#if ARDUINO > 100
            pinMode(pin, INPUT_PULLUP);
#else
            pinWriteMask |= mask;
#endif
          }
          Firmata.setPinState(pin, pinValue);
        }
      }
      mask = mask << 1;
    }
    writePort(port, (byte)value, pinWriteMask);
  }
}

void reportAnalogCallback(byte analogPin, int value)
{
  if (analogPin < TOTAL_ANALOG_PINS) {
    if (value == 0) {
      analogInputsToReport = analogInputsToReport & ~ (1 << analogPin);
    } else {
      analogInputsToReport = analogInputsToReport | (1 << analogPin);
      if (!isResetting) {
        Firmata.sendAnalog(analogPin, analogRead(analogPin));
      }
    }
  }
}

void reportDigitalCallback(byte port, int value)
{
  if (port < TOTAL_PORTS) {
    reportPINs[port] = (byte)value;
    if (value) outputPort(port, readPort(port, portConfigInputs[port]), true);
  }
}

/*==============================================================================
 * SYSEX-BASED commands
 *============================================================================*/

void sysexCallback(byte command, byte argc, byte *argv)
{
  byte mode;
  byte stopTX;
  byte slaveAddress;
  byte data;
  int slaveRegister;
  unsigned int delayTime;

  switch (command) {
    // --- Custom Dynamic Peripherals ---
    case CUSTOM_I2C_CONFIG: {
      byte sda = argv[0];
      byte scl = argv[1];
      Wire.end();
      Wire.setSDA(PIN_TO_DIGITAL(sda));
      Wire.setSCL(PIN_TO_DIGITAL(scl));
      Wire.begin();
      isI2CEnabled = true;
      Firmata.sendString("I2C dynamic configured");
      break;
    }
    
    case CUSTOM_SPI_CONFIG: {
      byte mosi = argv[0];
      byte miso = argv[1];
      byte sclk = argv[2];
      current_cs_pin = argv[3];
      SPI.end();
      SPI.setMOSI(PIN_TO_DIGITAL(mosi));
      SPI.setMISO(PIN_TO_DIGITAL(miso));
      SPI.setSCLK(PIN_TO_DIGITAL(sclk));
      SPI.begin();
      pinMode(PIN_TO_DIGITAL(current_cs_pin), OUTPUT);
      digitalWrite(PIN_TO_DIGITAL(current_cs_pin), HIGH);
      Firmata.sendString("SPI dynamic configured");
      break;
    }
    
    case CUSTOM_SPI_TRANSFER: {
      if (current_cs_pin == 255) {
        Firmata.sendString("SPI not configured");
        break;
      }
      byte cs_pin = argv[0];
      byte data_len = (argc - 1) / 2;
      byte tx_buf[data_len];
      byte rx_buf[data_len * 2];
      
      for (byte i = 0; i < data_len; i++) {
        tx_buf[i] = argv[1 + i*2] | (argv[2 + i*2] << 7);
      }
      
      digitalWrite(PIN_TO_DIGITAL(cs_pin), LOW);
      for (byte i = 0; i < data_len; i++) {
        byte received = SPI.transfer(tx_buf[i]);
        rx_buf[i*2] = received & 0x7F;
        rx_buf[i*2 + 1] = (received >> 7) & 0x7F;
      }
      digitalWrite(PIN_TO_DIGITAL(cs_pin), HIGH);
      
      Firmata.sendSysex(CUSTOM_SPI_TRANSFER, data_len * 2, rx_buf);
      break;
    }
    
    case CUSTOM_UART_CONFIG: {
      byte rx = argv[0];
      byte tx = argv[1];
      unsigned long baudrate = argv[2] | (argv[3] << 7) | ((unsigned long)argv[4] << 14);
      if (customSerial != NULL) {
        customSerial->end();
        delete customSerial;
      }
      customSerial = new HardwareSerial(PIN_TO_DIGITAL(rx), PIN_TO_DIGITAL(tx));
      customSerial->begin(baudrate);
      Firmata.sendString("UART dynamic configured");
      break;
    }

    case CUSTOM_UART_WRITE: {
      if (customSerial == NULL) {
        Firmata.sendString("UART not configured");
        break;
      }
      byte data_len = argc / 2;
      for (byte i = 0; i < data_len; i++) {
        byte b = argv[i*2] | (argv[i*2 + 1] << 7);
        customSerial->write(b);
      }
      break;
    }

    case CUSTOM_CAN_CONFIG: {
#ifdef CAN_SUPPORTED
      byte rx = argv[0];
      byte tx = argv[1];
      unsigned long baudrate = argv[2] | (argv[3] << 7) | ((unsigned long)argv[4] << 14) | ((unsigned long)argv[5] << 21);
      if (canBus != NULL) {
        canBus->end();
        delete canBus;
      }
      PinName rx_name = digitalPinToPinName(PIN_TO_DIGITAL(rx));
      PinName tx_name = digitalPinToPinName(PIN_TO_DIGITAL(tx));
      canBus = new STM32_CAN(rx_name, tx_name);
      canBus->begin();
      canBus->setBaudRate(baudrate);
      Firmata.sendString("CAN configured");
#else
      Firmata.sendString("CAN hardware not supported on this board");
#endif
      break;
    }

    case CUSTOM_CAN_WRITE: {
#ifdef CAN_SUPPORTED
      if (canBus == NULL) {
        Firmata.sendString("CAN not configured");
        break;
      }
      unsigned long id = 0;
      for (byte i = 0; i < 5; i++) {
        id |= ((unsigned long)argv[i] << (i * 7));
      }
      byte isExtended = argv[5];
      byte len = argv[6];
      
      CAN_message_t txMsg;
      txMsg.id = id;
      txMsg.len = len;
      txMsg.flags.extended = isExtended;
      
      for (byte i = 0; i < len; i++) {
        txMsg.buf[i] = argv[7 + i*2] | (argv[8 + i*2] << 7);
      }
      canBus->write(txMsg);
#else
      Firmata.sendString("CAN hardware not supported");
#endif
      break;
    }

    // --- Standard Commands ---
    case I2C_REQUEST:
      mode = argv[1] & I2C_READ_WRITE_MODE_MASK;
      if (argv[1] & I2C_10BIT_ADDRESS_MODE_MASK) {
        Firmata.sendString("10-bit addressing not supported");
        return;
      }
      else {
        slaveAddress = argv[0];
      }

      if (argv[1] & I2C_END_TX_MASK) {
        stopTX = I2C_RESTART_TX;
      }
      else {
        stopTX = I2C_STOP_TX;
      }

      switch (mode) {
        case I2C_WRITE:
          Wire.beginTransmission(slaveAddress);
          for (byte i = 2; i < argc; i += 2) {
            data = argv[i] + (argv[i + 1] << 7);
            wireWrite(data);
          }
          Wire.endTransmission();
          delayMicroseconds(70);
          break;
        case I2C_READ:
          if (argc == 6) {
            slaveRegister = argv[2] + (argv[3] << 7);
            data = argv[4] + (argv[5] << 7);
          }
          else {
            slaveRegister = I2C_REGISTER_NOT_SPECIFIED;
            data = argv[2] + (argv[3] << 7);
          }
          readAndReportData(slaveAddress, (int)slaveRegister, data, stopTX);
          break;
        case I2C_READ_CONTINUOUSLY:
          if ((queryIndex + 1) >= I2C_MAX_QUERIES) {
            Firmata.sendString("too many queries");
            break;
          }
          if (argc == 6) {
            slaveRegister = argv[2] + (argv[3] << 7);
            data = argv[4] + (argv[5] << 7);
          }
          else {
            slaveRegister = (int)I2C_REGISTER_NOT_SPECIFIED;
            data = argv[2] + (argv[3] << 7);
          }
          queryIndex++;
          query[queryIndex].addr = slaveAddress;
          query[queryIndex].reg = slaveRegister;
          query[queryIndex].bytes = data;
          query[queryIndex].stopTX = stopTX;
          break;
        case I2C_STOP_READING:
          byte queryIndexToSkip;
          if (queryIndex <= 0) {
            queryIndex = -1;
          } else {
            queryIndexToSkip = 0;
            for (byte i = 0; i < queryIndex + 1; i++) {
              if (query[i].addr == slaveAddress) {
                queryIndexToSkip = i;
                break;
              }
            }
            for (byte i = queryIndexToSkip; i < queryIndex + 1; i++) {
              if (i < I2C_MAX_QUERIES) {
                query[i].addr = query[i + 1].addr;
                query[i].reg = query[i + 1].reg;
                query[i].bytes = query[i + 1].bytes;
                query[i].stopTX = query[i + 1].stopTX;
              }
            }
            queryIndex--;
          }
          break;
        default:
          break;
      }
      break;
    case I2C_CONFIG:
      delayTime = (argv[0] + (argv[1] << 7));
      if (argc > 1 && delayTime > 0) {
        i2cReadDelayTime = delayTime;
      }
      if (!isI2CEnabled) {
        enableI2CPins();
      }
      break;
    case SERVO_CONFIG:
      if (argc > 4) {
        byte pin = argv[0];
        int minPulse = argv[1] + (argv[2] << 7);
        int maxPulse = argv[3] + (argv[4] << 7);

        if (IS_PIN_DIGITAL(pin)) {
          if (servoPinMap[pin] < MAX_SERVOS && servos[servoPinMap[pin]].attached()) {
            detachServo(pin);
          }
          attachServo(pin, minPulse, maxPulse);
          setPinModeCallback(pin, PIN_MODE_SERVO);
        }
      }
      break;
    case SAMPLING_INTERVAL:
      if (argc > 1) {
        samplingInterval = argv[0] + (argv[1] << 7);
        if (samplingInterval < MINIMUM_SAMPLING_INTERVAL) {
          samplingInterval = MINIMUM_SAMPLING_INTERVAL;
        }
      }
      break;
    case EXTENDED_ANALOG:
      if (argc > 1) {
        int val = argv[1];
        if (argc > 2) val |= (argv[2] << 7);
        if (argc > 3) val |= (argv[3] << 14);
        analogWriteCallback(argv[0], val);
      }
      break;
    case CAPABILITY_QUERY:
      Firmata.write(START_SYSEX);
      Firmata.write(CAPABILITY_RESPONSE);
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        if (IS_PIN_DIGITAL(pin)) {
          Firmata.write((byte)INPUT);
          Firmata.write(1);
          Firmata.write((byte)PIN_MODE_PULLUP);
          Firmata.write(1);
          Firmata.write((byte)OUTPUT);
          Firmata.write(1);
        }
        if (IS_PIN_ANALOG(pin)) {
          Firmata.write(PIN_MODE_ANALOG);
          Firmata.write(10);
        }
        if (IS_PIN_PWM(pin)) {
          Firmata.write(PIN_MODE_PWM);
          Firmata.write(DEFAULT_PWM_RESOLUTION);
        }
        if (IS_PIN_DIGITAL(pin)) {
          Firmata.write(PIN_MODE_SERVO);
          Firmata.write(14);
        }
        if (IS_PIN_I2C(pin)) {
          Firmata.write(PIN_MODE_I2C);
          Firmata.write(1);
        }
#ifdef FIRMATA_SERIAL_FEATURE
        serialFeature.handleCapability(pin);
#endif
        Firmata.write(127);
      }
      Firmata.write(END_SYSEX);
      break;
    case PIN_STATE_QUERY:
      if (argc > 0) {
        byte pin = argv[0];
        Firmata.write(START_SYSEX);
        Firmata.write(PIN_STATE_RESPONSE);
        Firmata.write(pin);
        if (pin < TOTAL_PINS) {
          Firmata.write(Firmata.getPinMode(pin));
          Firmata.write((byte)Firmata.getPinState(pin) & 0x7F);
          if (Firmata.getPinState(pin) & 0xFF80) Firmata.write((byte)(Firmata.getPinState(pin) >> 7) & 0x7F);
          if (Firmata.getPinState(pin) & 0xC000) Firmata.write((byte)(Firmata.getPinState(pin) >> 14) & 0x7F);
        }
        Firmata.write(END_SYSEX);
      }
      break;
    case ANALOG_MAPPING_QUERY:
      Firmata.write(START_SYSEX);
      Firmata.write(ANALOG_MAPPING_RESPONSE);
      for (byte pin = 0; pin < TOTAL_PINS; pin++) {
        Firmata.write(IS_PIN_ANALOG(pin) ? PIN_TO_ANALOG(pin) : 127);
      }
      Firmata.write(END_SYSEX);
      break;
    case SERIAL_MESSAGE:
#ifdef FIRMATA_SERIAL_FEATURE
      serialFeature.handleSysex(command, argc, argv);
#endif
      break;
  }
}

/*==============================================================================
 * SETUP()
 *============================================================================*/

void systemResetCallback()
{
  isResetting = true;

#ifdef FIRMATA_SERIAL_FEATURE
  serialFeature.reset();
#endif

  if (isI2CEnabled) {
    disableI2CPins();
  }

  for (byte i = 0; i < TOTAL_PORTS; i++) {
    reportPINs[i] = false;
    portConfigInputs[i] = 0;
    previousPINs[i] = 0;
  }

  for (byte i = 0; i < TOTAL_PINS; i++) {
    if (IS_PIN_ANALOG(i)) {
      setPinModeCallback(i, PIN_MODE_ANALOG);
    } else if (IS_PIN_DIGITAL(i)) {
      setPinModeCallback(i, OUTPUT);
    }
    servoPinMap[i] = 255;
  }
  analogInputsToReport = 0;
  detachedServoCount = 0;
  servoCount = 0;
  isResetting = false;
}

void setup()
{
  Firmata.setFirmwareVersion(FIRMATA_FIRMWARE_MAJOR_VERSION, FIRMATA_FIRMWARE_MINOR_VERSION);

  Firmata.attach(ANALOG_MESSAGE, analogWriteCallback);
  Firmata.attach(DIGITAL_MESSAGE, digitalWriteCallback);
  Firmata.attach(REPORT_ANALOG, reportAnalogCallback);
  Firmata.attach(REPORT_DIGITAL, reportDigitalCallback);
  Firmata.attach(SET_PIN_MODE, setPinModeCallback);
  Firmata.attach(SET_DIGITAL_PIN_VALUE, setPinValueCallback);
  Firmata.attach(START_SYSEX, sysexCallback);
  Firmata.attach(SYSTEM_RESET, systemResetCallback);

  Firmata.begin(57600);
  while (!Serial) {
    ;
  }

  systemResetCallback();
}

/*==============================================================================
 * LOOP()
 *============================================================================*/
void loop()
{
  byte pin, analogPin;

  checkDigitalInputs();

  while (Firmata.available())
    Firmata.processInput();

  currentMillis = millis();
  if (currentMillis - previousMillis > samplingInterval) {
    previousMillis += samplingInterval;
    for (pin = 0; pin < TOTAL_PINS; pin++) {
      if (IS_PIN_ANALOG(pin) && Firmata.getPinMode(pin) == PIN_MODE_ANALOG) {
        analogPin = PIN_TO_ANALOG(pin);
        if (analogInputsToReport & (1 << analogPin)) {
          Firmata.sendAnalog(analogPin, analogRead(analogPin));
        }
      }
    }
    if (queryIndex > -1) {
      for (byte i = 0; i < queryIndex + 1; i++) {
        readAndReportData(query[i].addr, query[i].reg, query[i].bytes, query[i].stopTX);
      }
    }
  }

  // Custom UART background polling
  if (customSerial != NULL && customSerial->available() > 0) {
    byte rx_buf[64];
    byte count = 0;
    while (customSerial->available() && count < 32) {
      byte r = customSerial->read();
      rx_buf[count * 2] = r & 0x7F;
      rx_buf[count * 2 + 1] = (r >> 7) & 0x7F;
      count++;
    }
    Firmata.sendSysex(CUSTOM_UART_CONFIG, count * 2, rx_buf);
  }

  // Custom CAN-bus background polling
#ifdef CAN_SUPPORTED
  if (canBus != NULL) {
    CAN_message_t rxMsg;
    if (canBus->read(rxMsg)) {
      byte msg_buf[32];
      
      // Encode ID (4 bytes -> 5 7-bit bytes)
      for (byte i = 0; i < 5; i++) {
        msg_buf[i] = (rxMsg.id >> (i * 7)) & 0x7F;
      }
      msg_buf[5] = rxMsg.flags.extended;
      msg_buf[6] = rxMsg.len;
      
      // Encode Payload (up to 8 bytes -> 16 7-bit bytes)
      for (byte i = 0; i < rxMsg.len; i++) {
        msg_buf[7 + i*2] = rxMsg.buf[i] & 0x7F;
        msg_buf[8 + i*2] = (rxMsg.buf[i] >> 7) & 0x7F;
      }
      
      Firmata.sendSysex(CUSTOM_CAN_READ, 7 + rxMsg.len * 2, msg_buf);
    }
  }
#endif

#ifdef FIRMATA_SERIAL_FEATURE
  serialFeature.update();
#endif
}
