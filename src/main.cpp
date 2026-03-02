#include <Arduino.h>

#include <Wire.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <ArduinoRS485.h>
#include <ArduinoModbus.h>
#include <Adafruit_MAX31856.h>

// Pin definitions
const uint8_t PWM_OUT = 9;
const uint8_t DAC_CS = 10;
const uint8_t MAX_CS = 4;
const uint8_t MAX_FLT = 3;
const uint8_t MAX_DRDY = 2;

Adafruit_MAX31856 maxthermo = Adafruit_MAX31856(MAX_CS);

// Register definitions
enum Register
{
  reg_working_process_variable,
  reg_target_setpoint,
  reg_manual_power,
  reg_working_power,
  reg_working_setpoint,
  reg_rate,
  reg_control_mode,
  reg_pid_p,
  reg_pid_i,
  reg_pid_d,
  reg_software_enable,
  reg_tc_type,
  reg_tc_error,
  SIZE_REGISTERS
};

// EEPROM storage locations
enum EEPROMAdress
{
  ee_rate = 10,
  ee_pid_p = 12,
  ee_pid_i = 14,
  ee_pid_d = 16
};

typedef enum
{
  DAC_A = 0,
  DAC_B = 1
} dac_channel_t;


void set_output_power(int power);
void working_setpoint_adjust();
void setup_timer();
void write_to_eeprom();
void read_from_eeprom();
void pid_calculation();
void dac_write(dac_channel_t channel, uint16_t value);

// About units:
// All temperature like variables (process_variable, setpoint, error, sum_error and diff_error, pid_p) are stored in tenths of a degree C
// Integration and derivative time are stored in seconds
// The time interval is stored in milliseconds
// The output is stored in hundreths of percents

void setup()
{
  pinMode(DAC_CS, OUTPUT);
  digitalWrite(DAC_CS, HIGH);
  pinMode(MAX_CS, OUTPUT);
  digitalWrite(MAX_CS, HIGH);
  pinMode(MAX_FLT, INPUT);
  pinMode(MAX_DRDY, INPUT);
  pinMode(PWM_OUT, OUTPUT);
  digitalWrite(PWM_OUT, LOW);
  setup_timer();

  //SPI.begin();

  // Start the Modbus RTU server, with (slave) id 1
  Serial.begin(9600);
  if (!ModbusRTUServer.begin(1, 9600))
  {
    Serial.println("Failed to start Modbus RTU Server!");
    while (1);
  }

  // Initialize to type K thermocouple
  maxthermo.begin();
  maxthermo.setThermocoupleType(MAX31856_TCTYPE_K);
  maxthermo.setConversionMode(MAX31856_CONTINUOUS);
  maxthermo.setNoiseFilter(MAX31856_NOISE_FILTER_50HZ);

  // Everything will be handled by holding registers, all data is stored as 16 bit int (Like Eurotherms do)
  ModbusRTUServer.configureHoldingRegisters(0x00, SIZE_REGISTERS);
  read_from_eeprom();
}

void loop()
{
  ModbusRTUServer.poll();

  // Read thermocouple error state and store it in the register
  ModbusRTUServer.holdingRegisterWrite(reg_tc_error, maxthermo.readFault());
  if (ModbusRTUServer.holdingRegisterRead(reg_tc_error))
  {
    ModbusRTUServer.holdingRegisterWrite(reg_software_enable, 0);
  }
  else
  {
    ModbusRTUServer.holdingRegisterWrite(reg_working_process_variable, static_cast<int>(maxthermo.readThermocoupleTemperature() * 10 + 0.5));
  }

  // Control logic, either manual or automatic
  if (ModbusRTUServer.holdingRegisterRead(reg_control_mode))
  {
    ModbusRTUServer.holdingRegisterWrite(reg_working_power, ModbusRTUServer.holdingRegisterRead(reg_manual_power));
  }
  else
  {
    working_setpoint_adjust();
    pid_calculation();
  }

  // Only apply output power if software enable is set, i.e. if the thermocouple is ok
  if(ModbusRTUServer.holdingRegisterRead(reg_software_enable) == 0)
  {
    ModbusRTUServer.holdingRegisterWrite(reg_working_power, 0);
  }
  else
  {
    set_output_power(ModbusRTUServer.holdingRegisterRead(reg_working_power));
  }
  write_to_eeprom();
}

void set_output_power(int power)
{
  // Adjust the three output signals (current, voltage, pwm) to set the desired power level

  // PWM Output
  // Direct timer manipulation to access 16 bit resolution
  OCR1A = power;

  // Analog voltage output via DAC
  // DAC Fullrange is 2.048V corresponding to 4095
  // DAC Signal will be 0 to 2 V for 0 to 100% power
  // Amplified to 0 to 10V or 0 to 5V externally
  uint16_t dac_b_value = power * 2 / 5;
  dac_write(DAC_B, dac_b_value);

  // Analog current output via DAC
  // Signal will be 4 to 20 mA for 0 to 100% power
  // This corresponds to 400 to 2000 in DAC value
  // Calculation with 32 bit to prevent overflow
  uint32_t tmp = static_cast<uint32_t>(power) * 8u / 25u + 800u;
  uint16_t dac_a_value = static_cast<uint16_t>(tmp);
  dac_write(DAC_A, dac_a_value);
}

void setup_timer()
{
  // Setup timer 1 for PWM output
  // Phase and frequency correct pwm mode
  // No prescaler
  // Top value 10000
  // PWM frequency 800 Hz
  noInterrupts();
  TCCR1A = 0 | (1 << COM1A1);
  TCCR1B = 0 | (1 << WGM13) | (1 << CS10);
  ICR1 = 10000;
  OCR1A = 0;
  interrupts();
}

void write_to_eeprom()
{
  static uint16_t pid_p = static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_pid_p));
  static uint16_t pid_i = static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_pid_i));
  static uint16_t pid_d = static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_pid_d));
  static uint16_t rate = static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_rate));

  if (static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_pid_p)) != pid_p)
  {
    pid_p = static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_pid_p));
    EEPROM.put(ee_pid_p, pid_p);
  }

  if (static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_pid_i)) != pid_i)
  {
    pid_i = static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_pid_i));
    EEPROM.put(ee_pid_i, pid_i);
  }

  if (static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_pid_d)) != pid_d)
  {
    pid_d = static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_pid_d));
    EEPROM.put(ee_pid_d, pid_d);
  }

  if (static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_rate)) != rate)
  {
    rate = static_cast<uint16_t>(ModbusRTUServer.holdingRegisterRead(reg_rate));
    EEPROM.put(ee_rate, rate);
  }
}

void read_from_eeprom()
{
  uint16_t temp;
  ModbusRTUServer.holdingRegisterWrite(reg_pid_p, EEPROM.get(ee_pid_p, temp));
  ModbusRTUServer.holdingRegisterWrite(reg_pid_i, EEPROM.get(ee_pid_i, temp));
  ModbusRTUServer.holdingRegisterWrite(reg_pid_d, EEPROM.get(ee_pid_d, temp));
  ModbusRTUServer.holdingRegisterWrite(reg_rate, EEPROM.get(ee_rate, temp));
}

void pid_calculation()
{
  // About units:
  // All temperature like variables (process_variable, setpoint, error, sum_error and diff_error, pid_p) are stored in tenths of a degree C
  // Integration and derivative time are stored in seconds
  // The time interval is stored in milliseconds, hence the factors 1000
  // The output is stored in hundreths of percents
  // Shamelessly copied from the Arduino PID Library

  // Timing stuff
  const uint16_t interval = 100;
  static uint32_t last_time = millis();
  uint32_t now = millis();
  unsigned long timeChange = (now - last_time);

  // Memory
  static double last_input = ModbusRTUServer.holdingRegisterRead(reg_working_process_variable);
  static double output_sum = 0;

  // Control loop
  if (timeChange >= interval)
  {
    // Convert from parallel for to standard form
    // TODO Units
    double kp = 10000 / ModbusRTUServer.holdingRegisterRead(reg_pid_p);
    double ki = kp / ModbusRTUServer.holdingRegisterRead(reg_pid_i) * interval / 1000;
    double kd = kp * ModbusRTUServer.holdingRegisterRead(reg_pid_d) * 1000 / interval;

    /*Compute all the working error variables*/
    double input = ModbusRTUServer.holdingRegisterRead(reg_working_process_variable);
    double error = ModbusRTUServer.holdingRegisterRead(reg_working_setpoint) - input;
    double dInput = (input - last_input);

    // Integral term
    output_sum += (ki * error);
    output_sum = constrain(output_sum, 0, 10000);

    // Add integral, proportional and differential term term
    double output = output_sum + kp * error - kd * dInput;
    output = constrain(output, 0, 10000);

    last_input = input;
    last_time = now;

    ModbusRTUServer.holdingRegisterWrite(reg_working_power, output);
  }
}

void working_setpoint_adjust()
{
  // Adjust the working setpoint by an increment based on the given ramp or until the target SP is reached
  // Only perform the update every 100 milliseconds
  // The setpoint is written to the Modbus register as 10ths of degrees, but kept internally as [deciDegrees times millisecond per minute] to minimize rounding errors

  static uint32_t last_update = millis();
  uint32_t now = millis();
  if (now - last_update < 100)
    return;

  static int32_t last_setpoint = ModbusRTUServer.holdingRegisterRead(reg_working_setpoint);
  
  // Calculate the change of setpoint
  int32_t increment = ModbusRTUServer.holdingRegisterRead(reg_rate) * (now - last_update);

  // Check if the controller is ramping up or down and adjust setpoint by increment or until target SP is reached
  if (ModbusRTUServer.holdingRegisterRead(reg_working_setpoint) < ModbusRTUServer.holdingRegisterRead(reg_target_setpoint))
  {
    last_setpoint = min(ModbusRTUServer.holdingRegisterRead(reg_target_setpoint) * 1000 * 60, last_setpoint + increment);
  }
  else if(ModbusRTUServer.holdingRegisterRead(reg_working_setpoint) > ModbusRTUServer.holdingRegisterRead(reg_target_setpoint))
  {
    last_setpoint = max(ModbusRTUServer.holdingRegisterRead(reg_target_setpoint) * 1000 * 60, last_setpoint - increment);
  }
  ModbusRTUServer.holdingRegisterWrite(reg_working_setpoint, static_cast<int>(last_setpoint / 1000 / 60 + 0.5));
  last_update = now;
}

void dac_write(dac_channel_t channel, uint16_t value)
{
  uint16_t dac_word = channel << 15 | 1 << 13 | 1 << 12 | (value & 0x0FFF);
  SPI.beginTransaction(SPISettings(20000000, MSBFIRST, SPI_MODE0));
  digitalWrite(DAC_CS, LOW);        //2212 Select SPI device
  SPI.transfer16(dac_word);
  digitalWrite(DAC_CS, HIGH);       // Deselect device
}