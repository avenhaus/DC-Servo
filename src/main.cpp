/*======================================================================*\
 * ESP32-S3 DC Servo Motor Controller
\*======================================================================*/

/*
 * The MT6701 has a 14 bit resolution (0-16383).
 * 
 * https://www.lcsc.com/datasheet/lcsc_datasheet_2110291630_Magn-Tek-MT6701QT-STD_C2913974.pdf
 * 
 * Vcc is 3.3V or 5V, but EEPROM programming needs 5V. ESP32 I/O is not 5V tolerant.
 * 
 *  ** Small MT6701 board needs solder bridge on back to select I2c / SSI communication. **
 * 
 * DRV8833 Dual H-Bridge Motor Driver
 * https://www.ti.com/lit/ds/symlink/drv8833.pdf
 * 
 * H-Bridge Logic 
 * +-----+-----+------+------+--------------------+
 * | IN1 | IN2 | OUT1 | OUT2 | FUNCTION           |
 * +-----+-----+------+------+--------------------+
 * |  0  |  0  |  Z   |  Z   | Coast / fast decay |
 * |  0  |  1  |  L   |  H   | Reverse            |
 * |  1  |  0  |  H   |  L   | Forward            |
 * |  1  |  1  |  L   |  L   | Brake / slow decay |
 * +-----+-----+------+------+--------------------+ 
 */


#include <Arduino.h>
#include "Config.h"
#include <SPI.h>
#include <EEPROM.h>
#include "MT6701.h"
#include <pid_v1.h>

PROGMEM const char EMPTY_STRING[] =  "";
PROGMEM const char NEW_LINE[] =  "\n";

PROGMEM const char PROJECT_NAME[] = __PROJECT_NAME__;
PROGMEM const char PROJECT_VERSION[] = __PROJECT_COMPILE__;
PROGMEM const char COMPILE_DATE[] = __DATE__;
PROGMEM const char COMPILE_TIME[] = __TIME__;


Print* debugStream = &LOGGER;

MT6701 encoder;
uint16_t angle = 0; // Encoder angle depending on encoder resolution (MT6701 0-16383)
int32_t rotations;  // Number of full rotations
int32_t position;   // Rotations times encoder resolution plus current angle

double input = 0.0, output = 0.0, setpoint = 0.0;
PID positionPID(&input, &output, &setpoint, 3.0, 0.1, 0.1, DIRECT);

#ifdef CONFIG_IDF_TARGET_ESP32S3
#define HAS_RGB_LED 1
#endif

/***********************************************************\
 * Initialization Code
\***********************************************************/

void setup() {
#ifdef HAS_RGB_LED
  // There is currently a bug in the ESP32-S3 neopixelWrite() code. This will produce the 
  // following errors and the RGB LED will not light up.:
  //  E (19) rmt: rmt_set_gpio(526): RMT GPIO ERROR
  //  E (19) rmt: rmt_config(686): set gpio for RMT driver failed
  //  ==> Remove the #ifdef RGB_BUILTIN logic in the neopixelWrite() code.
  neopixelWrite(RGB_BUILTIN, 0, 10, 10);
#else
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
#endif

  pinMode(BUTTON_PIN, INPUT_PULLUP);

  LOGGER.begin(SERIAL_SPEED);
  delay(100);
  DEBUG_printf(FST("\n\n# %s %s | %s | %s\n"), PROJECT_NAME, PROJECT_VERSION, COMPILE_DATE, COMPILE_TIME);

  positionPID.SetMode(AUTOMATIC);
  positionPID.SetOutputLimits(-MOTOR_PWM_RANGE, MOTOR_PWM_RANGE);
  positionPID.SetSampleTime(LOOP_DELAY);

#if 0
  EEPROM.begin(EEPROM_SIZE);
  eeprom_data_t eeprom_data;
  EEPROM.get(0, eeprom_data);
  if (eeprom_data.magic == EEPROM_MAGIC) {
    memcpy(home_angle, eeprom_data.home_angle, sizeof(home_angle));
    DEBUG_print(FST("# Home angles loaded from EEPROM: "));
    for (int i = 0; i < ENCODER_COUNT; i++) {
      DEBUG_printf(FST("%0.2f"), home_angle[i]);
      if (i < ENCODER_COUNT - 1) { DEBUG_print(FST(", ")); }
    }
    DEBUG_println();
#ifdef HAS_RGB_LED
    neopixelWrite(RGB_BUILTIN, 0, 10, 0);
#endif
  } else {
    DEBUG_println(FST("# No valid EEPROM data found"));
#ifdef HAS_RGB_LED
    neopixelWrite(RGB_BUILTIN, 10, 10, 0);
#endif
  }
  EEPROM.end();
#endif

  analogWriteResolution(MOTOR_PWM_BITS);
  analogWriteFrequency(10000); // TODO: Increase frequency to 50kHz
  analogWrite(MOTOR_FORWARD_PIN, 0);
  analogWrite(MOTOR_REVERSE_PIN, 0);

  SPI.begin();
  bool ok = encoder.initializeSSI(MT6701_CS_PIN);
  angle = encoder.angleRead() * (16384.0f/360.0f); // TODO: optimize this
  DEBUG_printf(FST("# Init Encoder: %d - Angle: %d\n") , ok, angle);

  DEBUG_println(FST("# Init complete\n"));
}


/***********************************************************\
 * Main Loop
\***********************************************************/

void loop() {
  static bool last_button = HIGH;
  u32_t start = millis();

  // Read all encoders and check if the data has changed.
  bool change = false;
  // mt6701_status_t status = encoder.fieldStatusRead();
  uint16_t new_angle = encoder.angleRead() * (16384.0f/360.0f); // TODO: optimize this
    if (angle != new_angle) {
      if (new_angle < (ENCODER_RESOLUTION/4) &&  angle > (ENCODER_RESOLUTION/4*3)) {rotations++;}
      else if (new_angle > (ENCODER_RESOLUTION/4*3) && angle < (ENCODER_RESOLUTION/4)) {rotations--;}
      angle = new_angle;
      position = rotations * ENCODER_RESOLUTION + angle;
      change = true;
    }

  // Save the home position if the button is pressed.
  bool button = digitalRead(BUTTON_PIN);
  if (button == LOW && last_button == HIGH) {
  }
  last_button = button;

  uint16_t poti = analogRead(POTI_PIN);
  #if 0
  analogWrite(MOTOR_FORWARD_PIN, poti>>2);
  static uint32_t last =0;
  if (millis() - last > 100) {
    DEBUG_printf(FST("%d: PWM: %d \n"), start, poti>>2);
    last = millis();
  }
  #endif

#if 1
  setpoint = map(poti, 0, 4095, -MOTOR_PWM_RANGE, MOTOR_PWM_RANGE);
  input = map(position, 0, ENCODER_RESOLUTION, -MOTOR_PWM_RANGE, MOTOR_PWM_RANGE);
  positionPID.Compute();

  static uint32_t last =0;
  if (millis() - last > 100) {
    DEBUG_printf(FST("%d: Angle: %d Poti: %d, Setpoint: %f Input: %f | Output: %f\n"), start, angle, poti, input, setpoint, output);
    last = millis();
  }

  if (output > 10) {
    analogWrite(MOTOR_FORWARD_PIN, 0);
    analogWrite(MOTOR_REVERSE_PIN, output + MOTOR_PWM_START);
  } else if (output < -10) {
    analogWrite(MOTOR_FORWARD_PIN, -output + MOTOR_PWM_START);
    analogWrite(MOTOR_REVERSE_PIN, 0);
  }
  else {
    analogWrite(MOTOR_FORWARD_PIN, 0);
    analogWrite(MOTOR_REVERSE_PIN, 0);
  }
  #endif
  


  // analogWrite(MOTOR_FORWARD_PIN, poti>>2);
  // // Output a record if there was a change.
  // if (change) {
  //   DEBUG_printf(FST("%d: Angle: %d | Rot: %d, | Pos: %d | Poti: %d\n"), start, angle, rotations, position);
  // }


  // Wait for the next loop.
  delay(LOOP_DELAY - (millis() % LOOP_DELAY));
}

#if 0
bool save() {
      for (int i = 0; i < ENCODER_COUNT; i++) {
      home_angle[i] = data[i].angle;
    }
    change = true;
    // Save calibration angles to EEPROM.
    EEPROM.begin(EEPROM_SIZE);
    eeprom_data_t eeprom_data;
    eeprom_data.magic = EEPROM_MAGIC;
    memcpy(eeprom_data.home_angle, home_angle, sizeof(home_angle));
    EEPROM.put(0, eeprom_data);
    EEPROM.end();
    DEBUG_println(FST("# Saved Home Position to EEPROM"));

}
#endif