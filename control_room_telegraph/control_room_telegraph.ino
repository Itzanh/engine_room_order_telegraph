/*
  Arduino Engine Room Order Telegraph (AEROT) is an arduino program that simulates an order telegraph for a vessel's engine room with an LCD display, connecting the control room and the engine room through a serial connection.

  This program is made to run on an Arduino MEGA.

  - Required components:
    - LCD display (16 columns X 2 rows)
    - Membrane keypad
    - 10 KOhm Potentiometer
  - Optional components:
    - DS3231 digital clock module
    - SDCard adapter module

*/

#include <Keypad.h>;
#include <LiquidCrystal.h>;
#include <Wire.h> // For the DS3231 digital clock
#include <DS3231.h> // Digital clock (optional for logging)
#include <SD.h> // SDCard writer (optional for logging)

//   [[ CONSTANTS - PINOUT ]]

// DIGITAL
const byte KEYBOARD_PIN_1_PIN PROGMEM = 2;
const byte KEYBOARD_PIN_2_PIN PROGMEM = 3;
const byte KEYBOARD_PIN_3_PIN PROGMEM = 4;
const byte KEYBOARD_PIN_4_PIN PROGMEM = 5;
const byte KEYBOARD_PIN_5_PIN PROGMEM = 6;
const byte KEYBOARD_PIN_6_PIN PROGMEM = 7;
const byte KEYBOARD_PIN_7_PIN PROGMEM = 8;
const byte KEYBOARD_PIN_8_PIN PROGMEM = 9;

// UNUSED! Still not implemented
const byte SHIFT_REGISTER_LATCH_PIN PROGMEM = 11; // 74HC595 pin 9 STCP
const byte SHIFT_REGISTER_CLOCK_PIN PROGMEM = 12; // 74HC595 pin 10 SHCP
const byte SHIFT_REGISTER_DATA_PIN PROGMEM = 13; // 74HC595 pin 8 DS

const byte BUZZER_PIN PROGMEM = 10; // PWM

const byte LCD_RS_PIN PROGMEM = 22;
const byte LCD_E_PIN PROGMEM = 23;
const byte LCD_D4_PIN PROGMEM = 24;
const byte LCD_D5_PIN PROGMEM = 25;
const byte LCD_D6_PIN PROGMEM = 26;
const byte LCD_D7_PIN PROGMEM = 27;

const byte SD_CARD_HOLDER_PIN PROGMEM = 53; // Optional, for logging only (set by ENABLE_LOGGING below)

//   [[ CONSTANTS - OPERATION ]]

// SAIL
const byte NUMBER_OF_SPEEDS PROGMEM = 5;
const byte SAIL_SPEED_KNOTS[NUMBER_OF_SPEEDS] = { 2, 5, 10, 20, 22 }; // This array must have "NUMBER_OF_SPEEDS" length
const byte SAIL_SPEED_RPM[NUMBER_OF_SPEEDS] = { 9, 22, 45, 90, 100 }; // This array must have "NUMBER_OF_SPEEDS" length

// ARDUINO
const unsigned short ARDUINO_ANALOG_MAX_VALUE PROGMEM = 1024;
const unsigned short SPEED_ANALOG_VALUE_THRESHOLD PROGMEM = (ARDUINO_ANALOG_MAX_VALUE / NUMBER_OF_SPEEDS);

// KEYBOARD
const byte KEYBOARD_ROWS = 4;
const byte KEYBOARD_COLUMNS = 4;
const char KEYBOARD_KEYS[KEYBOARD_ROWS][KEYBOARD_COLUMNS] = { // Define the symbols on the buttons of the keypads
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
const byte KEYBOARD_ROW_PINS[KEYBOARD_ROWS] = {KEYBOARD_PIN_8_PIN, KEYBOARD_PIN_7_PIN, KEYBOARD_PIN_6_PIN, KEYBOARD_PIN_5_PIN}; // Connect to the row pinouts of the keypad
const byte KEYBOARD_COLUMN_PINS[KEYBOARD_COLUMNS] = {KEYBOARD_PIN_4_PIN, KEYBOARD_PIN_3_PIN, KEYBOARD_PIN_2_PIN, KEYBOARD_PIN_1_PIN}; // Connect to the column pinouts of the keypad

// DISPLAY
const byte DISPLAY_COLUMNS PROGMEM = 16;
const byte DISPLAY_ROWS PROGMEM = 12;
const char* DISPLAY_SAIL_DIRECTION[3] = { "STOP  ", "AHEAD ", "ASTERN" }; // These strings must be 6 characters long
const byte DISPLAY_SAIL_SPEED_NAME_POSITION PROGMEM = 7;
const char* DISPLAY_SAIL_SPEED_NAMES[NUMBER_OF_SPEEDS] = { "DEADSLW", "SLOW   ", "HALF   ", "FULL   ", "FLANK  " }; // This array must have "NUMBER_OF_SPEEDS" length, and the strings must be 7 characters long
const byte DISPLAY_ACKNOWLEDGED PROGMEM = 15;
const byte DISPLAY_SAIL_SPEED_KNOTS_POSITION PROGMEM = 16;
const byte DISPLAY_SAIL_SPEED_RPM_POSITION PROGMEM = 23;
const byte DISPLAY_SAIL_SPEED_RPM_MISMATCH_POSITION PROGMEM = 31;

const char DISPLAY_INITIAL_LINE_1[] PROGMEM = "STOP            "; // This string must be 16 characters long
const char DISPLAY_INITIAL_LINE_2[] PROGMEM = "00 KTS 000 RPM ="; // This string must be 16 characters long

const char DISPLAY_MSG_INITIALIZING_LINE_1[]           PROGMEM = "INITIALIZING    "; // This string must be 16 characters long
const char DISPLAY_MSG_INITIALIZING_CLOCK_LINE_1[]     PROGMEM = "INITIALIZ. CLOCK"; // This string must be 16 characters long
const char DISPLAY_MSG_SD_CARD_INIT_ERR_CLOCK_LINE_1[] PROGMEM = "SD CARD INIT ERR"; // This string must be 16 characters long
const char DISPLAY_MSG_SD_CARD_FILE_ERR_CLOCK_LINE_1[] PROGMEM = "SD CARD FILE ERR"; // This string must be 16 characters long

// DIGITAL TRANSMISSION
const unsigned short TELEGRAPH_SERIAL_BAUD_RATE PROGMEM = 9600; // This variable must have its value matched with the value on the engine room telegraph

const byte MESSAGE_TYPE_STATUS_UPDATE PROGMEM = 1;
const byte MESSAGE_TYPE_STATUS_ACKNOWLEDGE PROGMEM = 2;
const byte MESSAGE_TYPE_SET_RPM PROGMEM = 3;
const byte MESSAGE_TYPE_RING_BELL PROGMEM = 4;
const byte MESSAGE_TYPE_PING PROGMEM = 5;

// LOGGING
// This option allows to log every single order to a SD Card. In order to be able to set this to true, you must have the optional components (the digital clock module and the SD Card adapter module) and have
// them connected to the arduino following the pinout described in the constants above.
const bool ENABLE_LOGGING PROGMEM = false;
const byte MESSAGE_TYPE_STARTUP PROGMEM = 14;
const byte MESSAGE_TYPE_DISCONNECTED PROGMEM = 15;
const char* LOGGING_FILE_NAME = "vdr.dat"; // The file must exist previously, copied to the SD Card from a computer, and must have a specific header. See the arduino_voyage_data_recorder_tool program for more info.

// The two variables below must have their values matched with the values on the engine room telegraph
const unsigned short PING_INTERVAL PROGMEM = 5000; // ms
const unsigned short PING_TIMEOUT PROGMEM = 8000; // ms (from PING_INVERVAL)

// ANALOG
const byte SPEED_POTENTIOMETER_PIN PROGMEM = A0;

//   [[ VARIABLES ]]
Keypad customKeypad = Keypad(makeKeymap(KEYBOARD_KEYS), KEYBOARD_ROW_PINS, KEYBOARD_COLUMN_PINS, KEYBOARD_ROWS, KEYBOARD_COLUMNS);
LiquidCrystal lcd(LCD_RS_PIN, LCD_E_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);
DS3231 clock;
File logFile;

unsigned long lastPingTime = 0;
unsigned long lastPongTime = 0;
bool disconnected = false;



class AcknowledgedSailStatus {
  public:
    byte direction; // 0 = Stop, 1 = Ahead, 2 = Astern
    byte speed; // 0 - 4 (Dead slow, Slow, Half, Full, Flank)

    AcknowledgedSailStatus() {
      this->direction = 0;
      this->speed = 0;
    }
};

AcknowledgedSailStatus* acknowledgedSail = new AcknowledgedSailStatus();

class SailStatus {
  private:
    byte direction; // 0 = Stop, 1 = Ahead, 2 = Astern
    byte speed; // 0 - 4 (Dead slow, Slow, Half, Full, Flank)
    bool acknowledged;
    byte currentRPM;

  public:
    SailStatus() {
      this->direction = 0;
      this->speed = 0;
      this->acknowledged = true;
      this->currentRPM = 0;
    }

    void setDirection(byte direction) {
      if (direction == this->direction) {
        return;
      }
      this->direction = direction;
      if (this->direction == 0) {
        this->speed = 0;
      }
      this->acknowledged = this->isAcknowledged(acknowledgedSail);

      this->sendNewStatus();

      lcd.setCursor(0, 0);
      lcd.print(DISPLAY_SAIL_DIRECTION[this->direction]);
      this->printAcknowledged();
      if (this->direction == 0) {
        this->printSpeed();
      }
    }

    void setSpeed(unsigned short analogSpeedValue) {
      if (this->direction == 0) {
        return;
      }
      byte speed = analogSpeedValue / SPEED_ANALOG_VALUE_THRESHOLD;
      if (speed >= NUMBER_OF_SPEEDS) {
        speed = NUMBER_OF_SPEEDS - 1;
      }
      if (speed == this->speed) {
        return;
      }
      this->speed = speed;
      this->acknowledged = this->isAcknowledged(acknowledgedSail);

      this->sendNewStatus();

      this->printAcknowledged();
      this->printSpeed();
    }

    void setAcknowledged() {
      if (this->acknowledged) {
        return;
      }
      this->acknowledged = true;
      acknowledgedSail->direction = this->direction;
      acknowledgedSail->speed = this->speed;
      this->printAcknowledged();
      // logEvent(MESSAGE_TYPE_STATUS_ACKNOWLEDGE, 0, nullptr); // TODO!
    }

    void setRPM(byte newRPM) {
      this->currentRPM = newRPM;
      this->printRPM();
      // logEvent(MESSAGE_TYPE_SET_RPM, 0, nullptr); // TODO!
    }

    void ringAlarmBell();

  private:
    bool isAcknowledged(AcknowledgedSailStatus* acknowledgedSail) {
      return (this->direction == acknowledgedSail->direction) && (this->speed == acknowledgedSail->speed);
    }

    void printSpeed() {
      if (this->direction == 0) {
        lcd.setCursor(DISPLAY_SAIL_SPEED_NAME_POSITION % DISPLAY_COLUMNS, DISPLAY_SAIL_SPEED_NAME_POSITION / DISPLAY_COLUMNS);
        lcd.print("       ");
        lcd.setCursor(DISPLAY_SAIL_SPEED_KNOTS_POSITION % DISPLAY_COLUMNS, DISPLAY_SAIL_SPEED_KNOTS_POSITION / DISPLAY_COLUMNS);
        lcd.print("00");
      } else {
        lcd.setCursor(DISPLAY_SAIL_SPEED_NAME_POSITION % DISPLAY_COLUMNS, DISPLAY_SAIL_SPEED_NAME_POSITION / DISPLAY_COLUMNS);
        lcd.print(DISPLAY_SAIL_SPEED_NAMES[this->speed]);
        lcd.setCursor(DISPLAY_SAIL_SPEED_KNOTS_POSITION % DISPLAY_COLUMNS, DISPLAY_SAIL_SPEED_KNOTS_POSITION / DISPLAY_COLUMNS);
        char buffer[3];
        buffer[2] = 0x00; // null-terminated
        memset(buffer, '0', 2);
        sprintf(buffer, "%02d", SAIL_SPEED_KNOTS[this->speed]);
        lcd.print(buffer);
      }
      this->printKnotsWithRPMMismatch();
    }

    void printAcknowledged() {
      if (this->acknowledged) {
        lcd.setCursor(DISPLAY_ACKNOWLEDGED % DISPLAY_COLUMNS, DISPLAY_ACKNOWLEDGED / DISPLAY_COLUMNS);
        lcd.print(" ");
      } else {
        lcd.setCursor(DISPLAY_ACKNOWLEDGED % DISPLAY_COLUMNS, DISPLAY_ACKNOWLEDGED / DISPLAY_COLUMNS);
        lcd.print("*");
      }
    }

    void printRPM() {
      lcd.setCursor(DISPLAY_SAIL_SPEED_RPM_POSITION % DISPLAY_COLUMNS, DISPLAY_SAIL_SPEED_RPM_POSITION / DISPLAY_COLUMNS);
      char buffer[4];
      buffer[3] = 0x00; // null-terminated
      memset(buffer, '0', 3);
      sprintf(buffer, "%03d", this->currentRPM);
      lcd.print(buffer);
      this->printKnotsWithRPMMismatch();
    }

    void printKnotsWithRPMMismatch() {
      byte targetRPM = 0;
      if (this->direction != 0) {
        targetRPM = SAIL_SPEED_RPM[this->speed];
      }

      lcd.setCursor(DISPLAY_SAIL_SPEED_RPM_MISMATCH_POSITION % DISPLAY_COLUMNS, DISPLAY_SAIL_SPEED_RPM_MISMATCH_POSITION / DISPLAY_COLUMNS);
      if (this->currentRPM < targetRPM) {
        lcd.print("<");
      } else if (this->currentRPM > targetRPM) {
        lcd.print(">");
      } else {
        lcd.print("=");
      }
    }

    void sendNewStatus();

};

SailStatus* sail = new SailStatus();



void setup() {
  Serial.begin(115200);
  Serial1.begin(TELEGRAPH_SERIAL_BAUD_RATE);

  pinMode(SHIFT_REGISTER_DATA_PIN, OUTPUT);
  pinMode(SHIFT_REGISTER_CLOCK_PIN, OUTPUT);
  pinMode(SHIFT_REGISTER_LATCH_PIN, OUTPUT);

  lcd.begin(DISPLAY_COLUMNS, DISPLAY_ROWS);
  lcd.clear();

  lcd.setCursor(0, 0);
  char charBuf[sizeof(DISPLAY_INITIAL_LINE_1)];
  memcpy(charBuf, DISPLAY_MSG_INITIALIZING_LINE_1, sizeof DISPLAY_MSG_INITIALIZING_LINE_1);
  lcd.print(charBuf);

  if (ENABLE_LOGGING) {
    lcd.setCursor(0, 0);
    memcpy(charBuf, DISPLAY_MSG_INITIALIZING_CLOCK_LINE_1, sizeof DISPLAY_MSG_INITIALIZING_CLOCK_LINE_1);
    lcd.print(charBuf);
    clock.begin();

    if (!SD.begin(SD_CARD_HOLDER_PIN)) {
      lcd.setCursor(0, 0);
      memcpy(charBuf, DISPLAY_MSG_SD_CARD_INIT_ERR_CLOCK_LINE_1, sizeof DISPLAY_MSG_SD_CARD_INIT_ERR_CLOCK_LINE_1);
      lcd.print(charBuf);
      while (true);
    }
    displayShipName();
    logFile = SD.open(LOGGING_FILE_NAME, FILE_WRITE);
    if (!logFile) {
      lcd.setCursor(0, 0);
      memcpy(charBuf, DISPLAY_MSG_SD_CARD_FILE_ERR_CLOCK_LINE_1, sizeof DISPLAY_MSG_SD_CARD_FILE_ERR_CLOCK_LINE_1);
      lcd.print(charBuf);
      while (true);
    }
  }

  logStartUpEvent();

  lcd.setCursor(0, 0);
  memcpy(charBuf, DISPLAY_INITIAL_LINE_1, sizeof DISPLAY_INITIAL_LINE_1);
  lcd.print(charBuf);

  lcd.setCursor(0, 1);
  memcpy(charBuf, DISPLAY_INITIAL_LINE_2, sizeof DISPLAY_INITIAL_LINE_2);
  lcd.print(charBuf);

}

void readKeypad() {
  char pressedKey = customKeypad.getKey();
  switch (pressedKey) {
    case 'A': {
        sail->setDirection(1);
        break;
      }
    case 'B': {
        sail->setDirection(0);
        break;
      }
    case 'C': {
        sail->setDirection(2);
        break;
      }
    case 'D': {
        sail->ringAlarmBell();
        break;
      }
  }
}

void readSpeedPotentiometer() {
  unsigned short analogSpeedValue = analogRead(SPEED_POTENTIOMETER_PIN);
  sail->setSpeed(analogSpeedValue);
}

void loop() {
  readKeypad();
  readSpeedPotentiometer();
  receiveMessage();

  if (!disconnected) {
    if ((millis() - lastPongTime) > PING_TIMEOUT) {
      disconnected = true;
      printDisconnected();
      logEvent(MESSAGE_TYPE_DISCONNECTED, 0, nullptr);
    }
    if ((millis() - lastPingTime) > PING_INTERVAL) {
      sendMessage(MESSAGE_TYPE_PING, 0, nullptr);
      lastPingTime = millis();
    }
  }
}

// TODO! Warning! Could get stuck!
void receiveMessage() {
  if (Serial1.available() == 0) {
    return;
  }
  byte header = Serial1.read(); // 4 bits = message type, 4 bytes = message length
  byte messageType = header >> 4;
  byte messageLength = header & 0b00001111;
  byte message[messageLength];
  byte checksum = header;

  for (byte i = 0; i < messageLength; i++) {
    while (Serial1.available() == 0);
    message[i] = Serial1.read();
    checksum ^= message[i];
  }
  // Read the checksum
  while (Serial1.available() == 0);
  byte remoteChecksum = Serial1.read();
  if (checksum != remoteChecksum) {
    Serial.println("Checksum error");
    return;
  }

  switch (messageType) {
    case MESSAGE_TYPE_STATUS_ACKNOWLEDGE: {
        sail->setAcknowledged();
        break;
      }
    case MESSAGE_TYPE_SET_RPM: {
        if (messageLength == 1) {
          sail->setRPM(message[0]);
        }
        break;
      }
    case MESSAGE_TYPE_PING: {
        lastPongTime = millis();
        break;
      }
  }
}

void sendMessage(byte messageType, byte messageLength, byte message[]) {
  if (messageType > 15 || messageLength > 15) {
    return;
  }
  Serial.print(messageType);
  Serial.print(" ");
  Serial.print(messageLength);
  Serial.print(" ");
  byte headerByte = (messageType << 4) | (messageLength & 0b00001111); // 4 bits = message type, 4 bytes = message length
  Serial.println(headerByte);

  // 1-byte header, message, 1-byte checksum
  byte messageToSend[2 + messageLength];
  messageToSend[0] = headerByte;
  if (messageLength > 0) {
    memcpy(messageToSend + 1, message, messageLength);
  }
  // calculate the checksum
  messageToSend[messageLength + 1] = 0;
  for (byte i = 0; i < messageLength + 1; i++) {
    messageToSend[messageLength + 1] ^= messageToSend[i];
  }
  Serial1.write(messageToSend, 2 + messageLength);
}

void SailStatus::sendNewStatus() {
  // Serialize the current status
  byte serialized[1];
  serialized[0] = (this->direction << 4) | (this->speed & 0b00001111); // 4 bits = direction, 4 bytes = speed
  sendMessage(MESSAGE_TYPE_STATUS_UPDATE, 1, serialized);

  logEvent(MESSAGE_TYPE_STATUS_UPDATE, 1, serialized);
}

void SailStatus::ringAlarmBell() {
  Serial.println(MESSAGE_TYPE_RING_BELL);
  sendMessage(MESSAGE_TYPE_RING_BELL, 0, nullptr);
  logEvent(MESSAGE_TYPE_RING_BELL, 0, nullptr);
}

void printDisconnected() {
  lcd.setCursor(DISPLAY_ACKNOWLEDGED % DISPLAY_COLUMNS, DISPLAY_ACKNOWLEDGED / DISPLAY_COLUMNS);
  lcd.print("!");
}

void logEvent(byte messageType, byte messageLength, byte message[]) {
  if (!ENABLE_LOGGING) {
    return;
  }
  RTCDateTime dt = clock.getDateTime();

  /* +----------------------+---------------------+-----+--------+----------------+
     | 4 bytes              | 1 byte              |     | 1 byte | 4 bytes        |
     | unix time in seconds | event type + length | ... | CRC    | FF  FF  FF  FF |
     +----------------------+---------------------+-----+--------+----------------+
  */
  byte dataLoggedLength = 10 + messageLength;
  byte dataLogged[dataLoggedLength];

  // Copy the unix timestamp in the first 4 bytes
  memcpy(dataLogged, &dt.unixtime, sizeof(uint32_t));

  // Add the header in the 4th byte
  byte headerByte = (messageType << 4) | (messageLength & 0b00001111); // 4 bits = message type, 4 bytes = message length
  dataLogged[4] = headerByte;

  // Copy the event details after the 5th byte
  if (messageLength > 0) {
    memcpy(dataLogged + 5, message, messageLength);
  }

  // calculate the checksum, and set in the 5th last byte
  dataLogged[dataLoggedLength - 5] = 0;
  for (byte i = 0; i < dataLoggedLength - 5; i++) {
    dataLogged[dataLoggedLength - 5] ^= dataLogged[i];
  }

  // Set the last four bytes to FF
  memset(dataLogged + dataLoggedLength - 4, 255, 4);

  logFile.write(dataLogged, dataLoggedLength);
  logFile.flush();
}

void logStartUpEvent() {
  byte dataLogged[4];
  unsigned long compildTimestamp = getUnixTimestampFromCompileDateAndTime(__DATE__, __TIME__);
  // Copy the unix timestamp in the first 4 bytes
  memcpy(dataLogged, &compildTimestamp, sizeof(uint32_t));
  logEvent(MESSAGE_TYPE_STARTUP, sizeof(uint32_t), dataLogged);
}

void displayShipName() {
  if (!ENABLE_LOGGING) {
    return;
  }

  File logFile = SD.open(LOGGING_FILE_NAME, FILE_READ);
  if (!logFile) {
    return;
  }
  char charBuf[sizeof(32)];
  logFile.read(charBuf, sizeof(charBuf));
  logFile.close();

  lcd.setCursor(0, 0);
  memcpy(charBuf, charBuf, sizeof charBuf);
  lcd.print(charBuf);
  delay(2000);
}

// CONVERT __DATE__ AND __TIME__ TO UNIX TIMESTAMP //

const uint8_t daysArray [] PROGMEM = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };

uint8_t conv2d(const char* p)
{
  uint8_t v = 0;

  if ('0' <= *p && *p <= '9')
  {
    v = *p - '0';
  }

  return 10 * v + *++p - '0';
}

long time2long(uint16_t days, uint8_t hours, uint8_t minutes, uint8_t seconds)
{
  return ((days * 24L + hours) * 60 + minutes) * 60 + seconds;
}

bool isLeapYear(uint16_t year)
{
  return (year % 4 == 0);
}

uint16_t date2days(uint16_t year, uint8_t month, uint8_t day)
{
  year = year - 2000;

  uint16_t days16 = day;

  for (uint8_t i = 1; i < month; ++i)
  {
    days16 += pgm_read_byte(daysArray + i - 1);
  }

  if ((month == 2) && isLeapYear(year))
  {
    ++days16;
  }

  return days16 + 365 * year + (year + 3) / 4 - 1;
}

unsigned long getUnixTimestampFromCompileDateAndTime(const char* date, const char* time)
{
  Serial.println(date);
  Serial.println(time);
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t minute;
  uint8_t second;

  year = conv2d(date + 9);

  switch (date[0])
  {
    case 'J': month = date[1] == 'a' ? 1 : month = date[2] == 'n' ? 6 : 7; break;
    case 'F': month = 2; break;
    case 'A': month = date[2] == 'r' ? 4 : 8; break;
    case 'M': month = date[2] == 'r' ? 3 : 5; break;
    case 'S': month = 9; break;
    case 'O': month = 10; break;
    case 'N': month = 11; break;
    case 'D': month = 12; break;
  }

  day = conv2d(date + 4);
  hour = conv2d(time);
  minute = conv2d(time + 3);
  second = conv2d(time + 6);

  unsigned long u;

  u = time2long(date2days(year, month, day), hour, minute, second);
  // u += 946681200;
  u += 360885601;

  return u;
}
