/*
  Arduino Engine Room Order Telegraph (AEROT) is an arduino program that simulates an order telegraph for a vessel's engine room with an LCD display, connecting the control room and the engine room through a serial connection.

  This program is made to run on an Arduino MEGA.

  - Required components:
    - LCD display (16 columns X 2 rows)
    - Push button
    - Rotary encoder

*/

#include <LiquidCrystal.h>;

//   [[ CONSTANTS - PINOUT ]]

// DIGITAL
const byte LCD_RS_PIN PROGMEM = 2;
const byte LCD_E_PIN PROGMEM = 3;
const byte LCD_D4_PIN PROGMEM = 4;
const byte LCD_D5_PIN PROGMEM = 5;
const byte LCD_D6_PIN PROGMEM = 6;
const byte LCD_D7_PIN PROGMEM = 7;

const byte BUTTON_ACKNOWLEDGE_PIN PROGMEM = 8;

const byte BUZZER_PIN PROGMEM = 9; // PWM

const byte SHIFT_REGISTER_LATCH_PIN PROGMEM = 10; // 74HC595 pin 9 STCP
const byte SHIFT_REGISTER_CLOCK_PIN PROGMEM = 11; // 74HC595 pin 10 SHCP
const byte SHIFT_REGISTER_DATA_PIN PROGMEM = 12; // 74HC595 pin 8 DS

// For manual RPM input only (set by RPM_AUTOMATIC_INPUT below)
const byte ROTARY_ENCODER_CLOCK PROGMEM = 20; // Must have an interrupt
const byte ROTARY_ENCODER_DIRECTION PROGMEM = 13;

// (Optional) For automatic RPM input, reading NMEA sentences from a serial port (set by RPM_AUTOMATIC_INPUT below)
const unsigned short RPM_INPUT_SERIAL_BAUD_RATE PROGMEM = 4800;

//   [[ CONSTANTS - OPERATION ]]

// SAIL
const byte NUMBER_OF_SPEEDS PROGMEM = 5;
const byte SAIL_SPEED_KNOTS[NUMBER_OF_SPEEDS] = { 2, 5, 10, 20, 22 }; // This array must have "NUMBER_OF_SPEEDS" length
const byte SAIL_SPEED_RPM[NUMBER_OF_SPEEDS] = { 9, 22, 45, 90, 100 }; // This array must have "NUMBER_OF_SPEEDS" length

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

const char DISPLAY_MSG_INITIALIZING_LINE_1[] PROGMEM = "INITIALIZING    "; // This string must be 16 characters long

// BUZZER
const bool DISABLE_BUZZER PROGMEM = false;
const unsigned short BUZZER_NOT_ACKNOWLEDGED_FREQUENCY PROGMEM = 2600; // ;-)
const unsigned short BUZZER_RING_BELL_FREQUENCY PROGMEM = 5200;
const unsigned short BUZZER_RING_BELL_DURATION PROGMEM = 500; // ms

// NMEA SENTENCES
const bool RPM_AUTOMATIC_INPUT PROGMEM = false; // true = use a rotary encoder for the user to input the speed, false = the RPM value will be read from NMEA sentences from the Serial2 input of the Arduino
const String RPM_NMEA_SENTENCE_START = "$ERRPM,S,1,"; // (Optional, only when RPM_AUTOMATIC_INPUT is set to true) Engine Room Monitoring Systems, Shaft #1

// DIGITAL TRANSMISSION
const unsigned short TELEGRAPH_SERIAL_BAUD_RATE PROGMEM = 9600; // This variable must have its value matched with the value on the engine room telegraph

const byte MESSAGE_TYPE_STATUS_UPDATE PROGMEM = 1;
const byte MESSAGE_TYPE_STATUS_ACKNOWLEDGE PROGMEM = 2;
const byte MESSAGE_TYPE_SET_RPM PROGMEM = 3;
const byte MESSAGE_TYPE_RING_BELL PROGMEM = 4;
const byte MESSAGE_TYPE_PING PROGMEM = 5;

const unsigned short PING_TIMEOUT PROGMEM = 8000; // ms. This variable must have its value matched with the value on the engine room telegraph

//   [[ VARIABLES ]]
LiquidCrystal lcd(LCD_RS_PIN, LCD_E_PIN, LCD_D4_PIN, LCD_D5_PIN, LCD_D6_PIN, LCD_D7_PIN);

unsigned long lastPongTime = 0;
bool disconnected = false;

volatile boolean turnDetected; // need volatile for Interrupts
volatile boolean rotationDirection; // CW or CCW rotation



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

    void setStatus(byte status[]) {
      this->direction = status[0] >> 4; // 4 bits = direction, 4 bytes = speed
      this->speed = status[0]  & 0b00001111;

      this->acknowledged = this->isAcknowledged(acknowledgedSail);

      lcd.setCursor(0, 0);
      lcd.print(DISPLAY_SAIL_DIRECTION[this->direction]);
      this->printAcknowledged();
      this->printSpeed();
    }

    void acknowledge();

    void changeRPM(bool increase);

    void setRPM(byte RPM);

    void ringAlarmBell() {
      if (DISABLE_BUZZER) {
        return;
      }
      if (this->acknowledged) {
        tone(BUZZER_PIN, BUZZER_RING_BELL_FREQUENCY, BUZZER_RING_BELL_DURATION);
      }
    }

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
        if (!DISABLE_BUZZER) {
          noTone(BUZZER_PIN);
        }
      } else {
        lcd.setCursor(DISPLAY_ACKNOWLEDGED % DISPLAY_COLUMNS, DISPLAY_ACKNOWLEDGED / DISPLAY_COLUMNS);
        lcd.print("*");
        if (!DISABLE_BUZZER) {
          tone(BUZZER_PIN, BUZZER_NOT_ACKNOWLEDGED_FREQUENCY);
        }
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

};

SailStatus* sail = new SailStatus();



void rotaryEncoderISR() {
  // TODO! delay(4);  // delay for Debouncing
  if (digitalRead(ROTARY_ENCODER_CLOCK))
    rotationDirection = digitalRead(ROTARY_ENCODER_DIRECTION);
  else
    rotationDirection = !digitalRead(ROTARY_ENCODER_DIRECTION);
  turnDetected = true;
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(TELEGRAPH_SERIAL_BAUD_RATE);

  // BUZZER
  pinMode(BUZZER_PIN, OUTPUT);

  lcd.begin(DISPLAY_COLUMNS, DISPLAY_ROWS);
  lcd.clear();

  lcd.setCursor(0, 0);
  char charBuf[sizeof(DISPLAY_INITIAL_LINE_1)];
  memcpy(charBuf, DISPLAY_MSG_INITIALIZING_LINE_1, sizeof DISPLAY_MSG_INITIALIZING_LINE_1);
  lcd.print(charBuf);

  // ACKNOWLEDGE BUTTON
  pinMode(BUTTON_ACKNOWLEDGE_PIN, INPUT_PULLUP);

  if (RPM_AUTOMATIC_INPUT) {
    // NMEA SERIAL INPUT
    Serial2.begin(RPM_INPUT_SERIAL_BAUD_RATE);
  } else {
    // ROTARY ENCODER
    pinMode(ROTARY_ENCODER_CLOCK, INPUT);
    pinMode(ROTARY_ENCODER_DIRECTION, INPUT);
    attachInterrupt(digitalPinToInterrupt(ROTARY_ENCODER_CLOCK), rotaryEncoderISR, FALLING);
  }

  lcd.setCursor(0, 0);
  memcpy(charBuf, DISPLAY_INITIAL_LINE_1, sizeof DISPLAY_INITIAL_LINE_1);
  lcd.print(charBuf);

  lcd.setCursor(0, 1);
  memcpy(charBuf, DISPLAY_INITIAL_LINE_2, sizeof DISPLAY_INITIAL_LINE_2);
  lcd.print(charBuf);
}

void loop() {
  // SERIAL FROM THE CONTROL ROOM
  receiveMessage();

  if (digitalRead(BUTTON_ACKNOWLEDGE_PIN) == LOW) { // ACKNOWLEDGE BUTTON
    sail->acknowledge();
  }

  if (RPM_AUTOMATIC_INPUT) {
    // NMEA SERIAL INPUT
    if (Serial2.available() > 0) {
      bool dataDecoded = false;
      String sentence = Serial2.readStringUntil('\n');
      byte RPM = getRPMfromNMEAsentence(sentence, dataDecoded);
      if (dataDecoded) {
        sail->setRPM(RPM);
      }
    }
  } else {
    // ROTARY ENCODER
    if (turnDetected) {
      sail->changeRPM(rotationDirection);
      turnDetected = false;
    }
  }

  if (!disconnected) {
    if ((millis() - lastPongTime) > PING_TIMEOUT) {
      disconnected = true;
      printDisconnected();
    }
  }
}

// TODO! Warning! Could get stuck!
void receiveMessage() {
  if (Serial1.available() == 0) {
    return;
  }
  byte header = Serial1.read(); // 4 bits = message type, 4 bytes = message length
  // Serial.print(header);
  // Serial.print(" ");
  byte messageType = header >> 4;
  byte messageLength = header & 0b00001111;
  byte message[messageLength];
  byte checksum = header;
  for (byte i = 0; i < messageLength; i++) {
    while (Serial1.available() == 0);
    message[i] = Serial1.read();
    checksum ^= message[i];
    // Serial.print(message[i]);
    // Serial.print(" ");
  }
  // Serial.println("");
  // Read the checksum
  while (Serial1.available() == 0);
  byte remoteChecksum = Serial1.read();
  if (checksum != remoteChecksum) {
    return;
  }

  switch (messageType) {
    case MESSAGE_TYPE_STATUS_UPDATE: {
        sail->setStatus(message);
        break;
      }
    case MESSAGE_TYPE_RING_BELL: {
        sail->ringAlarmBell();
        break;
      }
    case MESSAGE_TYPE_PING: {
        sendMessage(MESSAGE_TYPE_PING, 0, nullptr);
        lastPongTime = millis();
        break;
      }
  }
}

void sendMessage(byte messageType, byte messageLength, byte message[]) {
  if (messageType > 15 || messageLength > 15) {
    return;
  }
  byte headerByte = (messageType << 4) | (messageLength & 0b00001111); // 4 bits = message type, 4 bytes = message length

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

byte getRPMfromNMEAsentence(String sentence, bool& dataDecoded) {
  sentence.trim();
  sentence.toUpperCase();
  // Check header
  if (sentence.length() < 19 || !sentence.startsWith(RPM_NMEA_SENTENCE_START)) {
    dataDecoded = false;
    return 0;
  }

  // Check checksum
  byte checksum = 0;
  for (byte i = 1; i < sentence.length(); i++) {
    if (sentence.charAt(i) == '*') {
      break;
    }
    checksum ^= sentence.charAt(i);
  }
  String endOfSentence = String("*");
  endOfSentence.concat(String(checksum, HEX));
  endOfSentence.toUpperCase();
  if (!sentence.endsWith(endOfSentence)) {
    dataDecoded = false;
    return 0;
  }

  // Extract data
  byte commas = 0;
  for (byte i = 0; i < sentence.length(); i++) {
    if (sentence.charAt(i) == ',') {
      commas++;
      if (commas == 3) {

        byte endPosition = 0;
        for (byte j = i + 1; j < sentence.length(); j++) {
          if (sentence.charAt(j) == ',') {
            endPosition = j;
            break;
          }
        }

        if (endPosition == 0 || endPosition - i <= 1) {
          dataDecoded = false;
          return;
        }

        String speedInString = sentence.substring(i + 1, endPosition - 1);
        float speed = speedInString.toFloat();

        dataDecoded = true;
        return speed;
      } // if (commas == 3)
    } // if (sentence.charAt(i) == ',')
  } // for (byte i = 0; i < sentence.length(); i++)

  dataDecoded = false;
  return 0;
}

void SailStatus::acknowledge() {
  if (this->acknowledged) {
    return;
  }
  this->acknowledged = true;
  acknowledgedSail->direction = this->direction;
  acknowledgedSail->speed = this->speed;
  this->printAcknowledged();
  sendMessage(MESSAGE_TYPE_STATUS_ACKNOWLEDGE, 0, nullptr);
}

void SailStatus::changeRPM(bool increase) {
  if (increase) {
    if (this->currentRPM < 255) {
      this->currentRPM++;
    }
  } else {
    if (this->currentRPM > 0) {
      this->currentRPM--;
    }
  }
  this->printRPM();

  byte message[1];
  message[0] = this->currentRPM;
  sendMessage(MESSAGE_TYPE_SET_RPM, 1, message);
}

void SailStatus::setRPM(byte RPM) {
  if (this->currentRPM == RPM) {
    return;
  }
  this->currentRPM = RPM;
  this->printRPM();

  byte message[1];
  message[0] = this->currentRPM;
  sendMessage(MESSAGE_TYPE_SET_RPM, 1, message);
}

void printDisconnected() {
  lcd.setCursor(DISPLAY_ACKNOWLEDGED % DISPLAY_COLUMNS, DISPLAY_ACKNOWLEDGED / DISPLAY_COLUMNS);
  lcd.print("!");
}
