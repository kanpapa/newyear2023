#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <TimeLib.h>

#define GPIO_OUT *(volatile uint32_t *)0x3FF44004
#define GPIO_OUT_W1TS *(volatile uint32_t *)0x3FF44008
#define GPIO_OUT_W1TC *(volatile uint32_t *)0x3FF4400C

byte invalid[8] = { 0x69, 0x6e, 0x76, 0x61, 0x6c, 0x69, 0x64, 0 };      // INVALID
byte startmsg[8] = { 0x0a, 0x73, 0x74, 0x61, 0x72, 0x74, 0x0a, 0 };     // -START-
byte stopmsg[8] = { 0x0a, 0x00, 0x73, 0x74, 0x6f, 0x70, 0x00, 0x0a };   // * STOP *

#define time_offset 32400    // JST UTC + 9H (3600 * 9) sec

// UDC RAM data
const byte dotdata[16][7] = {
  { 0x1B, 0x12, 0x12, 0x1F, 0x15, 0x1B, 0x1F },  // rabbit1
  { 0x0A, 0x0A, 0x0E, 0x15, 0x1F, 0x11, 0x0E },  // rabbit2
  { 0x1f, 0x1e, 0x1c, 0x18, 0x18, 0x1c, 0x1e },
  { 0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10 },
  { 0x1B, 0x12, 0x12, 0x1F, 0x15, 0x1F, 0x1B },
  { 0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10 },
  { 0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10 },
  { 0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10 },
  { 0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10 },
  { 0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10 },
  { 0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10 },
  { 0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10 },
  { 0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10 },
  { 0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10 },
  { 0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10 },
  { 0x1f, 0x10, 0x10, 0x1e, 0x10, 0x10, 0x10 }
};

// 2023 NEW YEAR MESSAGES
const byte msg[7][8] = {
  { 0x00, 0x68, 0x61, 0x70, 0x70, 0x79, 0x00, 0x00 },  // HAPPY
  { 0x6e, 0x65, 0x77, 0x00, 0x79, 0x65, 0x61, 0x72 },  // YEAR
  { 0x81, 0x00, 0x12, 0x10, 0x12, 0x13, 0x00, 0x81 },  // 2023
  { 0x00, 0x31, 0x39, 0x4f, 0x3c, 0x43, 0x00, 0x00 },  // アケマシテ
  { 0x00, 0x35, 0x52, 0x43, 0x5e, 0x44, 0x33, 0x00 },  // オメデトウ
  { 0x3a, 0x5e, 0x3b, 0x5e, 0x32, 0x4f, 0x3d, 0x00 },  // ゴザイマス
  { 0x81, 0x00, 0x12, 0x10, 0x12, 0x13, 0x00, 0x81 }  // 2023
};


#define MYPORT_TX 32
#define MYPORT_RX 33

// The TinyGPSPlus object
TinyGPSPlus gps;

// Global vars
uint16_t data;
uint16_t adrs;

int ss_old;

// SoftwareSerial object
SoftwareSerial myPort;

// Write the Register
void write_reg(uint16_t adrs, uint16_t data, uint8_t a3, uint8_t a4) {
  // Set bit
  GPIO_OUT_W1TS = (data << 12) | (adrs << 21);  // D0-D7
  digitalWrite(25, a3);                         // A3
  digitalWrite(26, a4);                         // A4

  digitalWrite(5, LOW);  // ~WR
  delay(1);
  digitalWrite(5, HIGH);
  delay(1);

  // Clear bit
  GPIO_OUT_W1TC = (data << 12) | (adrs << 21);  // D0-D7
}

void displaytime(int hh, int mm, int ss) {
  int cols = 0;
  if (hh < 10) {
    write_reg(cols++, 16, HIGH, HIGH);
  } else {
    write_reg(cols++, (hh / 10) + 16, HIGH, HIGH);
  }
  write_reg(cols++, (hh % 10) + 16, HIGH, HIGH);

  write_reg(cols++, 0x1a, HIGH, HIGH);  // ":"

  if (mm < 10) {
    write_reg(cols++, 16, HIGH, HIGH);
  } else {
    write_reg(cols++, (mm / 10) + 16, HIGH, HIGH);
  }
  write_reg(cols++, (mm % 10) + 16, HIGH, HIGH);

  write_reg(cols++, 0x1a, HIGH, HIGH);  // ":"

  if (ss < 10) {
    write_reg(cols++, 16, HIGH, HIGH);
  } else {
    write_reg(cols++, (ss / 10) + 16, HIGH, HIGH);
  }
  write_reg(cols++, (ss % 10) + 16, HIGH, HIGH);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Program start.");

  // Setup Sortfare serial
  myPort.begin(9600, SWSERIAL_8N1, MYPORT_RX, MYPORT_TX, false);
  if (!myPort) {  // If the object did not initialize, then its configuration is invalid
    Serial.println("Invalid SoftwareSerial pin configuration, check config");
    while (1) {  // Don't continue with invalid configuration
      delay(1000);
    }
  }
  Serial.println("Initialize SoftwareSerial complited.");

  pinMode(4, OUTPUT);  // ~RST
  pinMode(5, OUTPUT);  // ~CE,~WR

  pinMode(12, OUTPUT);  // D0
  pinMode(13, OUTPUT);  // D1
  pinMode(14, OUTPUT);  // D2
  pinMode(15, OUTPUT);  // D3
  pinMode(16, OUTPUT);  // D4
  pinMode(17, OUTPUT);  // D5
  pinMode(18, OUTPUT);  // D6
  pinMode(19, OUTPUT);  // D7

  // XX
  pinMode(21, OUTPUT);  // A0
  pinMode(22, OUTPUT);  // A1
  pinMode(23, OUTPUT);  // A2

  // XX
  pinMode(25, OUTPUT);  // A3
  pinMode(26, OUTPUT);  // A4
  pinMode(27, OUTPUT);  // ~FL

  // INIT ESP32 PORT
  digitalWrite(4, HIGH);   // ~RESET
  digitalWrite(5, HIGH);   // ~CE, ~WR
  digitalWrite(27, HIGH);  // ~FL
  delay(1);

  // Display Reset
  digitalWrite(4, LOW);   // ~RESET
  delay(1);               // wait
  digitalWrite(4, HIGH);  // ~RESET
  delay(1);               // wait

  // Setup the Control Word Register
  data = 2;
  adrs = 0;
  write_reg(adrs, data, LOW, HIGH);

  // Load to the UDC RAM
  for (int udc = 0; udc < 16; udc++) {
    write_reg(0, udc, LOW, LOW);  // UDC Address Register

    for (adrs = 0; adrs < 7; adrs++) {
      write_reg(adrs, dotdata[udc][adrs], HIGH, LOW);  // UDC RAM Register
    }
  }

  // initrize global var
  data = 0;
  adrs = 0;

  // START MSG
  for (int k = 0; k < 8; k++) {
    write_reg(k, startmsg[k], HIGH, HIGH);
  }

  delay(3000);  // wait 3sec
}

// STOP MSG
void output_stopmsg() {
  for (int k = 0; k < 8; k++) {
    write_reg(k, stopmsg[k], HIGH, HIGH);
  }
}

// INVALID
void output_invalidmsg() {
  for (int j = 0; j < 8; j++) {
    write_reg(j, invalid[j], HIGH, HIGH);
  }
}

void loop() {
  while (myPort.available() > 0) {
    if (gps.encode((char)myPort.read())) {
      if(ss_old != gps.time.second()) {
        ss_old = gps.time.second();
        setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
        adjustTime(time_offset);
        int hh = hour();
        int mm = minute();
        int ss = second();

        // DEBUG
        Serial.print(hh);
        Serial.print(":");
        Serial.print(mm);
        Serial.print(":");
        Serial.println(ss);

        if (hh != 0) {  // Check 0:00 JST
          displaytime(hh, mm, ss);
        } else {
          while (1) {  // LOOP: HAPPY NEW YEAR 2023 //
            for (int msg_num = 0; msg_num < 7; msg_num++) {
              for (adrs = 0; adrs < 8; adrs++) {
                // Write the Character RAM
                write_reg(adrs, msg[msg_num][adrs], HIGH, HIGH);
                delay(200);
              }
            }
          }
        }
      } 
    }
  }
}
