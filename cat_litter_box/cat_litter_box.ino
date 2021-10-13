/*-----------------includes-----------------*/
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>

/*-----------------define-----------------*/
#define MOTION_SENSOR_PIN 2 // only 2, 3 can be interrupts

/*-----------------typedef and struct-----------------*/
struct poop_event {
  int cat_id;
  int duration;
};

/*-----------------global var-----------------*/
unsigned long start_time;
unsigned long end_time;
const int chipSelect = 10;

SoftwareSerial WiFi_serial (6, 5); // RX, TX

void setup() {
  /*misc*/
  Serial.begin(9600);
  WiFi_serial.begin(9600);
  /*init peripherals: wifi, RFID reader, motion sensor, SD card*/
  SD_card_init();
  //WiFi_init();
  /*set pin mode*/
  pinMode(MOTION_SENSOR_PIN, INPUT);
  /*set interrupt*/
  attachInterrupt(digitalPinToInterrupt(MOTION_SENSOR_PIN), test_motion_sensor, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(MOTION_SENSOR_PIN), motion_sensor_handler, CHANGE);
}

void loop() {
  /*Check cmd via serial, can be replaced by WiFi to receive remote instruction*/
  if(Serial.available()) {
    String cmd = Serial.readString();
    Serial.println("Command received: " + cmd);
    if(cmd == "Send\n") {
      send_data();
    }
  }
}

/*-----------------functions-----------------*/
void motion_sensor_handler() {
  if (digitalRead(MOTION_SENSOR_PIN)) { // check whether enter or leave
    /*enter handler*/
    RFID_read();
    start_time = millis(); // can also record time in real word. may need internet
  } else {
    /*leave handler*/
    end_time = millis();
    SDcard_write();
  }
  return;
}

void send_data() {
  Serial.println("Send request received!");
  SDcard_read();
  //TODO: send latest data
  return;
}

int RFID_read() {
  //TODO: implement RFID tag read
  return 0;
}

void SD_card_init() {
  Serial.print("Initializing SD card...");
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    while (1);
  }
  Serial.println("card initialized.");
}

void SDcard_read() {
  File data_log = SD.open("datalog.txt");
  if(data_log) {
    Serial.println("datalog.txt:");
    while(data_log.available()) {
      Serial.write(data_log.read());
    }
    data_log.close();
  } else {
    Serial.println("error opening file");
  }
}

void SDcard_write() {
  File data_log = SD.open("datalog.txt", FILE_WRITE);
  if(data_log) {
    data_log.println("new entry");
    data_log.close();
  } else {
    Serial.println("error opening file");
  }
  return;
}

/*-----------------------------test-----------------------------*/
void test_motion_sensor() {
  test_WiFi_connection();
  if (digitalRead(MOTION_SENSOR_PIN)) {
    Serial.println("Measuring!");
    start_time = millis();
  } else {
    end_time = millis();
    Serial.println((end_time-start_time)/1000);
  }
}

void test_WiFi_connection() {
  WiFi_serial.println("Can u see?"); // display this message in UDP client
}
