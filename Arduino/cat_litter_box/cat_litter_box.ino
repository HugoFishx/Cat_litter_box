/*--------------------------------includes--------------------------------*/
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>
#include <LowPower.h>

/*--------------------------------define--------------------------------*/
#define MOTION_SENSOR_PIN 2 // only 2, 3 can be interrupts
#define WAKE_UP_PIN 3
#define WiFi_SERIAL_RX 6
#define WiFi_SERIAL_TX 5
#define RFID_SERIAL_RX 8
#define RFID_SERIAL_TX 7
#define RFID_READ_DELAY 1000
#define DEBUG_MODE 1
#define WiFi_CHIPSELECT 10
#define WiFi_MOSI 11
#define WiFi_MISO 12
#define WiFi_CLK 13

/*--------------------------------typedef and struct--------------------------------*/
struct poop_event {
  unsigned cat_id;
  int duration;
};

/*--------------------------------global var--------------------------------*/
unsigned long start_time;
unsigned long end_time;

// RFID
const int BUFFER_SIZE = 14; // RFID DATA FRAME FORMAT: 1byte head (value: 2), 10byte data (2byte version + 8byte tag), 2byte checksum, 1byte tail (value: 3)
const int DATA_SIZE = 10; // 10byte data (2byte version + 8byte tag)
const int DATA_VERSION_SIZE = 2; // 2byte version (actual meaning of these two bytes may vary)
const int DATA_TAG_SIZE = 8; // 8byte tag
const int CHECKSUM_SIZE = 2; // 2byte checksum

struct poop_event new_event;

SoftwareSerial WiFi_serial (WiFi_SERIAL_RX, WiFi_SERIAL_TX);
SoftwareSerial RFID_serial (RFID_SERIAL_RX, RFID_SERIAL_TX);

/*--------------------------------main--------------------------------*/
void setup() {
  /*misc.*/
  Serial.begin(9600);
  WiFi_serial.begin(9600);
  RFID_serial.begin(9600);
  /*init peripherals: wifi, RFID reader, motion sensor, SD card*/
  SD_card_init();
  WiFi_init();
  /*set pin mode*/
  pinMode(MOTION_SENSOR_PIN, INPUT);
  /*set interrupt*/
  //attachInterrupt(digitalPinToInterrupt(MOTION_SENSOR_PIN), test_motion_sensor, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTION_SENSOR_PIN), motion_sensor_handler, CHANGE);
}

void loop() {
  // Check cmd via serial, can be replaced by WiFi to receive remote instruction
  cmd_handler();
}

/*--------------------------------interuppt handler--------------------------------*/
// Handle motino sensor interrupt both high and low; record tag, duration in SD card
void motion_sensor_handler() {
  if (digitalRead(MOTION_SENSOR_PIN)) { // check whether enter or leave
    /*enter handler*/
    debug_print("Cat enter!");
    start_time = millis(); // can also record time in real word. may need internet
    RFID_serial.listen(); // only one software serial can used at the same time
    new_event.cat_id = RFID_read();
    debug_print("ID acquired!");
    WiFi_serial.listen();
  } else {
    /*leave handler*/
    debug_print("Cat left!");
    end_time = millis();
    new_event.duration = end_time - start_time;
    SDcard_write(new_event);
    // go_to_sleep(); need another interuppt from wifi
  }
  return;
}

/*-----------------------------test-----------------------------*/
// Interrupt handler; measure motion sensor duration;
void test_motion_sensor() {
  test_WiFi_connection();
  if (digitalRead(MOTION_SENSOR_PIN)) {
    debug_print("Measuring!");
    start_time = millis();
  } else {
    end_time = millis();
    debug_print(String((end_time-start_time)/1000)); 
    // go_to_sleep();
  }
}

// Send message via serial to wifi
void test_WiFi_connection() {
  WiFi_serial.print("System triggered at:"); // display this message in UDP client
  WiFi_serial.println(millis());
}

// Wrapper of print
void debug_print(String str) {
  if(DEBUG_MODE) {
    Serial.println(str);
    Serial.flush(); // make sure all data in serial has been sent out
  }
}

/*--------------------------------command handlers--------------------------------*/
// Handle command from serial
void cmd_handler() {
  // TODO: changed to wifi serial when bidirection is enabled
  if(Serial.available()) {
    String cmd = Serial.readString();
    debug_print("Command received: " + cmd);
    if(cmd == "Send\n") {
      send_data();
    }
  }
}

// Send whole txt via serial
void send_data() {
  debug_print("Send request received!");
  SDcard_read(); // TODO: decouple SD read and send
  //TODO: send latest data
  return;
}

/*--------------------------------WiFi functions--------------------------------*/
// Get basic information of wifi
void WiFi_init() {
  WiFi_serial.listen();
  WiFi_serial.println("Init");
  WiFi_serial.flush();
  while(!WiFi_serial.available()) {}
  debug_print(WiFi_serial.readString());
}

/*--------------------------------SD card functions--------------------------------*/
// Get basic information of SD card
void SD_card_init() {
  debug_print("Initializing SD card...");
  if (!SD.begin(WiFi_CHIPSELECT)) {
    debug_print("Card failed, or not present");
    while (1);
  }
  Serial.println("card initialized.");
}

// Read from datalog.txt in SD card
void SDcard_read() {
  File data_log = SD.open("datalog.txt");
  if(data_log) {
    debug_print("Transmitting data");
    while(data_log.available()) {
      WiFi_serial.write(data_log.read()); // send via wifi
    }
    data_log.close();
  } else {
    debug_print("error opening file");
  }
}

// Write new data to datalog.txt
void SDcard_write(struct poop_event event_to_be_written) {
  File data_log = SD.open("datalog.txt", FILE_WRITE);
  if(data_log) {
    data_log.print("ID: ");
    data_log.print(event_to_be_written.cat_id);
    data_log.print("Time: ");
    data_log.println(event_to_be_written.duration);
    data_log.close();
  } else {
    debug_print("error opening file");
  }
  debug_print("Data Recorded!");
  return;
}

/*--------------------------------RFID functions--------------------------------*/
// Read RFID. State is based on the number received currently. Conditions checked in the end.
unsigned RFID_read() {
  debug_print("Start reading RFID!");
  uint8_t buffer[BUFFER_SIZE]; // used to store an incoming data frame 
  int buffer_index = 0;
  int count = 0;
  bool call_extract_tag = false;
  
  while(!call_extract_tag && ( count < RFID_READ_DELAY)) {
    count++;
    if (RFID_serial.available() > 0){
      
      int ssvalue = RFID_serial.read(); // read 
      if (ssvalue == -1) { // no data was read
        continue;
      }
  
      if (ssvalue == 2) { // RDM630/RDM6300 found a tag => tag incoming 
        buffer_index = 0;
      } else if (ssvalue == 3) { // tag has been fully transmitted       
        call_extract_tag = true; // extract tag at the end of the function call
      }
  
      if (buffer_index >= BUFFER_SIZE) { // checking for a buffer overflow (It's very unlikely that an buffer overflow comes up!)
        Serial.println("Error: Buffer overflow detected!");
        break;
      }
      
      buffer[buffer_index++] = ssvalue; // everything is alright => copy current value to buffer
  
      if (call_extract_tag == true) {
        if (buffer_index == BUFFER_SIZE) {
          unsigned tag = extract_tag(buffer);
          return tag;
        } else { // something is wrong... start again looking for preamble (value: 2)
          buffer_index = 0;
          call_extract_tag = false; // index not correct, looking again
          continue;
        }
      }    
    }     
  }
  
  return 0;
}

// Helper of RFID
unsigned extract_tag(uint8_t* buffer) {
    uint8_t msg_head = buffer[0];
    uint8_t *msg_data = buffer + 1; // 10 byte => data contains 2byte version + 8byte tag
    uint8_t *msg_data_version = msg_data;
    uint8_t *msg_data_tag = msg_data + 2;
    uint8_t *msg_checksum = buffer + 11; // 2 byte
    uint8_t msg_tail = buffer[13];

    // print message that was sent from RDM630/RDM6300
    Serial.println("--------");

    Serial.print("Message-Head: ");
    Serial.println(msg_head);

    Serial.println("Message-Data (HEX): ");
    for (int i = 0; i < DATA_VERSION_SIZE; ++i) {
      Serial.print(char(msg_data_version[i]));
    }
    Serial.println(" (version)");
    for (int i = 0; i < DATA_TAG_SIZE; ++i) {
      Serial.print(char(msg_data_tag[i]));
    }
    Serial.println(" (tag)");

    Serial.print("Message-Checksum (HEX): ");
    for (int i = 0; i < CHECKSUM_SIZE; ++i) {
      Serial.print(char(msg_checksum[i]));
    }
    Serial.println("");

    Serial.print("Message-Tail: ");
    Serial.println(msg_tail);

    Serial.println("--");

    long tag = hexstr_to_value(msg_data_tag, DATA_TAG_SIZE);
    Serial.print("Extracted Tag: ");
    Serial.println(tag);

    long checksum = 0;
    for (int i = 0; i < DATA_SIZE; i+= CHECKSUM_SIZE) {
      long val = hexstr_to_value(msg_data + i, CHECKSUM_SIZE);
      checksum ^= val;
    }
    Serial.print("Extracted Checksum (HEX): ");
    Serial.print(checksum, HEX);
    if (checksum == hexstr_to_value(msg_checksum, CHECKSUM_SIZE)) { // compare calculated checksum to retrieved checksum
      Serial.print(" (OK)"); // calculated checksum corresponds to transmitted checksum!
    } else {
      Serial.print(" (NOT OK)"); // checksums do not match
    }

    Serial.println("");
    Serial.println("--------");

    return tag;
}

// Helper of RFID
long hexstr_to_value(char *str, unsigned int length) { // converts a hexadecimal value (encoded as ASCII string) to a numeric value
  char* copy = malloc((sizeof(char) * length) + 1); 
  memcpy(copy, str, sizeof(char) * length);
  copy[length] = '\0'; 
  // the variable "copy" is a copy of the parameter "str". "copy" has an additional '\0' element to make sure that "str" is null-terminated.
  long value = strtol(copy, NULL, 16);  // strtol converts a null-terminated string to a long value
  free(copy); // clean up 
  return value;
}

/*--------------------------------Low power--------------------------------*/
void go_to_sleep() {
  debug_print("Going to sleep!");
  LowPower.powerStandby(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 
}
