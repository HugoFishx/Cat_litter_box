#define MOTION_SENSOR_PIN 1

// global variables
unsigned long start_time;
unsigned long end_time;

void setup() {
  // init peripherals: wifi, RFID reader, motion sensor

  // set pin mode
  pinMode(MOTION_SENSOR_PIN, INPUT);
  // set interrupt
  attachInterrupt(digitalPinToInterrupt(MOTION_SENSOR_PIN), motion_sensor_handler, CHANGE);
}

void loop() {

}

void motion_sensor_handler() {
  if (digitalRead(MOTION_SENSOR_PIN)) { // check whether enter or leave
    // enter handler
    RFID_read();
    start_time = millis(); // can also record time in real word. may need internet
  } else {
    // leave handler
    end_time = millis();
    
  }
  return;
}

int RFID_read() {
  return 0;
}
