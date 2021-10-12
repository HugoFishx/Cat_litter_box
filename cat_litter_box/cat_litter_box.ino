
/*-----------------includes-----------------*/

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

void setup() {
  /*init peripherals: wifi, RFID reader, motion sensor, SD card*/

  /*set pin mode*/
  pinMode(MOTION_SENSOR_PIN, INPUT);
  /*set interrupt*/
  attachInterrupt(digitalPinToInterrupt(MOTION_SENSOR_PIN), test_motion_sensor, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(MOTION_SENSOR_PIN), motion_sensor_handler, CHANGE);
  /*misc*/
  Serial.begin(9600);
}

void loop() {
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
    SDcard_store();
  }
  return;
}

void test_motion_sensor() {
  if (digitalRead(MOTION_SENSOR_PIN)) {
    Serial.println("Start!");
    start_time = millis();
  } else {
    end_time = millis();
    Serial.println((end_time-start_time)/1000);
  }
}

int RFID_read() {
  return 0;
}

void SDcard_store() {
  return;
}
