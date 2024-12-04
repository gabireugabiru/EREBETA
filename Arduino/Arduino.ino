#include "stdint.h"
#include "AccelStepper.h"
#include "Adafruit_TCS34725.h"
#include "EEPROM.h"
#define BUTTON_1_PIN 8
#define BUTTON_2_PIN 9

#define EN_PIN  4
#define DIR_PIN 3
#define PUL_PIN 2

#define STOP_TIME_ADDRESS_HIGH 0
#define STOP_TIME_ADDRESS_LOW 1

#define RED_FLOOR_ADDRESS 2
#define GREEN_FLOOR_ADDRESS 3
#define BLUE_FLOOR_ADDRESS 4

Adafruit_TCS34725 sensTCS = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_300MS, TCS34725_GAIN_1X);

const uint16_t STEP = 800; 

const uint16_t FIRST_FLOOR  = 0  * STEP;
const uint16_t SECOND_FLOOR = 8  * STEP;
const uint16_t THIRD_FLOOR  = 16 * STEP;

uint16_t red_floor = FIRST_FLOOR;
uint16_t green_floor = SECOND_FLOOR;
uint16_t blue_floor = THIRD_FLOOR;

uint16_t stop_time = 5000;

AccelStepper stepper(1, PUL_PIN, DIR_PIN);
uint16_t r, g, b, c, TempCor, LUX;
uint32_t time_stopped = 0;

typedef enum {
  RED = 0,
  GREEN = 1,
  BLUE = 2,
} COLOR;

typedef enum {
  IDLE,
  MOVING,
  RETURNING,
} STATE;

STATE state = IDLE;
void enable_stepper() {
  digitalWrite(EN_PIN, LOW); 
}
void write_stop_time(uint16_t val) {
  uint8_t high = val >> 8;
  uint8_t low = val & 0x00ff;
  EEPROM.write(STOP_TIME_ADDRESS_HIGH, high);
  EEPROM.write(STOP_TIME_ADDRESS_LOW, low);

}
void update_stop_time() {
  uint8_t low = EEPROM.read(STOP_TIME_ADDRESS_LOW);
  uint8_t high = EEPROM.read(STOP_TIME_ADDRESS_HIGH);
  if (low == 0xff && high == 0xff) {
    stop_time = 5000;
  } else {
    stop_time = ((uint16_t)high << 8) | low;
    Serial.println(stop_time);
  }
}
uint16_t update_floor(uint8_t value) {
  switch (value) {
    case 0:
      return FIRST_FLOOR;
    case 1:
      return SECOND_FLOOR;
    case 2:
      return THIRD_FLOOR;
    default:
      return FIRST_FLOOR;
  }
}

void write_red_floor(uint8_t value) {
  EEPROM.write(RED_FLOOR_ADDRESS, value);
}
void write_green_floor(uint8_t value) {
  EEPROM.write(GREEN_FLOOR_ADDRESS, value);
}
void write_blue_floor(uint8_t value) {
  EEPROM.write(BLUE_FLOOR_ADDRESS, value);
}

void update_red_floor() { 
  uint8_t value = EEPROM.read(RED_FLOOR_ADDRESS);
  red_floor = update_floor(value);
  Serial.println(red_floor);
}
void update_green_floor() {
  uint8_t value = EEPROM.read(GREEN_FLOOR_ADDRESS);
  green_floor = update_floor(value);
  Serial.println(green_floor);
}
void update_blue_floor() {
  uint8_t value = EEPROM.read(BLUE_FLOOR_ADDRESS);
  blue_floor = update_floor(value);
  Serial.println(blue_floor);
}

void disable_stepper() {
  digitalWrite(EN_PIN, HIGH);
}

COLOR get_color() {
  sensTCS.getRawData(&r, &g, &b, &c);

  if (r > max(g, b) ) {
    return RED;
  }
  if (g > max(b, r)) {
    return GREEN;
  } 
  if (b > max(r, g)) {
    return BLUE;
  }
  return RED;
}

void setup() {
  Serial.begin(9600);

  if (!sensTCS.begin()) {
    Serial.println("Sensor com problema");
    while (1) {};
  } 
  update_stop_time();
  update_red_floor();
  update_green_floor();
  update_blue_floor();
  pinMode( BUTTON_1_PIN, INPUT_PULLUP);
  pinMode( BUTTON_2_PIN, INPUT_PULLUP);
  pinMode(EN_PIN, OUTPUT);
  stepper.setMaxSpeed(STEP);  
  stepper.setAcceleration(80);
  disable_stepper();
}

void IDLE_handler() {
  if (digitalRead(BUTTON_1_PIN) == LOW) {
    state = MOVING;
    stepper.moveTo(0);
  }
  if (digitalRead(BUTTON_2_PIN) == LOW) {
    COLOR selected_color = get_color();
    switch (selected_color) {
      case RED:
        Serial.println("RED");
        stepper.moveTo(red_floor);
        state = MOVING;
        break;
      case GREEN:
        Serial.println("GREEN");
        state = MOVING;
        stepper.moveTo(green_floor);
        break;
      case BLUE:
        Serial.println("BLUE");
        state = MOVING;
        stepper.moveTo(blue_floor);
        break;
    }
  }
}
void RETURNING_handler() {
  if (millis() > (time_stopped + stop_time)) {
    enable_stepper();
    stepper.run();
    if (stepper.distanceToGo() == 0) {
      state = IDLE;
    }
  }
}
void MOVING_handler() {
  stepper.run();
  if (stepper.distanceToGo() == 0) {
    stepper.moveTo(FIRST_FLOOR);
    disable_stepper();
    state = RETURNING;
    time_stopped = millis();
  }
}
void SERIAL_handler() {
  if (Serial.available() > 0) {
    uint16_t a = Serial.parseInt();
    switch (a) {
      case 0:
        Serial.println("Configração tempo de espera em segundos: ");
        while (Serial.available() < 1) {};
        write_stop_time(Serial.parseFloat() * 1000);
        update_stop_time();
        Serial.print("Tempo de espera agora é ");
        Serial.print(stop_time * 0.001);
        Serial.println(" segundos");
      break;
      case 1:
        Serial.println("Configuração do vermelho");
        while (Serial.available() < 1) {};
        write_red_floor(Serial.parseInt() - 1);
        update_red_floor();
      break;
      case 2:
        Serial.println("Configuração do verde");
        while (Serial.available() < 1) {};
        write_green_floor(Serial.parseInt() - 1);
        update_green_floor();
      break;
      case 3:
        Serial.println("Configuração do azul");
        while (Serial.available() < 1) {};
        write_blue_floor(Serial.parseInt() - 1);
        update_blue_floor();
      break;
    }
  }
}
void loop() {
  switch (state) {
    case IDLE:
      disable_stepper();
      IDLE_handler();   
      SERIAL_handler();
    break;
    case MOVING:
      enable_stepper();
      MOVING_handler();
    break;
    case RETURNING:
      RETURNING_handler(); 
    default:
    break;
  }
}