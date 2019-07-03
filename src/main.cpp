#include <Arduino.h>
#include "SSD1306.h"
#include "OLEDDisplay.h"
#include "math.h"

// board at https://www.amazon.com/gp/product/B07DKD79Y9
const int oled_address=0x3c;
const int pin_oled_sda = 4;
const int pin_oled_sdl = 15;
const int pin_oled_rst = 16;

const int pin_touch = T4;

SSD1306 display(oled_address, pin_oled_sda, pin_oled_sdl);

void setup() {
  Serial.begin(115200);
  pinMode(pin_touch, INPUT);
  pinMode(pin_oled_rst, OUTPUT);
  digitalWrite(pin_oled_rst, LOW);
  delay(10);
  digitalWrite(pin_oled_rst, HIGH);
  delay(100);

  display.init();
  display.println("Setup complete");
  display.display();


  // put your setup code here, to run once:
}

float get_tilt_angle() {
  // TODO: read tilt angle from gyroscope
  return 0;
}

void control_robot(float sp_x, float sp_v, float sp_a, float height) {
  // get the current timestamp
  // calculate dt from last timestamp
  // calculate the current wheel position velocity and acceleration
  // read current theta
  //float theta = get_tilt_angle();
  // calculate angular velocity
  //float top_position = wheel_x + height * sin(theta);  


  // last_angle = angle
  // last_wheel_osition = wheel_position
}

void loop() {
  static uint32_t loop_count = 0;
  static uint32_t touch_count = 0;

  display.clear();
  int touch_value = touchRead(pin_touch);
  if(touch_value < 147) {
    ++touch_count;
  }
  display.drawString(0,0,String(touch_value));
  display.drawString(0,10,String(touch_count));
  display.drawString(0,20,String(loop_count));
  display.display();
  ++loop_count;
  delay(100);
  // put your main code here, to run repeatedly:
}