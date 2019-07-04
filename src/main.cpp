#include <Arduino.h>
#include "SSD1306.h"
#include "OLEDDisplay.h"
#include "math.h"
#include "BluetoothSerial.h"

// board at https://www.amazon.com/gp/product/B07DKD79Y9
const int oled_address=0x3c;
const int pin_oled_sda = 4;
const int pin_oled_sdl = 15;
const int pin_oled_rst = 16;

const int pin_touch = T4;

SSD1306 display(oled_address, pin_oled_sda, pin_oled_sdl);


class Button {
public:
  int32_t pin = -1;
  uint32_t press_count = 0;
  uint32_t click_count = 0;
  unsigned long down_since_ms = 0;
  bool last_reading_down = false;
  bool is_touched = false;
  bool is_pressed = false;
  uint16_t touch_value = 0;

  const uint16_t min_press_ms = 100;

  void init(int32_t pin) {
    this->pin = pin;
    pinMode(pin, INPUT);
    press_count = 0;
    click_count = 0;
    down_since_ms = 0;
    is_touched = false;
    is_pressed = false;
  }

  void execute() {
    auto ms = millis();
    if(pin<0) {
      return;
    }
    touch_value = touchRead(pin);
    bool new_touched = touch_value < 140 ? true : false;
    if(new_touched) {
      if(!is_touched) {
        down_since_ms = ms;
        is_touched = true;
      } else {
        if(!is_pressed && ms - down_since_ms > min_press_ms) {
          is_pressed = true;
          ++press_count;
        }
      }
    } else {
      if(is_pressed) {
        is_pressed = false;
        ++click_count;
      }
      is_touched = false;
    }
  }
};


class LineReader {
  public:
  const uint32_t buffer_size = 500;
  String buffer;
  String line;
  BluetoothSerial bluetooth;
  bool line_available = true;

  LineReader() {
    buffer.reserve(buffer_size);
    line.reserve(buffer_size);
  }

  bool get_line(Stream & stream) {
    while(stream.available()) {
      char c = (char)stream.read();
      if(c=='\n' || c== '\r') {
        if(buffer.length() > 0) {
          line = buffer;
          buffer = "";
          return true;
        }
      } else {
        buffer.concat(c);
      }
    }
    return false;
  }
  
};

// globals
Button button;
const uint32_t bluetooth_buffer_reserve = 500;
BluetoothSerial bluetooth;

void setup() {
  Serial.begin(115200);
  button.init(pin_touch);
  bluetooth.begin("bke");
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
  static LineReader line_reader;
  static String last_bluetooth_line;
  button.execute();

  if(line_reader.get_line(bluetooth)) {
    last_bluetooth_line = line_reader.line;
  }

  display.clear();

  display.drawString(0, 0, String(button.touch_value));
  display.drawString(0, 10, String(button.press_count));
  display.drawString(0, 20, String(button.click_count));
  display.drawString(0, 30, String(loop_count));
  display.drawString(0, 40, last_bluetooth_line);
  display.display();
  ++loop_count;
  delay(10);
  // put your main code here, to run repeatedly:
}