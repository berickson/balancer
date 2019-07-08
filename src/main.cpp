#include <Arduino.h>

#include "I2Cdev.h"
#include "MPU6050.h"


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
MPU6050 mpu;

void setup() {


  Serial.begin(115200);
  button.init(pin_touch);
  bluetooth.begin("bke");
  pinMode(pin_oled_rst, OUTPUT);
  digitalWrite(pin_oled_rst, LOW);
  delay(10);
  digitalWrite(pin_oled_rst, HIGH);
  delay(100);

  // init display before mpu since it initializes shared i2c
  display.init();

  Serial.println("Initializing I2C devices...");
  mpu.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  mpu.reset();
  delayMicroseconds(50000);
  mpu.setSleepEnabled(false);
  mpu.setTempSensorEnabled(true);
  mpu.setFullScaleAccelRange(0);
    

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
bool every_n_ms(unsigned long last_loop_ms, unsigned long loop_ms, unsigned long ms) {
  return (last_loop_ms % ms) + (loop_ms - last_loop_ms) >= ms;
}

void loop() {
  static uint32_t loop_count = 0;
  static LineReader line_reader;
  static String last_bluetooth_line;
  static unsigned long last_loop_ms = 0;
  unsigned long loop_ms = millis();

  if(every_n_ms(loop_ms, last_loop_ms, 1)) {
    // read the button
    button.execute();

    // check for a new line in bluetooth
    if(line_reader.get_line(bluetooth)) {
      last_bluetooth_line = line_reader.line;
    }
  }

  if(every_n_ms(loop_ms, last_loop_ms, 100)) {
    display.clear();
    // display.drawString(0, 0, "touch_value: " + String(button.touch_value));
    // display.drawString(0, 10, "press_count: " + String(button.press_count));
    // display.drawString(0, 20, "click_count: "+String(button.click_count));
    // display.drawString(0, 30, "loop_count: " + String(loop_count/1000)+String("k "));
    // display.drawString(0, 40, last_bluetooth_line);


    int16_t t_raw, ax_raw, ay_raw, az_raw, gx, gy, gz;
    t_raw = mpu.getTemperature();
    float t = float(t_raw)/340.+36.53;
    mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx, &gy, &gz);
    float ax = ax_raw / 16200.0;
    float ay = ay_raw / 16700.0;
    float az = az_raw / 14800.0;

    display.drawString(0, 0, String("t:")+String(t));
    display.drawString(0, 10, String("accel[") +String(ax)+","+String(ay)+","+String(az)+String("]"));
    display.drawString(0, 20, String("gyro[") +String(gx)+","+String(gy)+","+String(gz)+String("]"));
    display.display();
  }
  ++loop_count;
  last_loop_ms = loop_ms;
  // put your main code here, to run repeatedly:
}