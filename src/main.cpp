#include "esp32-common.h"

//#include "I2Cdev.h"


#include "SSD1306.h"
#include "OLEDDisplay.h"
#include "math.h"
/*
with bluetooth
RAM:   [==        ]  18.6% (used 61044 bytes from 327680 bytes)
Flash: [=====     ]  46.4% (used 1460578 bytes from 3145728 bytes)

without
RAM:   [=         ]  12.7% (used 41632 bytes from 327680 bytes)
Flash: [===       ]  27.0% (used 850670 bytes from 3145728 bytes)
*/
#include "WiFi.h"
#include "functional"
#include <vector>
#include "Mpu6050Wrapper.h"
#include "speedometer.h"

#include "FunctionalInterrupt.h"
#include "QuadratureEncoder.h"

// https://github.com/pvizeli/CmdParser
#include "CmdBuffer.hpp"
#include "CmdParser.hpp"
#include "CmdCallback.hpp"

#include "Preferences.h"

#include "SPIFFS.h"
#include "esp_wifi.h"

// https://github.com/me-no-dev/ESPAsyncWebServer
#include "ESPAsyncWebServer.h"

#include "StringStream.h"


const int pin_left_a = 27;
const int pin_left_b = 14;
const int pin_right_b = 12;
const int pin_right_a = 13;

const int pin_right_cmd_rev = 23;
const int pin_right_cmd_fwd = 19;

const int pin_left_cmd_fwd = 22;
const int pin_left_cmd_rev = 21;

const int pin_built_in__led = 25;

const int pin_enable_ext_3v3 = 26;

const int pin_mpu_interrupt = 2;

const int pin_touch = 33; // T8;

const uint8_t left_cmd_fwd_pwm_channel = 0;
const uint8_t left_cmd_rev_pwm_channel = 1;
const uint8_t right_cmd_fwd_pwm_channel = 2;
const uint8_t right_cmd_rev_pwm_channel = 3;


class BlackBox {

public:
  const char * file_path = "/black_box.csv";
  uint32_t reset_ms = 0;
  struct Entry {
    uint32_t ms = 0;
    float pitch = NAN;
    float left_power = NAN;
    float left_position = NAN;
    float left_velocity = NAN;
    float left_smooth_velocity = NAN;
    static String csv_header() {
      return "ms,pitch,left_power,left_position,left_velocity,left_smooth_velocity";
    }
    String csv_line() {
      return String(ms)
        +","+String(pitch)
        +","+String(left_power)
        +","+String(left_position)
        +","+String(left_velocity)
        +","+String(left_smooth_velocity);
    }
  };

  const int capacity = 500;
  std::vector<BlackBox::Entry> entries;
  BlackBox() {
    entries.reserve(capacity);
  }
  void add_entry(const BlackBox::Entry & entry) {
    if(entries.size() < capacity) {
      entries.emplace_back(entry);
    }
  }
  void reset() {
    reset_ms = millis();
    entries.clear();
  }

  void write_to_disk() {
    if(SPIFFS.exists(file_path)) SPIFFS.remove(file_path);
    fs::File file = SPIFFS.open(file_path, "w");
    file.println(Entry::csv_header());
    for(auto & entry : entries) {
      file.println(entry.csv_line());
    }
  }
};


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

  const uint16_t min_press_ms = 20;

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
    bool new_touched = touch_value < 90 ? true : false;
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

/*
class LineReader {
  public:
  const uint32_t buffer_size = 500;
  String buffer;
  String line;
  bool line_available = true;

  LineReader() {
    buffer.reserve(buffer_size);
    line.reserve(buffer_size);
  }

  bool get_line(Stream & stream) {
    while(stream.available()) {
      char c = (char)stream.read();
      if(c=='\n') continue;
      if(c== '\r') {
        if(true){ //buffer.length() > 0) {
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
*/


String get_led_state_name() {
  return digitalRead(pin_built_in__led)?"ON":"OFF";
}

String get_variable_value(const String& var)
{
  if(var == "LED_STATE_NAME"){
    return get_led_state_name();
  }

  return String();
}

class PID {
public:
  float k_p=1;
  float k_i=1;
  float k_d=0;

  float max_i_contribution = 0.05;
  float max_output = 1;
  float min_output = -1;
  float output = 0;

  float set_p = 0;
  float set_d = 0;

  float error_i = 0;

  float last_p = NAN;
  unsigned long last_us = 0;

public:

  PID(float k_p=1.0, float k_i=0, float k_d=0) {
    set_gains(k_p, k_i, k_d);
  }

  void set_gains(float k_p, float k_i, float k_d) {
    this->k_p = k_p;
    this->k_i = k_i;
    this->k_d = k_d;
  }

  void reset() {
    last_p = NAN;
    last_us = 0;
    error_i = 0;
  }

  void set(float p, float d = 0) {
    set_p = p;
    set_d = d;
  }
  
  float next_output(unsigned long us, float p, float d=NAN) {
    float error_p = set_p - p;
    if(!isnan(last_p)) {
      float dt = (us-last_us)/1.0E6;
      if(dt >= 0) {
        error_i += error_p * dt;
        if (isnan(d)) {
          d = (p-last_p)/dt;
        }
      }
    }
    last_p = p;
    last_us = us;
    float error_d = isnan(d) ? 0 : set_d - d;

    if(fabs(error_i * k_i) > max_i_contribution) {
      error_i = max_i_contribution / k_i;
    }

    float output = k_p * error_p + k_i * error_i + k_d * error_d;
    output = constrain(output, min_output, max_output);
    return output;
  }
};

// globals
Button button;
Mpu6050Wrapper mpu;
QuadratureEncoder left_encoder(pin_left_a, pin_left_b);
QuadratureEncoder right_encoder(pin_right_a, pin_right_b);
Speedometer left_speedometer(0.2/982);
Speedometer right_speedometer(0.2/1036 );
PID left_wheel_pid;
PID right_wheel_pid;
float goal_x_position;
enum ControlMode { manual, seeking_goal_x_position, motor_power };
ControlMode control_mode = ControlMode::manual;
BlackBox black_box;

float g_left_power = 0;
float g_right_power = 0;

static PID pitch_pid(0.9 , 0, 0.03 );
static PID velocity_pid(1 , 0, 0.1  );


uint32_t loop_count = 0;

bool page_down_requested = false;


float get_x_position() {
  return (left_speedometer.get_meters_travelled() + right_speedometer.get_meters_travelled()) / 2.0;
}

float get_velocity() {
  return (left_speedometer.get_smooth_velocity() + right_speedometer.get_smooth_velocity()) / 2.0;
}




// which = 0 for left, 1 for right
void set_single_motor_power(int which, float power) {
  if(which == 0) {
    g_left_power = power;
  } else {
    g_right_power = power;
  }
  int channel_fwd = (which==0) ? left_cmd_fwd_pwm_channel : right_cmd_fwd_pwm_channel;
  int channel_rev = (which==0) ? left_cmd_rev_pwm_channel : right_cmd_rev_pwm_channel;
  int power_channel = (power>0) ? channel_fwd : channel_rev;
  int zero_channel = (power>0) ? channel_rev : channel_fwd;
  int min_duty = 16; // duty cycle when wheel almost begins to spin
  int max_duty = 255; // full speed
  int range = max_duty - min_duty;
  int n_power = fabs(power * range);
  int duty = (n_power == 0 ) ? 0 : n_power + min_duty; 
  // int percent_power = fabs(power*255)
  // uint32_t duty = uint32_t(255*fabs(power));
  //Serial.println("set_motor_speed channel: "+String(power_channel) + "zero_channel: " + String(zero_channel) + " duty: " + String(duty));
  ledcWrite(power_channel, duty);
  ledcWrite(zero_channel, 0);
}

void set_motor_power(float left, float right) {
  set_single_motor_power(0, left);
  set_single_motor_power(1, right);
}

void set_motor_power(float both) {
  set_motor_power(both, both);
}

void set_wheel_speed(float left_speed, float right_speed) {
  left_wheel_pid.reset();
  left_wheel_pid.set(left_speed);
  right_wheel_pid.reset();
  right_wheel_pid.set(right_speed);
}

void shutdown() {
  // turn off power to peripherals
  digitalWrite(pin_enable_ext_3v3,LOW);

  // turn off pullups for i2c since they waste power
  digitalWrite(15, LOW);
  digitalWrite(4, LOW);
  pinMode(15, INPUT_PULLDOWN);
  pinMode(4, INPUT_PULLDOWN);

  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);


  esp_deep_sleep_start();
}


void cmd_set_motor_power(CommandEnvironment & env) {
  double left_power = atof(env.args.getCmdParam(1));
  double right_power = env.args.getParamCount() == 1 ? left_power : atof(env.args.getCmdParam(2));
  set_single_motor_power(0, left_power);
  set_single_motor_power(1, right_power);
  control_mode = ControlMode::motor_power;
  env.cout.println("ok");
}

void cmd_reset_odo(CommandEnvironment & env) {
  right_encoder.reset();
  left_encoder.reset();
}

void cmd_set_enable_wifi(CommandEnvironment & env) {
  bool enable_wifi = String(env.args.getCmdParam(1))=="1";
  wifi_task.set_enable(enable_wifi);
  preferences.begin("main");
  preferences.putBool("enable_wifi", enable_wifi);
  preferences.end();
}

void set_goal_distance(float distance) {
  goal_x_position = get_x_position() + distance;
  control_mode = ControlMode::seeking_goal_x_position;
}

void cmd_go(CommandEnvironment & env) {
  float distance = 0;
  if(env.args.getParamCount() == 1) {
    distance = atof(env.args.getCmdParam(1));
  }
  goal_x_position = get_x_position() + distance;
  control_mode = ControlMode::seeking_goal_x_position;
  black_box.reset();
  env.cout.println("set target distance to " + String(distance));
}

void cmd_stop(CommandEnvironment & env) {
  set_motor_power(0);
  control_mode = ControlMode::motor_power;

}

void cmd_shutdown(CommandEnvironment & env) {
  shutdown();
}

void cmd_page_down(CommandEnvironment & env) {
  page_down_requested = true;
}

void cmd_set_peripheral_power(CommandEnvironment & env) {
  bool enable = (atoi(env.args.getCmdParam(1)) == 1);

  digitalWrite(pin_enable_ext_3v3, enable);
  if(enable) {
    pinMode(pin_enable_ext_3v3, OUTPUT);
    digitalWrite(pin_enable_ext_3v3, true);
  } else {
    pinMode(pin_enable_ext_3v3,INPUT);
  }
}

void cmd_set_wheel_speed(CommandEnvironment & env) {
  set_wheel_speed(atof(env.args.getCmdParam(1)), atof(env.args.getCmdParam(2)));
  control_mode = ControlMode::manual;
}

void cmd_set_wheel_speed_pid(CommandEnvironment & env) {
  float k_p = atof(env.args.getCmdParam(1));
  float k_i = atof(env.args.getCmdParam(2));
  float k_d = atof(env.args.getCmdParam(3));
  left_wheel_pid.set_gains(k_p, k_i, k_d);
  right_wheel_pid.set_gains(k_p, k_i, k_d);
}

void cmd_set_pitch_pid(CommandEnvironment & env) {
  float k_p = atof(env.args.getCmdParam(1));
  float k_i = atof(env.args.getCmdParam(2));
  float k_d = atof(env.args.getCmdParam(3));
  pitch_pid.set_gains(k_p, k_i, k_d);
}

void cmd_get_pitch_pid(CommandEnvironment & env) {
  env.cout.print("k_p: ");
  env.cout.print(pitch_pid.k_p);
  env.cout.print(", k_i: ");
  env.cout.print(pitch_pid.k_i);
  env.cout.print(", k_d: ");
  env.cout.print(pitch_pid.k_d);
  env.cout.println();
}


void cmd_set_velocity_pid(CommandEnvironment & env) {
  float k_p = atof(env.args.getCmdParam(1));
  float k_i = atof(env.args.getCmdParam(2));
  float k_d = atof(env.args.getCmdParam(3));
  velocity_pid.set_gains(k_p, k_i, k_d);
}

void cmd_get_black_box(CommandEnvironment & env) {
  env.cout.println(BlackBox::Entry::csv_header());
  for(auto & entry : black_box.entries) {
    env.cout.println(entry.csv_line());
  }
}

void go_to_goal_x(float pitch, float cart_x, float cart_velocity, float goal_x) {
  auto us = micros();
  auto pendulum_length = 0.1;
  static float last_pitch = 0;
  
  //  from the pendulums point of view
  auto pendulum_x = pitch * pendulum_length + cart_x;

  // use a position pid t find desired velocity
  static PID position_pid;
  position_pid.max_output = 0.1;
  position_pid.min_output = -0.1;
  position_pid.set(goal_x);
  auto goal_velocity = position_pid.next_output(us, pendulum_x);

  // use desired velocity to find desired pitch
  velocity_pid.max_output = 5*M_PI/180.;
  velocity_pid.min_output = -5*M_PI/180.;
  velocity_pid.set(goal_velocity);
  auto goal_pitch = velocity_pid.next_output(us, cart_velocity);

  // use desired pitch to get wheel_velocity
  //goal_pitch = 0;
  const auto zero_pitch = 0;// -3.5*M_PI/180;
  pitch_pid.set(goal_pitch+zero_pitch);
  if(pitch!=last_pitch) {
    float motor_power = -1.0* pitch_pid.next_output(us, pitch);
    // static auto last_ms = millis();
    //set_wheel_speed(motor_power, motor_power);
    // Serial.print(millis()-last_ms);
    // Serial.print(",");
    // Serial.print(pitch,4);
    // Serial.print(",");
    // Serial.print(motor_power);
    // Serial.println();
    // last_ms = millis();
    set_motor_power(motor_power, motor_power);

    last_pitch = pitch;
  }
}

// encoder interrupt handlers

void IRAM_ATTR left_a_change() {
  sensor_a_changed(left_encoder);
}

void IRAM_ATTR left_b_change() {
  sensor_b_changed(left_encoder);
}

void IRAM_ATTR right_a_change() {
  sensor_a_changed(right_encoder);
}

void IRAM_ATTR right_b_change() {
  sensor_b_changed(right_encoder);
}

String get_status_json() {
  return (String)"{\"loop_count\":"+String(loop_count)+"}";
}


void setup() {
  esp32_common_setup();
  button.init(pin_touch);
  
  left_encoder.init();
  right_encoder.init();
  attachInterrupt(pin_left_a, left_a_change, CHANGE);
  attachInterrupt(pin_left_b, left_b_change, CHANGE);
  attachInterrupt(pin_right_a, right_a_change, CHANGE);
  attachInterrupt(pin_right_b, right_b_change, CHANGE);


  left_wheel_pid.set_gains(3.0, 15.0, 0);
  right_wheel_pid.set_gains(3.0, 15.0, 0);

  pinMode(pin_oled_rst, OUTPUT);
  pinMode(pin_built_in__led, OUTPUT);

  pinMode(pin_left_cmd_fwd, OUTPUT);
  pinMode(pin_left_cmd_rev, OUTPUT);
  pinMode(pin_right_cmd_fwd, OUTPUT);
  pinMode(pin_right_cmd_rev, OUTPUT);
  pinMode(pin_enable_ext_3v3, OUTPUT);

  digitalWrite(pin_built_in__led, LOW);

  digitalWrite(pin_enable_ext_3v3, HIGH);

  digitalWrite(pin_left_cmd_fwd, LOW);
  digitalWrite(pin_left_cmd_rev, LOW);
  digitalWrite(pin_right_cmd_fwd, LOW);
  digitalWrite(pin_right_cmd_rev, LOW);

  const int pwm_frequency = 100;
  const int pwm_bits = 8;

  ledcSetup(left_cmd_fwd_pwm_channel, pwm_frequency, pwm_bits);
  ledcAttachPin(pin_left_cmd_fwd, left_cmd_fwd_pwm_channel);

  ledcSetup(left_cmd_rev_pwm_channel, pwm_frequency, pwm_bits);
  ledcAttachPin(pin_left_cmd_rev, left_cmd_rev_pwm_channel);

  ledcSetup(right_cmd_fwd_pwm_channel, pwm_frequency, pwm_bits);
  ledcAttachPin(pin_right_cmd_fwd, right_cmd_fwd_pwm_channel);

  ledcSetup(right_cmd_rev_pwm_channel, pwm_frequency, pwm_bits);
  ledcAttachPin(pin_right_cmd_rev, right_cmd_rev_pwm_channel);
  commands.reserve(50);
  commands.emplace_back(Command{"set_enable_wifi", cmd_set_enable_wifi});
  commands.emplace_back(Command{"set_motor_power", cmd_set_motor_power});
  commands.emplace_back(Command{"set_peripheral_power", cmd_set_peripheral_power});
  commands.emplace_back(Command{"shutdown", cmd_shutdown});
  commands.emplace_back(Command{"page_down", cmd_page_down});
  commands.emplace_back(Command{"reset_odo", cmd_reset_odo});
  commands.emplace_back(Command{"set_wheel_speed", cmd_set_wheel_speed});
  commands.emplace_back(Command{"set_wheel_speed_pid", cmd_set_wheel_speed_pid});
  commands.emplace_back(Command{"go", cmd_go});
  commands.emplace_back(Command{"stop", cmd_stop, "Emergency stop, sets motor power to zero"});
  commands.emplace_back(Command{"set_pitch_pid", cmd_set_pitch_pid});
  commands.emplace_back(Command{"get_pitch_pid", cmd_get_pitch_pid});
  commands.emplace_back(Command{"set_velocity_pid", cmd_set_velocity_pid});
  commands.emplace_back(Command{"get_black_box", cmd_get_black_box, "returns last recording as csv"});
  
  set_motor_power(0);
  control_mode = ControlMode::motor_power;

  Serial.println("Initializing mpu...");
  mpu.setup();

  // set up web server routes
  server.on("/led_on", HTTP_GET, [](AsyncWebServerRequest *request) {
    digitalWrite(pin_built_in__led, HIGH);  // GET /led_on turns the LED on
    request->send(SPIFFS, "/index.html", "text/html", false, get_variable_value);
  });

  server.on("/stats", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "application/json", get_status_json());
  });

  server.on("/led_off", HTTP_GET, [](AsyncWebServerRequest *request) {
    digitalWrite(pin_built_in__led, LOW);  // GET /led_ff turns the LED on
    request->send(SPIFFS, "/index.html", "text/html", false, get_variable_value);
  });
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", "text/html", false, get_variable_value);
  });  
  
  server.on("/black_box.csv", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/black_box.csv", "text/csv", false, get_variable_value);
  });

  server.on("/command_line.html", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/command_line.html", "text/html", false, get_variable_value);
  });

  server.on("/sleep", HTTP_PUT, [](AsyncWebServerRequest *request) {
    shutdown();
  });
  
  server.on("/favicon.ico", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/favicon.ico", "image/x-icon", false);
  });

  server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(SPIFFS, "/index.html", "text/html", false, get_variable_value);
  });



  Serial.println("Setup complete");
  // put your setup code here, to run once:
}

void loop() {
  static unsigned long last_loop_ms = 0;
  unsigned long loop_ms = millis();

  esp32_common_loop();

  if(every_n_ms(loop_ms, last_loop_ms, 1)) {
    // read the button
    button.execute();

  }

  if(false && every_n_ms(loop_ms, last_loop_ms, 500)) {
    Serial.print("x: ");
    Serial.print(get_x_position());
    Serial.print(" v: ");
    Serial.print(get_velocity());
    Serial.println();
  }

  if(every_n_ms(loop_ms, last_loop_ms, 10)) {
    auto us = micros();
    mpu.execute();
    left_speedometer.set_ticks(us, left_encoder.odometer_a+left_encoder.odometer_b);
    right_speedometer.set_ticks(us, right_encoder.odometer_a+right_encoder.odometer_b);
    
    static float left_power = 0;
    static float right_power = 0;

    if(control_mode == ControlMode::seeking_goal_x_position) {
      if(abs(mpu.pitch) < 40. * DEG_TO_RAD) {
        go_to_goal_x(mpu.pitch, get_x_position(), get_velocity(), goal_x_position);
        BlackBox::Entry entry;
        entry.ms = millis()-black_box.reset_ms;
        entry.pitch = mpu.pitch;
        entry.left_power = g_left_power;
        entry.left_position = left_speedometer.get_meters_travelled();
        entry.left_velocity = left_speedometer.get_velocity();
        entry.left_smooth_velocity = left_speedometer.get_smooth_velocity();
        black_box.add_entry(entry);
      } else {
        set_motor_power(0);
        control_mode = ControlMode::motor_power;
        black_box.write_to_disk();
      }
    } else if (control_mode == ControlMode::motor_power) {
      ;
    } else {
      left_power = left_wheel_pid.next_output(us, left_speedometer.get_velocity(), left_speedometer.get_smooth_acceleration());
      right_power = right_wheel_pid.next_output(us, right_speedometer.get_velocity(), right_speedometer.get_smooth_acceleration());
      //Serial.println((String)"pl: "+ power_left + "pr: " + power_right);
      set_motor_power(left_power, right_power);
    }
  }

  if(every_n_ms(loop_ms, last_loop_ms, 100)) {
    static int current_page = 0;
    const int max_page = 2;
    if(page_down_requested) {
      ++current_page;
      page_down_requested = false;
    }
    if (current_page > max_page) {
      current_page = 0;
    }
    int16_t t_raw, ax_raw, ay_raw, az_raw, gx, gy, gz;
    t_raw = mpu.mpu.getTemperature();
    float t = float(t_raw)/340.+36.53;
    mpu.mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx, &gy, &gz);
    float ax = ax_raw / 16200.0;
    float ay = ay_raw / 16700.0;
    float az = az_raw / 14800.0;

    display.clear();
    if (current_page == 0) {
      display.drawString(0, 0, "la:"+String(left_encoder.odometer_a)+ " lb:"+String(left_encoder.odometer_b));
      display.drawString(0, 10," ra:"+String(right_encoder.odometer_a)+" rb:"+String(right_encoder.odometer_b));
      display.drawString(0, 20, "lp: " + String(loop_count/1000)+String("k ") +" n:"+String(mpu.readingCount));
      display.drawString(0, 30, WiFi.localIP().toString());
      display.drawString(0,40, (String) "deg pitch: " + mpu.pitch * 180./M_PI);
      display.drawString(0, 50, (String) "vl:" + left_speedometer.get_velocity() + " vr: " + right_speedometer.get_velocity());
    display.display();

    }
    if (current_page == 1) {
      display.drawString(0, 0, String("t:")+String(t)+" n:"+String(mpu.readingCount));
      display.drawString(0, 10, String("accel[") +String(ax)+","+String(ay)+","+String(az)+String("]"));
      display.drawString(0, 20, String("gyro[") +String(gx)+","+String(gy)+","+String(gz)+String("]"));
      display.drawString(0, 30, "yaw:" + String(mpu.yaw_pitch_roll[0]) + "pitch" + mpu.yaw_pitch_roll[1] + "roll: " + mpu.yaw_pitch_roll[2]);
      display.drawString(0,40, (String) "pitch: " + mpu.pitch);
      //display.drawString(0, 30,(String)mpu. )

    }
    if (current_page == 2) {
      display.drawString(0, 0, "touch_value: " + String(button.touch_value));
      display.drawString(0, 10, "press_count: " + String(button.press_count));
      display.drawString(0, 20, "click_count: "+String(button.click_count));
    }
    display.display();
  }
  ++loop_count;
  last_loop_ms = loop_ms;
}