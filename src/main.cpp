#include <Arduino.h>

//#include "I2Cdev.h"


#include "SSD1306.h"
#include "OLEDDisplay.h"
#include "math.h"
#include "BluetoothSerial.h"
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


// board at https://www.amazon.com/gp/product/B07DKD79Y9
const int oled_address=0x3c;
const int pin_oled_sda = 4;
const int pin_oled_sdl = 15;
const int pin_oled_rst = 16;

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

SSD1306 display(oled_address, pin_oled_sda, pin_oled_sdl);

bool every_n_ms(unsigned long last_loop_ms, unsigned long loop_ms, unsigned long ms) {
  return (last_loop_ms % ms) + (loop_ms - last_loop_ms) >= ms;
}



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

const int wifi_port = 80;
//const int wifi_max_clients = 1;
AsyncWebServer server(wifi_port);

struct HttpRoute {
  String method;
  String path;
  std::function< void(WiFiClient & client) > execute;
};



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


class WifiTask {
public:
  LineReader line_reader;
  WiFiClient client;
  unsigned long connect_start_ms = 0;
  unsigned long last_execute_ms = 0;
  unsigned long last_client_activity_ms = 0;
  String ssid;
  String password;

  String method="";  // GET, PUT, ETC.
  String path="";    // URI
  String version=""; // HTTP Version


  bool enabled=false;
  bool trace = false;
  bool log_serial = true;

  enum {
    status_disabled,
    status_not_connected,
    status_connecting,
    status_awaiting_client,
    status_awaiting_command,
    status_awaiting_header
  } current_state = status_disabled;

  void set_enable(bool enable_wifi) {
    if(enable_wifi==this->enabled) return;
    enabled = enable_wifi;
    if(enabled) {
      current_state = status_not_connected;
    } else {
      WiFi.disconnect(true, true);
      current_state = status_disabled;
    }
  }

  void set_connection_info(String ssid, String password) {
    WiFi.disconnect();
    this->ssid = ssid;
    this->password = password;
    if(current_state != status_disabled) {
      current_state = status_not_connected;
    }

    Serial.print("connection info set to ssid: ");
    Serial.print(ssid);
    Serial.print(" password: ");
    Serial.print(password);
    Serial.println();
  }


  void execute() {
    auto ms = millis();
    auto wifi_status = WiFi.status();


    switch (current_state) {
      case status_disabled:
        break;

      case status_not_connected:
        connect_start_ms = ms;
        WiFi.mode(WIFI_STA);
        esp_wifi_set_ps (WIFI_PS_NONE);
        WiFi.begin(ssid.c_str(), password.c_str());
        current_state = status_connecting;
        break;
      
      case status_connecting:
        if(wifi_status == WL_CONNECT_FAILED) {
          current_state = status_not_connected;
          if(trace) Serial.println("connection failed");
          current_state = status_not_connected;
          break;
        }
        if (wifi_status == WL_CONNECTED) {
          server.begin();
          current_state = status_awaiting_client;
          if(trace) Serial.print("wifi connected, web server started");
        } else {
          // if(every_n_ms(last_execute_ms, ms, 1000)) {
          //   Serial.print(wifi_status);
          // }
          if(ms - connect_start_ms > 5000) {
            if(trace) Serial.print("couldn't connect, trying again");
            WiFi.disconnect();
            current_state = status_not_connected;
            break;
          }
        }
        break;

      case status_awaiting_client:
        if (wifi_status != WL_CONNECTED) {
          if(trace) Serial.print("wifi connected, web server stopped");
          current_state = status_not_connected;
          server.end();
          break;
        }
        break;


      default:
        Serial.println("invalid sate in WifiTask");
    }
    last_execute_ms = ms;
  }

};

class PID {
public:
  float k_p=1;
  float k_i=1;
  float k_d=0;
  bool additive = false;

  float max_i_contribution = 1.0;
  float max_output = 1;
  float min_output = -1;
  float output = 0;

  float set_p = 0;
  float set_d = 0;

  float error_i = 0;

  float last_p = NAN;
  unsigned long last_us = 0;

public:

  PID(float k_p=1.0, float k_i=0, float k_d=0, bool additive=false) {
    set_gains(k_p, k_i, k_d, additive);
  }

  void set_gains(float k_p, float k_i, float k_d, bool additive) {
    this->k_p = k_p;
    this->k_i = k_i;
    this->k_d = k_d;
    this->additive = additive;
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

    float power = k_p * error_p + k_i * error_i + k_d * error_d;
    power = constrain(power, min_output, max_output);
    if(additive) {
      output += power;
    } else {
      output = power;
    }
    constrain(output, min_output, max_output);
    return output;
  }
};

// globals
Button button;
const uint32_t bluetooth_buffer_reserve = 500;
BluetoothSerial bluetooth;
Mpu6050Wrapper mpu;
WifiTask wifi_task;
QuadratureEncoder left_encoder(pin_left_a, pin_left_b);
QuadratureEncoder right_encoder(pin_right_a, pin_right_b);
Speedometer left_speedometer(0.2/982);
Speedometer right_speedometer(0.2/1036 );
PID left_wheel_pid;
PID right_wheel_pid;
//CmdCallback<100> commands;
Preferences preferences;
float goal_x_position;
enum ControlMode { manual, seeking_goal_x_position };
ControlMode control_mode = ControlMode::manual;

static PID pitch_pid(10 , 0, 0.3  );
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
void set_motor_power(int which, float power) {
  int channel_fwd = (which==0) ? left_cmd_fwd_pwm_channel : right_cmd_fwd_pwm_channel;
  int channel_rev = (which==0) ? left_cmd_rev_pwm_channel : right_cmd_rev_pwm_channel;
  int power_channel = (power>0) ? channel_fwd : channel_rev;
  int zero_channel = (power>0) ? channel_rev : channel_fwd;
  uint32_t duty = uint32_t(255*fabs(power));
  //Serial.println("set_motor_speed channel: "+String(power_channel) + "zero_channel: " + String(zero_channel) + " duty: " + String(duty));
  ledcWrite(power_channel, duty);
  ledcWrite(zero_channel, 0);
}

void set_motor_power(float left, float right) {
  set_motor_power(0, left);
  set_motor_power(1, right);
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



class CommandEnvironment {
public:
  CommandEnvironment(CmdParser & args, Stream & cout, Stream & cerr) 
  : args(args),cout(cout),cerr(cerr)
  {
  }
  CmdParser & args;
  Stream & cout;
  Stream & cerr;
  bool ok = true;
};

typedef void (*CommandCallback)(CommandEnvironment &env);


class Command {
public:
  Command() 
    : execute(nullptr)
  {}
  Command(const char * name, CommandCallback callback, char * helpstring = nullptr) {
    this->name = name;
    this->execute = callback;
    this->helpstring = helpstring;
  }
  String name;
  CommandCallback execute;
  String helpstring;
};


void cmd_set_wifi_config(CommandEnvironment & env) {
  char * ssid = env.args.getCmdParam(1);
  char * password = env.args.getCmdParam(2);
  preferences.begin("main",false);
  preferences.putString("ssid", ssid);
  preferences.putString("password", password);
  preferences.end();
  wifi_task.set_connection_info(ssid, password);
}

void cmd_set_motor_power(CommandEnvironment & env) {
  double left_power = atof(env.args.getCmdParam(1));
  double right_power = atof(env.args.getCmdParam(2));
  set_motor_power(0, left_power);
  set_motor_power(1, right_power);
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

void cmd_set_goal_distance(CommandEnvironment & env) {
  auto distance = atof(env.args.getCmdParam(1));
  goal_x_position = get_x_position() + distance;
  control_mode = ControlMode::seeking_goal_x_position;
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
  bool additive = (atoi(env.args.getCmdParam(4)) == 1);
  left_wheel_pid.set_gains(k_p, k_i, k_d, additive);
  right_wheel_pid.set_gains(k_p, k_i, k_d, additive);
}

void cmd_set_pitch_pid(CommandEnvironment & env) {
  float k_p = atof(env.args.getCmdParam(1));
  float k_i = atof(env.args.getCmdParam(2));
  float k_d = atof(env.args.getCmdParam(3));
  bool additive = (atoi(env.args.getCmdParam(4)) == 1);
  pitch_pid.set_gains(k_p, k_i, k_d, additive);
}


void cmd_set_velocity_pid(CommandEnvironment & env) {
  float k_p = atof(env.args.getCmdParam(1));
  float k_i = atof(env.args.getCmdParam(2));
  float k_d = atof(env.args.getCmdParam(3));
  bool additive = (atoi(env.args.getCmdParam(4)) == 1);
  velocity_pid.set_gains(k_p, k_i, k_d, additive);
}

 void go_to_goal_x(float pitch, float cart_x, float cart_velocity, float goal_x) {
  auto us = micros();
  auto pendulum_length = 0.3;
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
  velocity_pid.max_output = 0.03;
  velocity_pid.min_output = -0.03;
  velocity_pid.set(goal_velocity);
  auto goal_pitch = velocity_pid.next_output(us, cart_velocity);

  // use desired pitch to get wheel_velocity
  pitch_pid.set(goal_pitch-0.03);
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

std::vector<Command> commands;

void cmd_help(CommandEnvironment & env) {
  for(auto command : commands) {
    env.cout.print(command.name);
    env.cout.print(": ");
    env.cout.print(command.helpstring);
    env.cout.println();
  }
}
Command * get_command_by_name(const char * command_name) {
  for(auto & command : commands) {
    if(command.name == command_name) {
      return &command;
    }
  }
  return nullptr;
}


void setup() {
  preferences.begin("main", true);
  wifi_task.set_connection_info(preferences.getString("ssid"), preferences.getString("password"));
  wifi_task.set_enable(preferences.getBool("enable_wifi"));
  preferences.end();

  Serial.begin(921600);
  button.init(pin_touch);
  bluetooth.begin("bke");

  left_encoder.init();
  right_encoder.init();
  attachInterrupt(pin_left_a, left_a_change, CHANGE);
  attachInterrupt(pin_left_b, left_b_change, CHANGE);
  attachInterrupt(pin_right_a, right_a_change, CHANGE);
  attachInterrupt(pin_right_b, right_b_change, CHANGE);


  left_wheel_pid.set_gains(3.0, 15.0, 0, false);
  right_wheel_pid.set_gains(3.0, 15.0, 0, false);

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
  commands.emplace_back(Command{"help", &cmd_help, "displays list of available commands"});
  commands.emplace_back(Command{"set_wifi_config", &cmd_set_wifi_config});
  commands.emplace_back(Command{"set_motor_power", cmd_set_motor_power});
  commands.emplace_back(Command{"set_enable_wifi", cmd_set_enable_wifi});
  commands.emplace_back(Command{"set_peripheral_power", cmd_set_peripheral_power});
  commands.emplace_back(Command{"shutdown", cmd_shutdown});
  commands.emplace_back(Command{"page_down", cmd_page_down});
  commands.emplace_back(Command{"reset_odo", cmd_reset_odo});
  commands.emplace_back(Command{"set_wheel_speed", cmd_set_wheel_speed});
  commands.emplace_back(Command{"set_wheel_speed_pid", cmd_set_wheel_speed_pid});
  commands.emplace_back(Command{"set_goal_distance", cmd_set_goal_distance});
  commands.emplace_back(Command{"set_pitch_pid", cmd_set_pitch_pid});
  commands.emplace_back(Command{"set_velocity_pid", cmd_set_velocity_pid});
  
   for(auto x: {0,1}) {
     set_motor_power(x,0);
   }


  digitalWrite(pin_oled_rst, LOW);
  delay(10);
  digitalWrite(pin_oled_rst, HIGH);
  delay(100);


  // init display before mpu since it initializes shared i2c
  display.init();

  Serial.println("Initializing mpu...");
  //delay(500);
  //mpu.enable_interrupts(pin_mpu_interrupt);

  mpu.setup();
  SPIFFS.begin();
  server.on(
    "/command",
    HTTP_POST,
    // request
    [](AsyncWebServerRequest * request){
      Serial.println("got a request");
      Serial.println(request->contentLength());

      auto params=request->params();
      for(int i=0;i<params;i++){
        AsyncWebParameter* p = request->getParam(i);
        if(p->isPost() && p->name() == "body") {
          CmdParser parser;
          String body = p->value();

          parser.parseCmd((char *)body.c_str());
          auto command = get_command_by_name(parser.getCommand());
          if(command == nullptr) {
              request->send(200,"text/plain","command failed");
          } else {
              String output_string;
              StringStream output_stream(output_string);
              CommandEnvironment env(parser, output_stream, output_stream);
              command->execute(env);
              if(env.ok) {
                request->send(200,"text/plain", output_string);
              } else {
                request->send(200,"text/plain","command failed");
              }
          }
        }
        request->send(400,"text/plain","no post body sent");
      }
    }
    );
  

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
  static LineReader line_reader;
  static String last_bluetooth_line;
  static unsigned long last_loop_ms = 0;
  unsigned long loop_ms = millis();


  if(every_n_ms(loop_ms, last_loop_ms, 1)) {
    wifi_task.execute();

    // read the button
    button.execute();

    // check for a new line in bluetooth
    if(line_reader.get_line(bluetooth)) {
      CmdParser parser;
      parser.parseCmd((char *)line_reader.line.c_str());
      Command * command = get_command_by_name(parser.getCommand());
      if(command) {
        CommandEnvironment env(parser, bluetooth, bluetooth);
        command->execute(env);
      } else {
        bluetooth.print("ERROR: Command not found - ");
        bluetooth.println(parser.getCommand());
      }
      last_bluetooth_line = line_reader.line;
    }
  }

  if(every_n_ms(loop_ms, last_loop_ms, 500)) {
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
      go_to_goal_x(mpu.pitch, get_x_position(), get_velocity(), goal_x_position);
    } else {
      left_power = left_wheel_pid.next_output(us, left_speedometer.get_velocity(), left_speedometer.get_smooth_acceleration());
      right_power = right_wheel_pid.next_output(us, right_speedometer.get_velocity(), right_speedometer.get_smooth_acceleration());
      //Serial.println((String)"pl: "+ power_left + "pr: " + power_right);
      set_motor_power(left_power, right_power);
    }

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
      display.drawString(0,40, (String) "pitch: " + mpu.pitch);
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