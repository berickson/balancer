#include <Arduino.h>

#include "I2Cdev.h"
#include "MPU6050.h"

#include "SSD1306.h"
#include "OLEDDisplay.h"
#include "math.h"
#include "BluetoothSerial.h"
#include "WiFi.h"
#include "secret.h"
#include "functional"
#include <vector>

#include "FunctionalInterrupt.h"
#include "QuadratureEncoder.h"

// board at https://www.amazon.com/gp/product/B07DKD79Y9
const int oled_address=0x3c;
const int pin_oled_sda = 4;
const int pin_oled_sdl = 15;
const int pin_oled_rst = 16;

const int pin_left_a = 36;
const int pin_left_b = 37;
const int pin_right_b = 38;
const int pin_right_a = 39;

const int pin_right_cmd_fwd = 27;
const int pin_right_cmd_rev = 14;

const int pin_left_cmd_fwd = 26;
const int pin_left_cmd_rev = 25;

const int pin_built_in__led = 25;

const int pin_touch = T4;

const uint8_t left_cmd_fwd_pwm_channel = 0;
const uint8_t left_cmd_rev_pwm_channel = 1;
const uint8_t right_cmd_fwd_pwm_channel = 2;
const uint8_t right_cmd_rev_pwm_channel = 3;

SSD1306 display(oled_address, pin_oled_sda, pin_oled_sdl);

void empty_callback(){}

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
const int wifi_max_clients = 1;
WiFiServer server(wifi_port, wifi_max_clients);

struct HttpRoute {
  String method;
  String path;
  std::function< void(WiFiClient & client) > execute;
};

void send_standard_header(WiFiClient & client) {
    client.println("HTTP/1.1 200 OK");
    client.println("Content-type:text/html");
    client.println();
}

void send_standard_response(WiFiClient & client) {
    send_standard_header(client);

    // the content of the HTTP response follows the header:
    client.println("<html><head><style>");
    client.println("* {font-size:40pt;}");
    client.println("</style></head><body>");
    client.println("<script>");
    client.println("function sleep() {");
    client.println("var request = new XMLHttpRequest();");
    client.println("request.open('PUT','/sleep');");
    client.println("request.send();");
    client.println("}");
    client.println("</script>");
    client.println("The LED is ");
    client.println(digitalRead(pin_built_in__led)?"ON":"OFF");
    client.println("<br><br>");
    client.println( "Click <a href='/led_on'>here</a> to turn the LED on.<br>" );
    client.println( "Click <a href='/led_off'>here</a> to turn the LED off.<br>" );
    client.println( "<br>" );
    client.println("<button onclick='sleep()'>sleep</button>");
    client.println("</body></html");

    // The HTTP response ends with another blank line:
    client.println();
}

HttpRoute route_turn_on_light {"GET","/led_on", [](WiFiClient & client)->void{
  digitalWrite(pin_built_in__led, HIGH);               // GET /H turns the LED on
  send_standard_response(client);
}};

HttpRoute route_turn_off_light {"GET","/led_off", [](WiFiClient & client)->void{
  digitalWrite(pin_built_in__led, LOW);               // GET /H turns the LED on
  send_standard_response(client);
}};

HttpRoute route_home {"GET","/", [](WiFiClient & client)->void{
  send_standard_response(client);
}};


HttpRoute route_sleep{"PUT","/sleep", [](WiFiClient & client)->void{
  touchAttachInterrupt(pin_touch, empty_callback, 120);
  esp_sleep_enable_touchpad_wakeup();
  client.println("Going to sleep now. You can use the touch pin to wake up. Good night!");
  client.println();
  client.stop();
  esp_deep_sleep_start();
}};


std::vector<HttpRoute> routes{route_home, route_turn_on_light, route_turn_off_light, route_sleep};

class WifiTask {
public:
  LineReader line_reader;
  WiFiClient client;
  unsigned long connect_start_ms = 0;
  unsigned long last_execute_ms = 0;
  unsigned long last_client_activity_ms = 0;

  String method="";  // GET, PUT, ETC.
  String path="";    // URI
  String version=""; // HTTP Version


  bool trace = false;
  bool log_serial = true;

  enum {
    status_not_connected,
    status_connecting,
    status_awaiting_client,
    status_awaiting_command,
    status_awaiting_header
  } current_state = status_not_connected;


  void execute() {
    auto ms = millis();
    auto wifi_status = WiFi.status();


    switch (current_state) {
      case status_not_connected:
        connect_start_ms = ms;
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
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
          if(trace) Serial.print("connected, waiting for client");
        } else {
          if(every_n_ms(last_execute_ms, ms, 1000)) {
            Serial.print(wifi_status);
          }
          if(ms - connect_start_ms > 5000) {
            if(trace) Serial.print("coudln't connect, trying again");
            WiFi.disconnect();
            current_state = status_not_connected;
            break;
          }
        }
        break;

      case status_awaiting_client:
        if (wifi_status != WL_CONNECTED) {
          current_state = status_not_connected;
          break;
        }
         client = server.available();
         if(client) {
           last_client_activity_ms = ms;
           current_state = status_awaiting_command;
           if(trace) Serial.println("client connected, awaiting command");
           break;
         } else {
          if(trace && every_n_ms(last_execute_ms, ms, 1000)) {
            if(trace) Serial.print(".");
          }
        }
        break;

      // the first line is a command, something like "GET / HTTP/1.1" or "GET /favicon.ico HTTP/1.1"
      case status_awaiting_command:
        if(wifi_status != WL_CONNECTED) {
          current_state = status_not_connected;
          break;
        }
        if(!client) {
          if(trace) Serial.println("Client disconnected");
          client.stop();
          current_state = status_awaiting_client;
          break;
        }

        while(client.available()) {
          last_client_activity_ms = ms;
          if(line_reader.get_line(client)) {

            // parse the command TODO: add error checking
            String & l = line_reader.line;
            int first_space = l.indexOf(" ");
            int last_space = l.lastIndexOf(" ");
            method = l.substring(0, first_space);
            path = l.substring(first_space+1, last_space);
            version = l.substring(last_space+1);

            Serial.print("method: ");
            Serial.println(method);
            Serial.print("path: ");
            Serial.println(path);
            Serial.print("version: ");
            Serial.println(version);

   

            if(log_serial) Serial.println(line_reader.line);
            if(trace) Serial.println("reading header");
            current_state = status_awaiting_header;
            break;
            }
        }
        break;

      case status_awaiting_header:
        if(wifi_status != WL_CONNECTED) {
          current_state = status_not_connected;
          break;
        }
        if(!client) {
          if(trace) Serial.println("Client disconnected");
          client.stop();
          current_state = status_awaiting_client;
          break;
        }
        while(client.available()) {
          if(line_reader.get_line(client)) {
            if(log_serial) Serial.println(line_reader.line);

            // a blank line means that the header is done
            if(line_reader.line.length()==0) {
              if(trace) Serial.println("blank line found, sending response");

              // send response from routes
              bool found = false;
              for(auto & route : routes) {
                if(route.method == this->method && route.path == this->path) {
                  found = true;
                  route.execute(client);
                  break;
                }
              }
              if(!found) {
                // send 404
                client.println("HTTP/1.1 404 Not Found");
                client.println("Content-type:text/html");
                client.println();
                client.println("<html><head></head><body>route not found, 404<br></body></html>");
                client.println();
              }


              client.stop();
              current_state = status_awaiting_client;
              break;
            }
            else {
              if(trace) Serial.println("waiting for blank line");
            }
          }
        }
        break;


      default:
        Serial.println("invalid sate in WifiTask");
    }
    last_execute_ms = ms;
  }

};

// globals
Button button;
const uint32_t bluetooth_buffer_reserve = 500;
BluetoothSerial bluetooth;
MPU6050 mpu;
WifiTask wifi_task;
QuadratureEncoder left_encoder(pin_left_a, pin_left_b);
QuadratureEncoder right_encoder(pin_right_a, pin_right_b);



// which = 0 for left, 1 for right
void set_motor_power(int which, float power) {
  int channel_fwd = (which==0) ? left_cmd_fwd_pwm_channel : right_cmd_fwd_pwm_channel;
  int channel_rev = (which==0) ? left_cmd_rev_pwm_channel : right_cmd_rev_pwm_channel;
  int power_channel = (power>0) ? channel_fwd : channel_rev;
  int zero_channel = (power>0) ? channel_rev : channel_fwd;
  uint32_t duty = uint32_t(255*fabs(power));
  Serial.println("set_motor_speed channel: "+String(channel_fwd) + "zero_channel: " + String(zero_channel) + " duty: " + String(duty));
  ledcWrite(power_channel, duty);
  digitalWrite(zero_channel, LOW);
}


void setup() {
  Serial.begin(921600);
  button.init(pin_touch);
  bluetooth.begin("bke");

  left_encoder.init();
  right_encoder.init();
  
  pinMode(pin_oled_rst, OUTPUT);
  pinMode(pin_built_in__led, OUTPUT);



  pinMode(pin_left_cmd_fwd, OUTPUT);
  pinMode(pin_left_cmd_rev, OUTPUT);
  pinMode(pin_right_cmd_fwd, OUTPUT);
  pinMode(pin_right_cmd_rev, OUTPUT);

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
  mpu.initialize();

  // verify connection
  Serial.println("Testing mpu connection...");
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

void loop() {
  static uint32_t loop_count = 0;
  static LineReader line_reader;
  static String last_bluetooth_line;
  static unsigned long last_loop_ms = 0;
  unsigned long loop_ms = millis();

  wifi_task.execute();

  if(every_n_ms(loop_ms, last_loop_ms, 1)) {
  
    // read the button
    button.execute();

    // check for a new line in bluetooth
    if(line_reader.get_line(bluetooth)) {
      last_bluetooth_line = line_reader.line;
    }
  }

  if(every_n_ms(loop_ms, last_loop_ms, 10)) {
    int16_t t_raw, ax_raw, ay_raw, az_raw, gx, gy, gz;
    t_raw = mpu.getTemperature();
    float t = float(t_raw)/340.+36.53;
    mpu.getMotion6(&ax_raw, &ay_raw, &az_raw, &gx, &gy, &gz);
    float ax = ax_raw / 16200.0;
    float ay = ay_raw / 16700.0;
    float az = az_raw / 14800.0;


    display.clear();

    display.drawString(0, 0, "la:"+String(left_encoder.odometer_a)+ " lb:"+String(left_encoder.odometer_b)+" ra:"+String(right_encoder.odometer_a)+" rb:"+String(right_encoder.odometer_b));


    
    //display.drawString(0, 0, String("t:")+String(t));
    //display.drawString(0, 10, String("accel[") +String(ax)+","+String(ay)+","+String(az)+String("]"));
    //display.drawString(0, 20, String("gyro[") +String(gx)+","+String(gy)+","+String(gz)+String("]"));
    //display.drawString(0, 0, "touch_value: " + String(button.touch_value));
    //display.drawString(0, 10, "press_count: " + String(button.press_count));
    //display.drawString(0, 20, "click_count: "+String(button.click_count));
    display.drawString(0, 30, "loop_count: " + String(loop_count/1000)+String("k "));
    display.drawString(0, 40, WiFi.localIP().toString());
    display.drawString(0, 50, last_bluetooth_line);
    display.display();
  }
  ++loop_count;
  last_loop_ms = loop_ms;
  // put your main code here, to run repeatedly:
}