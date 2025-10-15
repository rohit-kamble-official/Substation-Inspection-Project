#include "esp_camera.h"
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <vector>
#include <ESP32Servo.h>

#define PAN_PIN 14
#define TILT_PIN 15
#define LIGHT_PIN 4

Servo panServo;
Servo tiltServo;

struct MOTOR_PINS {
  int pinEn;
  int pinIN1;
  int pinIN2;
};

std::vector<MOTOR_PINS> motorPins = {
  {2, 12, 13}, // RIGHT_MOTOR Pins
  {15, 1, 3}   // LEFT_MOTOR Pins
};

#define UP 1
#define DOWN 2
#define LEFT 3
#define RIGHT 4
#define STOP 0

#define RIGHT_MOTOR 0
#define LEFT_MOTOR 1

#define FORWARD 1
#define BACKWARD -1

const int PWMFreq = 1000;        // 1 KHz
const int PWMResolution = 8;     // 8-bit

// Camera pins
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

const char* ssid     = "MyWiFiCar";
const char* password = "12345678";

AsyncWebServer server(80);
AsyncWebSocket wsCamera("/Camera");
AsyncWebSocket wsCarInput("/CarInput");
uint32_t cameraClientId = 0;

// Correctly terminated raw string literal
const char* htmlHomePage PROGMEM = R"HTMLHOMEPAGE(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body { text-align:center; font-family:sans-serif; }
  .button { font-size:30px; background:black; color:white; padding:15px; border-radius:25%; margin:5px; }
  .slider { width:80%; }
</style>
</head>
<body>
<h2>ESP32-CAM Car Control</h2>
<img id="cameraImage" src="" width="400" height="250"><br>
<button class="button" ontouchstart='sendButtonInput("MoveCar","1")' ontouchend='sendButtonInput("MoveCar","0")'>UP</button><br>
<button class="button" ontouchstart='sendButtonInput("MoveCar","3")' ontouchend='sendButtonInput("MoveCar","0")'>LEFT</button>
<button class="button" ontouchstart='sendButtonInput("MoveCar","4")' ontouchend='sendButtonInput("MoveCar","0")'>RIGHT</button><br>
<button class="button" ontouchstart='sendButtonInput("MoveCar","2")' ontouchend='sendButtonInput("MoveCar","0")'>DOWN</button><br>
Speed: <input type="range" min="0" max="255" value="150" class="slider" id="Speed" oninput='sendButtonInput("Speed",value)'><br>
Light: <input type="range" min="0" max="255" value="0" class="slider" id="Light" oninput='sendButtonInput("Light",value)'><br>
Pan: <input type="range" min="0" max="180" value="90" class="slider" id="Pan" oninput='sendButtonInput("Pan",value)'><br>
Tilt: <input type="range" min="0" max="180" value="90" class="slider" id="Tilt" oninput='sendButtonInput("Tilt",value)'><br>

<script>
var wsCamera = new WebSocket("ws://"+window.location.hostname+"/Camera");
var wsCar = new WebSocket("ws://"+window.location.hostname+"/CarInput");
wsCamera.binaryType = 'blob';
wsCamera.onmessage = function(event){
  document.getElementById("cameraImage").src = URL.createObjectURL(event.data);
};
function sendButtonInput(key, value) {
  wsCar.send(key + "," + value);
}
</script>
</body>
</html>
)HTMLHOMEPAGE";

// Motor control
void rotateMotor(int motorNumber, int motorDirection) {
  if(motorDirection==FORWARD){
    digitalWrite(motorPins[motorNumber].pinIN1,HIGH);
    digitalWrite(motorPins[motorNumber].pinIN2,LOW);
  } else if(motorDirection==BACKWARD){
    digitalWrite(motorPins[motorNumber].pinIN1,LOW);
    digitalWrite(motorPins[motorNumber].pinIN2,HIGH);
  } else {
    digitalWrite(motorPins[motorNumber].pinIN1,LOW);
    digitalWrite(motorPins[motorNumber].pinIN2,LOW);
  }
}

void moveCar(int inputValue) {
  switch(inputValue) {
    case UP: rotateMotor(RIGHT_MOTOR,FORWARD); rotateMotor(LEFT_MOTOR,FORWARD); break;
    case DOWN: rotateMotor(RIGHT_MOTOR,BACKWARD); rotateMotor(LEFT_MOTOR,BACKWARD); break;
    case LEFT: rotateMotor(RIGHT_MOTOR,FORWARD); rotateMotor(LEFT_MOTOR,BACKWARD); break;
    case RIGHT: rotateMotor(RIGHT_MOTOR,BACKWARD); rotateMotor(LEFT_MOTOR,FORWARD); break;
    case STOP: rotateMotor(RIGHT_MOTOR,STOP); rotateMotor(LEFT_MOTOR,STOP); break;
    default: rotateMotor(RIGHT_MOTOR,STOP); rotateMotor(LEFT_MOTOR,STOP); break;
  }
}

// WebSocket events
void onCarInputWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
    AwsEventType type, void *arg, uint8_t *data, size_t len) {

  if(type==WS_EVT_CONNECT){
    Serial.println("Car client connected");
  } else if(type==WS_EVT_DISCONNECT){
    moveCar(STOP);
    ledcWrite(LIGHT_PIN, 0);
    panServo.write(90);
    tiltServo.write(90);
  } else if(type==WS_EVT_DATA){
    AwsFrameInfo *info = (AwsFrameInfo*)arg;
    if(info->final && info->opcode==WS_TEXT){
      std::string msg((char*)data,len);
      int comma = msg.find(',');
      String key = msg.substr(0,comma).c_str();
      int value = atoi(msg.substr(comma+1).c_str());
      if(key=="MoveCar") moveCar(value);
      else if(key=="Speed") ledcWrite(motorPins[0].pinEn, value);
      else if(key=="Light") ledcWrite(LIGHT_PIN, value);
      else if(key=="Pan") panServo.write(value);
      else if(key=="Tilt") tiltServo.write(value);
    }
  }
}

void onCameraWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
    AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if(type==WS_EVT_CONNECT) cameraClientId = client->id();
  else if(type==WS_EVT_DISCONNECT) cameraClientId = 0;
}

// Camera setup
void setupCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;

  if(esp_camera_init(&config)!=ESP_OK) {
    Serial.println("Camera init failed");
  }
}

// Pin setup
void setUpPinModes() {
  panServo.attach(PAN_PIN);
  tiltServo.attach(TILT_PIN);

  // Updated for ESP32 Core 3.x: ledcAttach(pin, freq, resolution)
  ledcAttach(motorPins[0].pinEn, PWMFreq, PWMResolution);
  ledcAttach(LIGHT_PIN, PWMFreq, PWMResolution);

  for(int i=0;i<motorPins.size();i++){
    pinMode(motorPins[i].pinIN1,OUTPUT);
    pinMode(motorPins[i].pinIN2,OUTPUT);
    pinMode(motorPins[i].pinEn,OUTPUT);
  }
  pinMode(LIGHT_PIN,OUTPUT);
  moveCar(STOP);
}

// Send camera image
void sendCameraPicture() {
  if(cameraClientId==0) return;
  camera_fb_t * fb = esp_camera_fb_get();
  if(!fb) return;
  wsCamera.binary(cameraClientId,fb->buf,fb->len);
  esp_camera_fb_return(fb);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  setUpPinModes();
  WiFi.softAP(ssid,password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP: "); Serial.println(IP);

  server.on("/",HTTP_GET,[](AsyncWebServerRequest *request){ request->send_P(200,"text/html",htmlHomePage); });
  wsCarInput.onEvent(onCarInputWebSocketEvent);
  server.addHandler(&wsCarInput);
  wsCamera.onEvent(onCameraWebSocketEvent);
  server.addHandler(&wsCamera);
  server.begin();

  setupCamera();
}

void loop() {
  wsCamera.cleanupClients();
  wsCarInput.cleanupClients();
  sendCameraPicture();
}
