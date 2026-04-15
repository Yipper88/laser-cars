/*
  ESP32 RC Car Controller with Laser
  Receives joystick commands via WebSocket: F, B, L, R, S, FIRE
*/

#include <Arduino.h>
#include <WiFi.h>
#include "LittleFS.h"
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "driver/pcnt.h"
#include "driver/timer.h"

// ===== MOTOR PINS =====================================================================================================================
#define MOTOR_LEFT_FWD    25
#define MOTOR_LEFT_BWD    27
#define MOTOR_RIGHT_FWD   32
#define MOTOR_RIGHT_BWD   33
#define MOTOR_LEFT_SPD    19
#define MOTOR_RIGHT_SPD   18

// ===== PLAYER ID ======
#define PLAYER_ID 0

// ===== LASER PIN =====================================================================================================================
#define LASER_PIN         16
#define LASER_READ_HIT    34
#define LASER_INTERRUPT   35
#define RMT_CHANNEL       0
static const uint32_t HDR_MARK_US  = 2000;
static const uint32_t HDR_GAP_US   = 1000;
static const uint32_t BIT_MARK_US  = 400;
static const uint32_t GAP0_US      = 400;
static const uint32_t GAP1_US      = 1200;
static const uint32_t STOP_MARK_US = 400;
static const uint32_t TAIL_OFF_US  = 500;

static const uint8_t RMT_CLK_DIV = 80;

// ===== WiFi AP SETTINGS ==============================================================================================================
#define AP_SSID           "RC_Car"
#define AP_PASS           "RCCar123"
IPAddress local_IP(192, 168, 4, 1);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

//===== RMT Carrier Configuration =====================================================================================================
void setupRMT_38kCarrier() {
  rmt_config_t c = {};
  c.rmt_mode = RMT_MODE_TX;
  c.channel = RMT_CHANNEL;
  c.gpio_num = LASER_PIN;
  c.mem_block_num = 1;
  c.clk_div = RMT_CLK_DIV;

  c.tx_config.loop_en = false;
  c.tx_config.carrier_en = true;
  c.tx_config.carrier_freq_hz = 38000;
  c.tx_config.carrier_duty_percent = 50;
  c.tx_config.carrier_level = RMT_CARRIER_LEVEL_HIGH;
  c.tx_config.idle_output_en = true;
  c.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;

    ESP_ERROR_CHECK(rmt_config(&c));
  ESP_ERROR_CHECK(rmt_driver_install(c.channel, 0, 0));
  rmt_register_tx_end_callback(rmtTxEndCallback, nullptr);

  txDone = true;
}

// ===== STATUS TRACKING ==============================================================================================================
unsigned long lastStatusUpdate = 0;
const unsigned long STATUS_UPDATE_INTERVAL = 2000;

// ===== Global Variables ==============================================================================================================
int motorSpeed = 200;
volatile bool isHit = false;
volatile unsigned long hitTime = 0;
const unsigned long coolDown = 3000;
  //Packet Decoding
volatile uint32_t last_time = 0;
volatile uint32_t duration = 0;
volatile bool header_received = false;
volatile int bit_count = 0;
volatile uint8_t received_data = 0;
volatile uint8_t final_value = 0;
volatile bool new_data_ready = false;
volatile bool txDone = true;

// ===== Interrupts =====================================================================================================================
void IRAM_ATTR hit_detected_isr();

// ===== FUNCTION DECLARATIONS ==========================================================================================================
void initLittleFS();
void initWebSocket();
void handleCommand(String cmd);
void stopMotors();
void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void setSpeed(int motorSpeed);
bool isDisabled();
void sendStatusUpdate();

// laser functions 
static void IRAM_ATTR rmtTxEndCallback(rmt_channel_t channel, void *arg) {
  txDone = true;
}
static inline uint16_t usToTicks(uint32_t us) {
  return (uint16_t)us;
}
static inline rmt_item32_t makeItem(bool level0, uint16_t dur0, bool level1, uint16_t dur1) {
  rmt_item32_t it;
  it.level0 = level0;
  it.duration0 = dur0;
  it.level1 = level1;
  it.duration1 = dur1;
  return it;
}

// laser 2-bit packet set up
void sendPacket2Bits(uint8_t bit1, uint8_t bit2) {
  const uint32_t gap1 = (bit1 ? GAP1_US : GAP0_US);
  const uint32_t gap2 = (bit2 ? GAP1_US : GAP0_US);

  rmt_item32_t items[4];

  items[0] = makeItem(true,  usToTicks(HDR_MARK_US),
                      false, usToTicks(HDR_GAP_US));

  items[1] = makeItem(true,  usToTicks(BIT_MARK_US),
                      false, usToTicks(gap1));

  items[2] = makeItem(true,  usToTicks(BIT_MARK_US),
                      false, usToTicks(gap2));

  items[3] = makeItem(true,  usToTicks(STOP_MARK_US),
                      false, usToTicks(TAIL_OFF_US));
  txDone = false;
  ESP_ERROR_CHECK(rmt_write_items(RMT_CHANNEL, items, 4, false));
} 

//Initializes pins and sets up WebSocket
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== ESP32 RC Car Starting ===");

  pinMode(MOTOR_LEFT_FWD, OUTPUT);
  pinMode(MOTOR_LEFT_BWD, OUTPUT);
  pinMode(MOTOR_RIGHT_FWD, OUTPUT);
  pinMode(MOTOR_RIGHT_BWD, OUTPUT);

  pinMode(LASER_INTERRUPT, INPUT_PULLUP);
  // 'CHANGE' triggers the ISR on both rising and falling edges
  attachInterrupt(digitalPinToInterrupt(LASER_INTERRUPT), hit_detected_isr, CHANGE);
  
  stopMotors();
  initLittleFS();

  WiFi.softAPConfig(local_IP, gateway, subnet);
  WiFi.softAP(AP_SSID, AP_PASS);

  Serial.println("WiFi AP Started");
  Serial.println(WiFi.softAPIP());

  initWebSocket();

  // Serve website correctly
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  server.begin();
  Serial.println("Web server started!");
  setupRMT_38kCarrier();
}


void loop() {
  ws.cleanupClients();  //Removes old disconnected connections
  unsigned long now = millis();

  // If we saw a header but 100ms has passed without finishing the packet
  if (header_received && (micros() - last_time > 100000)) {
      header_received = false;
      bit_count = 0;
      Serial.println("Packet Timeout - Resetting");
  }

  if (new_data_ready) {
    isHit = true;
    hitTime = millis();
    //Stop Motors Immediately
    stopMotors();
    String hitmsg = "HIT_BY_P" + String(final_value + 1);
    ws.textAll(hitmsg);
    Serial.println("Hit received from ID: " + String(final_value));
    new_data_ready = false;
  }
  //Cooldown Check and Re-enable
  if(isDisabled() && (now - hitTime >= coolDown)){
    //Cooldown expired, re-enable tank
    isHit = false;
    Serial.println("Tank re-enabled after cooldown.");
    ws.textAll("READY");
  }

  //Status Update
  if(now - lastStatusUpdate >= STATUS_UPDATE_INTERVAL){
    sendStatusUpdate();
    lastStatusUpdate = now;
  }
}

// ===== INTERRUPTS ==================================================================================================================
void IRAM_ATTR hit_detected_isr(){
  uint32_t now = micros();
  duration = now - last_time;
  last_time = now;

  bool pin_state = digitalRead(LASER_INTERRUPT); //High is space, Low is Mark

  if(pin_state == HIGH) { // We just finished a MARK (Laser Pulse)
        if (duration > 1800 && duration < 2200) { 
            header_received = true; 
            bit_count = 0;
            received_data = 0;
        }
    } else { // We just finished a SPACE (Gap)
        if (header_received) {
            if (duration > 300 && duration < 500) {
                // Logic 0 detected
                received_data <<= 1;
                bit_count++;
            } else if (duration > 1000 && duration < 1400) {
                // Logic 1 detected
                received_data = (received_data << 1) | 1;
                bit_count++;
            }
        }
    }
    if (bit_count == 2) { 
        // Success! Handle your 2-bit hit data here
        final_value = received_data; // Transfer the 2-bit number
        new_data_ready = true;        // Flag for the main loop
        header_received = false;
        bit_count = 0;
        received_data = 0;
    }
}

// ===== COMMAND HANDLER ==============================================================================================================
void handleCommand(String cmd) {
  if (isDisabled()) {
    Serial.println("Commands disabled - tank was hit!");
    return;
  }

  if (cmd == "F") moveForward();
  else if (cmd == "B") moveBackward();
  else if (cmd == "L") turnLeft();
  else if (cmd == "R") turnRight();
  else if (cmd == "S") stopMotors();
else if (cmd.startsWith("FIRE:")) {
  handleLaserCommand(cmd);
  }  else if (cmd.startsWith("SPEED:")){
    int newSpeed = cmd.substring(6).toInt();
    if(newSpeed >= 0 && newSpeed <= 255){
      motorSpeed = newSpeed;
      Serial.printf("Speed set to: %d\n", motorSpeed);
    }
    setSpeed(motorSpeed);
  }
  else Serial.println("Unknown command: " + cmd);
}

// ===== MOTOR CONTROL ===============================================================================================================
void stopMotors() {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  setSpeed(0);
  Serial.println("STOP");
}

void moveForward() {
  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  setSpeed(motorSpeed);
  Serial.println("FORWARD");
}

void moveBackward() {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, HIGH);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, HIGH);
  setSpeed(motorSpeed);
  Serial.println("BACKWARD");
}

void turnLeft() {
  digitalWrite(MOTOR_LEFT_FWD, LOW);
  digitalWrite(MOTOR_LEFT_BWD, HIGH);
  digitalWrite(MOTOR_RIGHT_FWD, HIGH);
  digitalWrite(MOTOR_RIGHT_BWD, LOW);
  setSpeed(motorSpeed);
  Serial.println("LEFT");
}

void turnRight() {
  digitalWrite(MOTOR_LEFT_FWD, HIGH);
  digitalWrite(MOTOR_LEFT_BWD, LOW);
  digitalWrite(MOTOR_RIGHT_FWD, LOW);
  digitalWrite(MOTOR_RIGHT_BWD, HIGH);
  setSpeed(motorSpeed);
  Serial.println("RIGHT");
}

void setSpeed(int speed){
    analogWrite(MOTOR_LEFT_SPD, speed);
    analogWrite(MOTOR_RIGHT_SPD, speed);
}

//===== LASER CONTROLS ==================================================================================================================
void handleLaserCommand(String msg) {
  msg.trim();

  if (!msg.startsWith("FIRE:")) {
    Serial.println("Unknown command");
    return;
  }

  if (!txDone) {
    Serial.println("Laser busy, ignoring command");
    return;
  }

  if (msg.length() != 7) {
    Serial.println("Invalid FIRE command length");
    return;
  }
  char b1 = msg.charAt(5);
  char b2 = msg.charAt(6);

  if ((b1 != '0' && b1 != '1') || (b2 != '0' && b2 != '1')) {
    Serial.println("Invalid FIRE bits");
    return;
  }
uint8_t bit1 = b1 - '0';
  uint8_t bit2 = b2 - '0';

  Serial.printf("Sending laser packet: %u%u\n", bit1, bit2);
  sendPacket2Bits(bit1, bit2);
}
//            alternate laser fire 
//             uint8_t code[2] = {1,0}; 
//              sendPacket2Bits(code[0],code[1]);  
//               Laser fire


//===== TANK HIT
bool isDisabled() {
  if (isHit && (millis() - hitTime < coolDown)) {
    return true;
  }
  return false;
}

//==== CONNECTION MAINTNENCE ============================================================================================================
void initWebSocket() {
  ws.onEvent([](AsyncWebSocket *server, AsyncWebSocketClient *client,
                AwsEventType type, void *arg, uint8_t *data, size_t len) {

    switch (type) {

      case WS_EVT_CONNECT:
        Serial.printf("Client #%u connected\n", client->id());
        stopMotors();
        sendStatusUpdate();
        break;

      case WS_EVT_DISCONNECT:
        Serial.printf("Client #%u disconnected\n", client->id());
        stopMotors();
        break;

      case WS_EVT_DATA: {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;

        if (info->final && info->index == 0 && info->len == len) {
          String cmd = "";
          cmd.reserve(len);

          for (size_t i = 0; i < len; i++) {
            cmd += (char)data[i];
          }

          cmd.trim();
          Serial.println("Received: " + cmd);
          handleCommand(cmd);
        }
        break;
      }

    } // switch end
  });

  server.addHandler(&ws);
}

void sendStatusUpdate() {
  String msg = "STATUS:{\"clients\":" + String(ws.count()) + "}";
  ws.textAll(msg);
}

//Checks if Website uploaded correctly
void initLittleFS() {
  if (!LittleFS.begin()) {
    Serial.println("LittleFS mount failed!");
  } else {
    Serial.println("LittleFS mounted successfully");
  }
}


