#include <Arduino.h>
#include <FlexCAN_T4.h>
#include <math.h>
#include <QNEthernet.h>

using namespace qindesign::network;

EthernetUDP udp;

const uint16_t localPort = 8888;

IPAddress teensyIP(172, 31, 201, 50);
IPAddress teensyDNS(8, 8, 8, 8);
IPAddress gateway(172, 31, 0, 1);
IPAddress subnet(255, 255, 0, 0);

#pragma pack(push, 1)
struct EthernetCommand {
  float amp;
  float freq;
  float kp;
  float kd;
  uint32_t mode;
  uint32_t can_bus_no;
  uint32_t can_id;
};
#pragma pack(pop)

// MODES
#define MENU_MODE 0
#define MIT_MODE 5

uint32_t CANBUS_NO = 0;
uint32_t ACT_ID = 1; // (CAN ID)

EthernetCommand cmd = {
  0.0f,
  0.0f,
  0.0f,
  0.0f,
  MENU_MODE,
  CANBUS_NO,
  ACT_ID
};

// CAN
#define NUM_TX_MAILBOXES 32
#define NUM_RX_MAILBOXES 32

#define NUM_FRAMES 10000
#define FRAME_TIMEOUT_US 5000

// MIT motor protocol limits
#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -45.0f
#define V_MAX 45.0f
#define KP_MAX 500.0f
#define KD_MAX 5.0f
#define T_MIN -18.0f
#define T_MAX 18.0f

// (Not needed currently)
// Sine command parameters — set SINE_AMP = 0 for fixed hold-zero
#define SINE_AMP 0.6f // rad peak
#define SINE_HZ 1.0f
#define KP 40.0f
#define KD 0.5f

FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can1;
FlexCAN_T4<CAN3, RX_SIZE_256, TX_SIZE_16> Can2;

volatile uint32_t rx_time = 0;
volatile bool rx_flag = false;

uint32_t latencies[NUM_FRAMES];
uint32_t frame_count = 0;
bool test_done = false;
uint32_t t_start_us = 0;

bool mitEnabled = false;

void setupUDP() {
  if (udp.begin(localPort)) {
    Serial.printf("UDP server started on port %d.\n", localPort);
  } else {
    Serial.println("Failed to start UDP server.");
  }
}

void printNetworkConfig() {
  Serial.print("- Local IP: ");
  Serial.println(Ethernet.localIP());
  Serial.print("- Subnet Mask: ");
  Serial.println(Ethernet.subnetMask());
  Serial.print("- Gateway: ");
  Serial.println(Ethernet.gatewayIP());
  Serial.print("- DNS: ");
  Serial.println(Ethernet.dnsServerIP());
}

void rxCallback(const CAN_message_t &msg) {
  rx_time = micros();
  rx_flag = true;
}

uint16_t floatToUint(float x, float x_min, float x_max, uint8_t bits) {
  if (x < x_min) x = x_min;
  if (x > x_max) x = x_max;
  return (uint16_t)((x - x_min) / (x_max - x_min) * ((1u << bits) - 1));
}

void packCmd(float p, float v, float kp_val, float kd_val, float t_ff, CAN_message_t &msg) {
  uint16_t p_int = floatToUint(p, P_MIN, P_MAX, 16);
  uint16_t v_int = floatToUint(v, V_MIN, V_MAX,  12);
  uint16_t kp_int = floatToUint(kp_val, 0.0f, KP_MAX, 12);
  uint16_t kd_int = floatToUint(kd_val, 0.0f, KD_MAX, 12);
  uint16_t t_int  = floatToUint(t_ff, T_MIN, T_MAX,  12);

  msg.flags.extended = 1;
  msg.id = ACT_ID;
  msg.len = 8;
  msg.buf[0] = p_int >> 8;
  msg.buf[1] = p_int & 0xFF;
  msg.buf[2] = v_int >> 4;
  msg.buf[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
  msg.buf[4] = kp_int & 0xFF;
  msg.buf[5] = kd_int >> 4;
  msg.buf[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
  msg.buf[7] = t_int & 0xFF;
}

void setupCAN() {
  // Setup all CAN channels (3 on the Teensy)
  Can0.begin();
  Can0.setBaudRate(1000000);
  Can0.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
  Can0.setMBFilter(ACCEPT_ALL);
  Can0.distribute();
  Can0.enableMBInterrupts();
  Can0.setClock(CLK_60MHz);
  Can0.onReceive(rxCallback);
  Serial.println("CAN0 ready");

  Can1.begin();
  Can1.setBaudRate(1000000);
  Can1.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
  Can1.setMBFilter(ACCEPT_ALL);
  Can1.distribute();
  Can1.enableMBInterrupts();
  Can1.setClock(CLK_60MHz);
  Can1.onReceive(rxCallback);
  Serial.println("CAN1 ready");

  Can2.begin();
  Can2.setBaudRate(1000000);
  Can2.setMaxMB(NUM_TX_MAILBOXES + NUM_RX_MAILBOXES);
  Can2.setMBFilter(ACCEPT_ALL);
  Can2.distribute();
  Can2.enableMBInterrupts();
  Can2.setClock(CLK_60MHz);
  Can2.onReceive(rxCallback);
  Serial.println("CAN2 ready");

}

void canWrite(const CAN_message_t &msg) {
  if (CANBUS_NO == 0) Can0.write(msg);
  else if (CANBUS_NO == 1) Can1.write(msg);
  else if (CANBUS_NO == 2) Can2.write(msg);
}

void canEvents() {
  if (CANBUS_NO == 0) Can0.events();
  else if (CANBUS_NO == 1) Can1.events();
  else if (CANBUS_NO == 2) Can2.events();
}

void sendMode(uint8_t mode) {
  CAN_message_t msg;
  msg.flags.extended = 1;
  msg.id = ACT_ID;
  msg.len = 8;
  for (int i = 0; i < 7; i++) msg.buf[i] = 0xFF;
  msg.buf[7] = mode;
  canWrite(msg);
}

void enterMIT() {
  Serial.println("Entering MIT mode...");

  // Send neutral frames first
  /*
  for (int i = 0; i < 50; i++) {
    CAN_message_t msg;
    packCmd(0, 0, 0, 0, 0, msg);
    canWrite(msg);
    delay(2);
  }*/

  // Send mode command
  sendMode(MIT_MODE);

  mitEnabled = true;
  t_start_us = micros();
}

void setup() {
  Serial.begin(115200);

  Ethernet.begin(teensyIP, subnet, gateway, teensyDNS);

  delay(500);

  printNetworkConfig();

  setupUDP();
  
  setupCAN();

  canEvents();  // flush mode-enter response
}

void loop() {

  canEvents();

  int packetSize = udp.parsePacket();

  if (packetSize > 0) {

      Serial.print("Received packet, size = ");
      Serial.println(packetSize);

      udp.read((uint8_t *)&cmd, sizeof(cmd));

      if (CANBUS_NO != cmd.can_bus_no) {
        CANBUS_NO = cmd.can_bus_no;
        Serial.printf("Switched to CAN bus %d\n", CANBUS_NO);
      }

      if (ACT_ID != cmd.can_id) {
        ACT_ID = cmd.can_id;
        Serial.printf("New CAN ID: %d\n", ACT_ID);
      }

      Serial.printf(
        "amp=%.2f freq=%.2f kp=%.1f kd=%.2f mode=%d can_bus_no=%d can_id=%d\n",
        cmd.amp,
        cmd.freq,
        cmd.kp,
        cmd.kd,
        cmd.mode,
        cmd.can_bus_no,
        cmd.can_id
      );

      if (cmd.mode == MIT_MODE) {
        /*Serial.println("Entering MIT Mode");
        sendMode(MIT_MODE);
        mitEnabled = true;
        t_start_us = micros();*/
        enterMIT();
      }
      
      if (cmd.mode == MENU_MODE) {
        sendMode(MENU_MODE);
        mitEnabled = false;
      }
  }

  if (cmd.mode == MIT_MODE) {
    float elapsed = (micros() - t_start_us) * 1e-6f;
    float pos = cmd.amp * sinf(2.0f * M_PI * cmd.freq * elapsed);
    float vel = cmd.amp * 2.0f * M_PI * cmd.freq * cosf(2.0f * M_PI * cmd.freq * elapsed);

    CAN_message_t msg;

    packCmd(pos, vel, cmd.kp, cmd.kd, 0.0f, msg);

    canWrite(msg);
    delayMicroseconds(2000);

  }
  
  //if (test_done) return;

  /*
  if (frame_count < NUM_FRAMES) {
    float elapsed = (micros() - t_start_us) * 1e-6f;
    float pos = SINE_AMP * sinf(2.0f * M_PI * SINE_HZ * elapsed);
    float vel = SINE_AMP * 2.0f * M_PI * SINE_HZ * cosf(2.0f * M_PI * SINE_HZ * elapsed);

    CAN_message_t msg;
    if (SINE_AMP > 0.0f)
      packCmd(pos, vel, KP, KD, 0.0f, msg);
    else
      packCmd(0.0f, 0.0f, 0.0f, 0.1f, 0.0f, msg);

    rx_flag = false;
    uint32_t tx_time = micros();
    canWrite(msg);

    while (!rx_flag) {
      canEvents();
      if (micros() - tx_time > FRAME_TIMEOUT_US) break;
    }

    latencies[frame_count] = rx_flag ? (rx_time - tx_time) : UINT32_MAX;
    if (!rx_flag) Serial.printf("Timeout on frame %lu\n", frame_count);
    frame_count++;

    delayMicroseconds(2000);

  } else {
    sendMode(MENU_MODE);

    uint64_t sum = 0;
    uint32_t mn = UINT32_MAX;
    uint32_t mx = 0;
    uint32_t valid = 0;

    for (uint32_t i = 0; i < NUM_FRAMES; i++) {
      if (latencies[i] == UINT32_MAX) continue;
      sum += latencies[i];
      if (latencies[i] < mn) mn = latencies[i];
      if (latencies[i] > mx) mx = latencies[i];
      valid++;
    }

    Serial.println("\n=== CAN Latency Results ===");
    Serial.printf("Frames received : %lu / %d\n", valid, NUM_FRAMES);
    if (valid > 0) {
      Serial.printf("Average: %.2f us\n", (double)sum / valid);
      Serial.printf("Min: %lu us\n", mn);
      Serial.printf("Max: %lu us\n", mx);
    }

    test_done = true;
  }*/
}