#include <stdint.h>
#include <Arduino.h>
#include "USB.h"
#include "USBHID.h"
#include "driver/pcnt.h"

// USBSerial available for debug — only declare active when needed to avoid HID interference
// USBCDC USBSerial;  // uncomment for debug, comment out for normal use

/*
  ## VID/PID
    046d  Logitech, Inc.
    c29b  G27 Racing Wheel
*/

#define DEV_VID               (0x046d)
#define DEV_PID               (0xc29b)
#define DEV_PRODUCT_NAME      "G27_emu"
#define DEV_MANUFACTURER_NAME "MB"
#define DEV_REPORT_ID         (0)
#define DEV_REPORT_SIZE       (11)
#define DEV_FFB_REQUEST_SIZE  (7)

// --- Pin definitions ---
#define PIN_ENC_A     4   // MT6701 A
#define PIN_ENC_B     5   // MT6701 B
#define PIN_IBT2_RPWM 7   // IBT-2 forward PWM
#define PIN_IBT2_LPWM 8   // IBT-2 reverse PWM
#define PIN_THROTTLE  1   // 49E throttle (ADC1)
#define PIN_BRAKE     2   // 49E brake (ADC1)
#define PIN_BTN1      9   // Button 1 (active LOW)
#define PIN_BTN2      10  // Button 2 (active LOW)

// --- Encoder / wheel range ---
#define ENCODER_PPR       1024   // MT6701 ABZ configured at 1024 PPR
#define WHEEL_MAX_DEGREES 900
// Total counts for full 900deg lock-to-lock
#define ENCODER_COUNTS    ((ENCODER_PPR * WHEEL_MAX_DEGREES) / 360)

// --- Pedal ADC range (49E typical output in mV) ---
#define PEDAL_MV_MIN  400
#define PEDAL_MV_MAX  2000

// --- Motor PWM ---
#define MOTOR_PWM_FREQ 20000  // 20kHz
#define MOTOR_PWM_BITS 8
#define MOTOR_MAX_PWM  0xf0   // leave some headroom below 0xff

// --- HID report timing ---
#define DT_REPORT_MS 2

// --- FFB commands (Logitech Force Feedback Protocol V1.6) ---
enum class EnumFfbCmd {
  DOWNLOAD_FORCE        = 0x00,
  DOWNLOAD_AND_PLAY_FORCE = 0x01,
  PLAY_FORCE            = 0x02,
  STOP_FORCE            = 0x03,
  DEFAULT_SPRING_ON     = 0x04,
  DEFAULT_SPRING_OFF    = 0x05,
  REFRESH_FORCE         = 0x0c,
  SET_DEFAULT_SPRING    = 0x0e,
};

// --- FFB force types (Logitech Force Feedback Protocol V1.6) ---
enum class EnumForceType {
  CONSTANT              = 0x00,
  SPRING                = 0x01,
  DAMPER                = 0x02,
  AUTO_CNT_SPRING       = 0x03,
  SAWTOOTH_UP           = 0x04,
  SAWTOOTH_DN           = 0x05,
  TRAPEZOID             = 0x06,
  RECTANGLE             = 0x07,
  VARIABLE              = 0x08,
  RAMP                  = 0x09,
  SQUARE_WAVE           = 0x0a,
  HI_RES_SPRING         = 0x0b,
  HI_RES_DAMPER         = 0x0c,
  HI_RES_AUTO_CNT_SPRING = 0x0d,
  FRICTION              = 0x0e
};


// --- HID descriptor compatible with G27 ---
static const uint8_t hid_report_descriptor[] = {
  0x05, 0x01,        // Usage Page (Generic Desktop)
  0x09, 0x04,        // Usage (Joystick)
  0xA1, 0x01,        // Collection (Application)
  0x15, 0x00,        //   Logical minimum (0)
  0x25, 0x07,        //   Logical maximum (7)
  0x35, 0x00,        //   Physical minimum (0)
  0x46, 0x3B, 0x01,  //   Physical maximum (315)
  0x65, 0x14,        //   Unit (20)
  0x09, 0x39,        //   Usage (Hat switch)
  0x75, 0x04,        //   Report size (4)
  0x95, 0x01,        //   Report count (1)
  0x81, 0x42,        //   Item
  0x65, 0x00,        //   Unit (0)
  0x25, 0x01,        //   Logical maximum (1)
  0x45, 0x01,        //   Physical maximum (1)
  0x05, 0x09,        //   Usage Page (Button)
  0x19, 0x01,        //   Usage Minimum (Button 1)
  0x29, 0x16,        //   Usage Maximum (Button 22)
  0x75, 0x01,        //   Report size (1)
  0x95, 0x16,        //   Report count (22)
  0x81, 0x02,        //   Item
  0x26, 0xFF, 0x3F,  //   Logical maximum (16383)
  0x46, 0xFF, 0x3F,  //   Physical maximum (16383)
  0x75, 0x0E,        //   Report size (14)
  0x95, 0x01,        //   Report count (1)
  0x05, 0x01,        //   Usage Page (Generic Desktop)
  0x09, 0x30,        //   Usage (X)
  0x81, 0x02,        //   Item
  0x26, 0xFF, 0x00,  //   Logical maximum (255)
  0x46, 0xFF, 0x00,  //   Physical maximum (255)
  0x75, 0x08,        //   Report size (8)
  0x95, 0x03,        //   Report count (3)
  0x09, 0x32,        //   Usage (Z)
  0x09, 0x35,        //   Usage (Rz)
  0x09, 0x31,        //   Usage (Y)
  0x81, 0x02,        //   Item
  0x06, 0x00, 0xFF,  //   Usage Page (Vendor-defined)
  0x09, 0x01,        //   Usage (Vendor-defined {ff00:1))
  0x95, 0x02,        //   Report count (2)
  0x81, 0x02,        //   Item
  0x95, 0x01,        //   Report count (1)
  0x75, 0x01,        //   Report size (1)
  0x25, 0x01,        //   Logical maximum (1)
  0x45, 0x01,        //   Physical maximum (1)
  0x05, 0x09,        //   Usage Page (Button)
  0x09, 0x17,        //   Usage (Button 23)
  0x81, 0x02,        //   Item
  0x06, 0x00, 0xFF,  //   Usage Page (Vendor-defined)
  0x09, 0x01,        //   Usage (Vendor-defined {ff00:1))
  0x95, 0x07,        //   Report count (7)
  0x81, 0x02,        //   Item
  0x26, 0xFF, 0x00,  //   Logical maximum (255)
  0x46, 0xFF, 0x00,  //   Physical maximum (255)
  0x06, 0x00, 0xFF,  //   Usage Page (Vendor-defined)
  0x09, 0x02,        //   Usage (Vendor-defined {ff00:2))
  0x95, 0x07,        //   Report count (7)
  0x75, 0x08,        //   Report size (8)
  0x91, 0x02,        //   Item
  0x95, 0x90,        //   Report count (144)
  0x09, 0x03,        //   Usage (Vendor-defined {ff00:3))
  0xB1, 0x02,        //   Item
  0xC0,              // End Collection
};


// --- FFB force type union ---
union FfbForceType {
  uint8_t bytes[DEV_FFB_REQUEST_SIZE - 1];
  struct f00_constant_t {
    uint8_t type;
    uint8_t f0;
    uint8_t f1;
    uint8_t f2;
    uint8_t f3;
    uint8_t zero;
  } constant;
};


// --- FFB request from host ---
union FfbRequest {
  uint8_t bytes[DEV_FFB_REQUEST_SIZE];

  struct cmd00_download_force_t {
    uint8_t cmd;
    FfbForceType force_type;
  } download_force;

  struct cmd01_download_and_play_force_t {
    uint8_t cmd;
    FfbForceType force_type;
  } download_and_play_force;

  struct cmd02_play_force_t {
    uint8_t cmd;
    FfbForceType force_type;
  } play_force;

  struct cmd03_stop_force_t {
    uint8_t cmd;
    uint8_t bytes[DEV_FFB_REQUEST_SIZE - 1];
  } stop_force;

  struct cmd04_default_spring_on_t {
    uint8_t cmd;
    uint8_t bytes[DEV_FFB_REQUEST_SIZE - 1];
  } default_spring_on;

  struct cmd05_default_spring_off_t {
    uint8_t cmd;
    uint8_t bytes[DEV_FFB_REQUEST_SIZE - 1];
  } default_spring_off;

  struct cmd0c_refresh_force_t {
    uint8_t cmd;
    FfbForceType force_type;
  } refresh_force;

  struct cmd0e_set_default_spring_t {
    uint8_t cmd;
    uint8_t zero;
    uint8_t k1;
    uint8_t k2;
    uint8_t clip;
    uint8_t zeros[2];
  } set_default_spring;
};


// --- HID report sent to host ---
union WheelStatus {
  uint8_t bytes[11];

  struct Status_t {
    uint8_t buttons_0;
    uint8_t buttons_1;
    uint8_t buttons_2;
    uint8_t axis_wheel_lsb6_and_btns2;
    uint8_t axis_wheel_msb;
    uint8_t axis_throttle;
    uint8_t axis_brake;
    uint8_t axis_clutch;
    uint8_t vendor_specific[3];

    void set_axis_wheel_float(float v, bool center_zero) {
      if (center_zero) v = (v + 1) / 2.0f;
      if (v < 0.0f) v = 0.0f;
      if (v > 1.0f) v = 1.0f;
      v = v * 0x3fff;
      set_axis_wheel_14bit((uint16_t)v);
    }

    void set_axis_wheel_14bit(uint16_t v) {
      axis_wheel_lsb6_and_btns2 = (v & 0x003f) << 2;
      axis_wheel_msb = (v >> 6) & 0x00ff;
    }

    uint16_t get_axis_wheel_14bit() {
      uint16_t v = 0x0000;
      v = (axis_wheel_lsb6_and_btns2 >> 2) & 0x003f;
      v |= (((uint16_t)axis_wheel_msb) << 6) & 0x3fc0;
      return v;
    }
  } status;
};


// --- FFB force computation ---
class FfbController {
public:
  FfbController(uint16_t axis_wheel_center, uint16_t axis_wheel_range)
    : force_current(0x7f),
      ffb_forces_en{ 0, 0, 0, 0 },
      ffb_default_spring_on(false),
      ffb_default_spring_k1(0),
      ffb_default_spring_k2(0),
      ffb_default_spring_clip(0) {
    axis_wheel_cnt = axis_wheel_center;
    axis_wheel_min = axis_wheel_center - axis_wheel_range / 2;
    axis_wheel_max = axis_wheel_center + axis_wheel_range / 2;
  }

  void apply_force(const FfbRequest &req) {
    EnumFfbCmd f_cmd = (EnumFfbCmd)(req.bytes[0] & 0x0f);

    bool f0_en = req.bytes[0] & 0x10;
    bool f1_en = req.bytes[0] & 0x20;
    bool f2_en = req.bytes[0] & 0x40;
    bool f3_en = req.bytes[0] & 0x80;

    if (f_cmd == EnumFfbCmd::DOWNLOAD_FORCE) {
      if (f0_en) { ffb_forces[0] = req.download_force.force_type; ffb_forces_en[0] = false; }
      if (f1_en) { ffb_forces[1] = req.download_force.force_type; ffb_forces_en[1] = false; }
      if (f2_en) { ffb_forces[2] = req.download_force.force_type; ffb_forces_en[2] = false; }
      if (f3_en) { ffb_forces[3] = req.download_force.force_type; ffb_forces_en[3] = false; }
    } else if (f_cmd == EnumFfbCmd::DOWNLOAD_AND_PLAY_FORCE) {
      if (f0_en) { ffb_forces[0] = req.download_and_play_force.force_type; ffb_forces_en[0] = true; }
      if (f1_en) { ffb_forces[1] = req.download_and_play_force.force_type; ffb_forces_en[1] = true; }
      if (f2_en) { ffb_forces[2] = req.download_and_play_force.force_type; ffb_forces_en[2] = true; }
      if (f3_en) { ffb_forces[3] = req.download_and_play_force.force_type; ffb_forces_en[3] = true; }
    } else if (f_cmd == EnumFfbCmd::PLAY_FORCE) {
      if (f0_en) { ffb_forces_en[0] = true; }
      if (f1_en) { ffb_forces_en[1] = true; }
      if (f2_en) { ffb_forces_en[2] = true; }
      if (f3_en) { ffb_forces_en[3] = true; }
    } else if (f_cmd == EnumFfbCmd::STOP_FORCE) {
      if (f0_en) { ffb_forces_en[0] = false; }
      if (f1_en) { ffb_forces_en[1] = false; }
      if (f2_en) { ffb_forces_en[2] = false; }
      if (f3_en) { ffb_forces_en[3] = false; }
    } else if (f_cmd == EnumFfbCmd::DEFAULT_SPRING_ON) {
      ffb_default_spring_on = true;
    } else if (f_cmd == EnumFfbCmd::DEFAULT_SPRING_OFF) {
      ffb_default_spring_on = false;
    } else if (f_cmd == EnumFfbCmd::REFRESH_FORCE) {
      if (f0_en) { ffb_forces[0] = req.refresh_force.force_type; ffb_forces_en[0] = true; }
      if (f1_en) { ffb_forces[1] = req.refresh_force.force_type; ffb_forces_en[1] = true; }
      if (f2_en) { ffb_forces[2] = req.refresh_force.force_type; ffb_forces_en[2] = true; }
      if (f3_en) { ffb_forces[3] = req.refresh_force.force_type; ffb_forces_en[3] = true; }
    } else if (f_cmd == EnumFfbCmd::SET_DEFAULT_SPRING) {
      ffb_default_spring_k1   = req.set_default_spring.k1;
      ffb_default_spring_k2   = req.set_default_spring.k2;
      ffb_default_spring_clip = req.set_default_spring.clip;
    }
  }

  void update(uint16_t axis_wheel_value) {
    int16_t f = 0;

    if (ffb_default_spring_on) {
      int16_t k = (axis_wheel_value > axis_wheel_cnt) ? ffb_default_spring_k1 : -ffb_default_spring_k2;
      k *= ffb_default_spring_clip;
      k /= 7;
      f += k * abs(axis_wheel_value - axis_wheel_cnt) * 2 / (axis_wheel_max - axis_wheel_min);
    }

    for (uint8_t i = 0; i < 4; ++i) {
      if (!ffb_forces_en[i]) continue;
      if (i != 0) continue;

      FfbForceType f_entry = ffb_forces[i];
      EnumForceType f_type = (EnumForceType)f_entry.bytes[0];

      if (f_type == EnumForceType::CONSTANT) {
        // f0 holds the force magnitude for slot 0 (only slot processed)
        f += f_entry.constant.f0 - 0x7f;
      }
      // Other force types can be implemented here
    }

    f += 0x7f;
    if (f < 0x00) f = 0x00;
    if (f > 0xff) f = 0xff;

    force_current = f;
  }

  uint8_t get_force() { return force_current; }

  uint16_t axis_wheel_min;
  uint16_t axis_wheel_cnt;
  uint16_t axis_wheel_max;
  uint8_t  force_current;

  FfbRequest   ffb_request;
  FfbForceType ffb_forces[4];
  bool         ffb_forces_en[4];
  bool         ffb_default_spring_on;
  uint8_t      ffb_default_spring_k1;
  uint8_t      ffb_default_spring_k2;
  uint8_t      ffb_default_spring_clip;
};


// --- Encoder state (accumulated across int16 rollovers if needed) ---
volatile int32_t encoder_count = 0;

void setup_encoder() {
  pcnt_config_t cfg = {};
  cfg.pulse_gpio_num = PIN_ENC_A;
  cfg.ctrl_gpio_num  = PIN_ENC_B;
  cfg.channel        = PCNT_CHANNEL_0;
  cfg.unit           = PCNT_UNIT_0;
  cfg.pos_mode       = PCNT_COUNT_INC;
  cfg.neg_mode       = PCNT_COUNT_DEC;
  cfg.lctrl_mode     = PCNT_MODE_REVERSE;
  cfg.hctrl_mode     = PCNT_MODE_KEEP;
  cfg.counter_h_lim  = 32767;
  cfg.counter_l_lim  = -32768;
  pcnt_unit_config(&cfg);
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_counter_resume(PCNT_UNIT_0);
}

void update_encoder() {
  int16_t raw = 0;
  pcnt_get_counter_value(PCNT_UNIT_0, &raw);
  encoder_count = raw;  // fits in int16 for 900deg @ 1024ppr (max 2560 counts)
}


// --- Motor setup ---
void setup_motor() {
  ledcAttach(PIN_IBT2_RPWM, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  ledcAttach(PIN_IBT2_LPWM, MOTOR_PWM_FREQ, MOTOR_PWM_BITS);
  // Start with motor stopped
  ledcWrite(PIN_IBT2_RPWM, 0);
  ledcWrite(PIN_IBT2_LPWM, 0);
}


// --- Main wheel controller ---
class WheelController : public USBHIDDevice {
public:
  WheelController()
    : status{},
      ffb_controller(0x1fff, 0x3fff) {
    hid.addDevice(this, sizeof(hid_report_descriptor));
    // G27 expects this bit set in buttons_0
    status.status.buttons_0 = 0x08;
    // Clutch pedal not used, set to center
    status.status.axis_clutch = 0x7f;
  }

  WheelStatus  status;
  FfbController ffb_controller;
  USBHID        hid;

  void init() {
    hid.begin();
  }

  // --- Steering axis from encoder ---
  void update_axis_wheel() {
    update_encoder();
    // Map encoder count to 14-bit axis (0..0x3fff), center at 0
    int32_t v = encoder_count + (ENCODER_COUNTS / 2);
    v = map(v, 0, ENCODER_COUNTS, 0, 0x3fff);
    if (v < 0x0000) v = 0x0000;
    if (v > 0x3fff) v = 0x3fff;
    status.status.set_axis_wheel_14bit((uint16_t)v);
  }

  // --- Pedals from 49E Hall sensors ---
  void update_pedals() {
    uint32_t t = analogReadMilliVolts(PIN_THROTTLE);
    uint32_t b = analogReadMilliVolts(PIN_BRAKE);

    // Clamp and map to 0..255
    t = constrain(t, PEDAL_MV_MIN, PEDAL_MV_MAX);
    b = constrain(b, PEDAL_MV_MIN, PEDAL_MV_MAX);

    status.status.axis_throttle = map(t, PEDAL_MV_MIN, PEDAL_MV_MAX, 0, 255);
    status.status.axis_brake    = map(b, PEDAL_MV_MIN, PEDAL_MV_MAX, 0, 255);
  }

  // --- Buttons (active LOW with internal pullup) ---
  void update_buttons() {
    // Preserve the 0x08 bit that G27 expects
    uint8_t b = status.status.buttons_0 & 0x0f;

    if (!digitalRead(PIN_BTN1)) b |= (1 << 4);
    if (!digitalRead(PIN_BTN2)) b |= (1 << 5);

    status.status.buttons_0 = b;
  }

  // --- FFB motor output via IBT-2 ---
  void update_ffb() {
    const int32_t half = ENCODER_COUNTS / 2;

    // Hard software endstops — push back toward center at full force
    if (encoder_count <= -half) {
      ledcWrite(PIN_IBT2_RPWM, MOTOR_MAX_PWM);
      ledcWrite(PIN_IBT2_LPWM, 0);
      return;
    }
    if (encoder_count >= half) {
      ledcWrite(PIN_IBT2_RPWM, 0);
      ledcWrite(PIN_IBT2_LPWM, MOTOR_MAX_PWM);
      return;
    }

    ffb_controller.update(status.status.get_axis_wheel_14bit());
    int16_t f = ffb_controller.get_force();

    bool dir_ccw = (f > 0x7f);
    f = abs(f - 0x7f) << 1;
    if (f > MOTOR_MAX_PWM) f = MOTOR_MAX_PWM;

    if (dir_ccw) {
      ledcWrite(PIN_IBT2_RPWM, 0);
      ledcWrite(PIN_IBT2_LPWM, f);
    } else {
      ledcWrite(PIN_IBT2_RPWM, f);
      ledcWrite(PIN_IBT2_LPWM, 0);
    }
  }

  void update_axes() {
    update_axis_wheel();
  }

  void sendState() {
    if (hid.ready()) {
      hid.SendReport(DEV_REPORT_ID, status.bytes, sizeof(status.bytes), 0);
    }
  }

  uint16_t _onGetDescriptor(uint8_t *buffer) override {
    memcpy(buffer, hid_report_descriptor, sizeof(hid_report_descriptor));
    return sizeof(hid_report_descriptor);
  }

  uint16_t _onGetFeature(uint8_t report_id, uint8_t *buffer, uint16_t len) override { return 0; }

  void _onSetFeature(uint8_t report_id, const uint8_t *buffer, uint16_t len) override {
    if (len != sizeof(FfbRequest)) return;
    ffb_controller.apply_force(*((const FfbRequest *)buffer));
  }

  void _onOutput(uint8_t report_id, const uint8_t *buffer, uint16_t len) override {}
};


WheelController whl;

TaskHandle_t TaskSendHidReportsHandle;

void TaskSendHidReports(void *parameter) {
  unsigned long time_last_report = millis();
  while (true) {
    unsigned long now = millis();
    if (now - time_last_report >= DT_REPORT_MS) {
      time_last_report = now;
      whl.sendState();
    }
    vTaskDelay(1);  // yield to other tasks, prevents watchdog trigger
  }
}

void setup() {
  setCpuFrequencyMhz(240);

  // Encoder
  setup_encoder();

  // Motor
  setup_motor();

  // Pedal ADC — use ADC1 only, 11dB for ~0..3.1V range
  analogSetPinAttenuation(PIN_THROTTLE, ADC_11db);
  analogSetPinAttenuation(PIN_BRAKE,    ADC_11db);
  analogReadResolution(12);

  // Buttons
  pinMode(PIN_BTN1, INPUT_PULLUP);
  pinMode(PIN_BTN2, INPUT_PULLUP);

  // Built-in LED
  pinMode(LED_BUILTIN, OUTPUT);

  // USB — disable "USB CDC on boot" in Arduino Tools for this to work cleanly
  USB.VID(DEV_VID);
  USB.PID(DEV_PID);
  USB.usbPower(500);
  USB.usbAttributes(TUSB_DESC_CONFIG_ATT_SELF_POWERED);
  USB.usbVersion(0x0200);
  USB.productName(DEV_PRODUCT_NAME);
  USB.manufacturerName(DEV_MANUFACTURER_NAME);
  USB.begin();

  // USBSerial only for debug — comment out if HID data is unstable
  // USBSerial.begin();

  whl.init();

  // HID report task on core 0, priority 1
  xTaskCreatePinnedToCore(TaskSendHidReports, "TaskSendHidReports", 10000, NULL, 1, &TaskSendHidReportsHandle, 0);
}

void loop() {
  whl.update_axes();    // MT6701 AB encoder → steering axis
  whl.update_pedals();  // 49E Hall sensors → throttle / brake
  whl.update_buttons(); // GPIO pullup buttons
  whl.update_ffb();     // FFB force → IBT-2 motor driver
}
