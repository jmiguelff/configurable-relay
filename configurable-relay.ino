/*
 * configurable-relay
 *
 * Purpose:
 *   Standalone configurable relay controller for cyclic operation.
 *   The relay is switched ON for a configurable duration (Ton) and OFF
 *   for a configurable duration (Toff), repeating continuously.
 *
 * Target platform:
 *   Arduino Nano Every (ATmega4809)
 *
 * Key features:
 *   - Persistent configuration stored in EEPROM (flash emulation)
 *   - Ton / Toff values stored in seconds
 *   - Robust startup validation using versioning and checksum
 *   - Non-blocking state machine based on millis()
 *   - Designed for future extension with LCD + rotary encoder UI
 *
 * Safety notes:
 *   - This firmware only controls the relay logic.
 *   - When switching mains voltage, use a properly rated relay module,
 *     adequate isolation, enclosure, fusing, and comply with local
 *     electrical regulations.
 *   - Never drive a relay coil directly from a GPIO pin.
 *
 * Status:
 *   Core logic implemented and tested via Serial interface.
 *   UI (LCD + encoder) to be added in a future iteration.
 */

#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ------ I/O for relay --------
static constexpr uint8_t RELAY_PIN = 7;
static constexpr bool RELAY_ACTIVE_HIGH = true;

// ------  LCD GLOBALS ------ 
static constexpr uint8_t LCD_ADDR = 0x27;
static constexpr uint8_t LCD_COLS = 16;
static constexpr uint8_t LCD_ROWS = 2;

// Start LCD object
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

static constexpr uint32_t LCD_UPDATE_MS = 500;
static uint32_t lastLcdUpdateMs = 0;

static inline void relayWrite(bool on)
{
  if (RELAY_ACTIVE_HIGH) {
    digitalWrite(RELAY_PIN, on ? HIGH : LOW);
  } else {
    digitalWrite(RELAY_PIN, on ? LOW : HIGH);
  }
}

// Helper to print a value as 3 digits (clamped) to keep the LCD stable
static void print3(uint32_t v)
{
  if (v > 999) v = 999;
  if (v < 100) lcd.print('0');
  if (v < 10)  lcd.print('0');
  lcd.print(v);
}

// ------  ENCODER GLOBALS ------ 
static constexpr uint8_t ENC_PIN_A  = 2;  // CLK
static constexpr uint8_t ENC_PIN_B  = 3;  // DT
static constexpr uint8_t ENC_PIN_SW = 4;  // SW (button)

// Emit one UI "click" per detent (typically 4 transitions)
static constexpr int8_t kStepsPerDetent = 4;

// ------ EEPROM logic --------
namespace RelayConfig {

  static constexpr uint16_t kVersion   = 0x0001; // Version for persistent structure
  static constexpr int      kEepromAddr = 0;     // Base EEPROM address

  // Defaults: 01:20 ON (80s) and 02:40 OFF (160s)
  static constexpr uint32_t kDefaultTonS  = (1UL * 60UL) + 20UL;
  static constexpr uint32_t kDefaultToffS = (2UL * 60UL) + 40UL;

  // Structure of data stored on persistent memory
  // NOTE: Avoid packed unless you must match an external binary format.
  struct Data {
    uint16_t version;
    uint32_t ton_s;
    uint32_t toff_s;
    uint8_t  reserved;
    uint8_t  checksum;
  };

  // Basic checksum implementation (XOR of all bytes except checksum itself)
  static uint8_t calcChecksum(const Data& d)
  {
    const uint8_t* p = reinterpret_cast<const uint8_t*>(&d);
    uint8_t cs = 0;

    // exclude last byte (checksum)
    for (size_t i = 0; i < sizeof(Data) - 1; i++) {
      cs ^= p[i];
    }
    return cs;
  }

  // Sane check mainly for initial boot / validation
  static bool isSane(uint32_t s)
  {
    // time on/off is sane if between 1s and 3600s (1 hour)
    return (s >= 1UL) && (s <= 60UL * 60UL);
  }

  // Class to manage persistent gets and sets
  class Store {
  public:
    void begin()
    {
      Data d;
      EEPROM.get(kEepromAddr, d);

      const bool valid =
          (d.version == kVersion) &&
          (d.checksum == calcChecksum(d)) &&
          isSane(d.ton_s) &&
          isSane(d.toff_s);

      if (!valid) {
        d.version   = kVersion;
        d.ton_s     = kDefaultTonS;
        d.toff_s    = kDefaultToffS;
        d.reserved  = 0;
        d.checksum  = calcChecksum(d);
        EEPROM.put(kEepromAddr, d);
      }

      data_ = d;
    }

    // Getters
    uint32_t getTonS() const  { return data_.ton_s; }
    uint32_t getToffS() const { return data_.toff_s; }

    // Setters (RAM only; call save() to persist)
    bool setTonS(uint32_t s)
    {
      if (!isSane(s)) return false;
      data_.ton_s = s;
      return true;
    }

    bool setToffS(uint32_t s)
    {
      if (!isSane(s)) return false;
      data_.toff_s = s;
      return true;
    }

    // Commit to EEPROM (call when user selects "Save")
    void save()
    {
      data_.version  = kVersion;
      data_.checksum = calcChecksum(data_);
      EEPROM.put(kEepromAddr, data_);
    }

    // Optional: restore defaults (RAM only)
    void setDefaults()
    {
      data_.ton_s  = kDefaultTonS;
      data_.toff_s = kDefaultToffS;
    }

  private:
    Data data_{};
  };

} // namespace RelayConfig

// ------ Relay State Machine --------
class RelayLogic {
public:
  enum class State : uint8_t {
    Stopped,
    On,
    Off
  };

  // Initialize the state machine with relay off
  void begin(uint32_t ton_s, uint32_t toff_s)
  {
    ton_s_  = ton_s;
    toff_s_ = toff_s;
    state_  = State::Stopped;
    relayWrite(false);
  }

  void setTimers(uint32_t ton_s, uint32_t toff_s)
  {
    ton_s_  = ton_s;
    toff_s_ = toff_s;
  }

  void start()
  {
    state_ = State::On;
    last_ms_ = millis();
    relayWrite(true);
  }

  void stop()
  {
    state_ = State::Stopped;
    relayWrite(false);
  }

  bool isRunning() const { return state_ != State::Stopped; }
  State state() const { return state_; }

  void update()
  {
    if (state_ == State::Stopped) return;

    const uint32_t now = millis();
    const uint32_t elapsed_s = (now - last_ms_) / 1000UL;

    switch (state_) {
      case State::On:
        if (elapsed_s >= ton_s_) {
          state_ = State::Off;
          last_ms_ = now;
          relayWrite(false);
        }
        break;

      case State::Off:
        if (elapsed_s >= toff_s_) {
          state_ = State::On;
          last_ms_ = now;
          relayWrite(true);
        }
        break;

      default:
        break;
    }
  }

  uint32_t remainingTimeS() const
  {
    if (state_ == State::Stopped) return 0;

    const uint32_t now = millis();
    const uint32_t elapsed_s = (now - last_ms_) / 1000UL;
    const uint32_t dur = (state_ == State::On) ? ton_s_ : toff_s_;

    return (elapsed_s >= dur) ? 0 : (dur - elapsed_s);
  }

private:
  State state_{State::Stopped};
  uint32_t ton_s_{0};
  uint32_t toff_s_{0};
  uint32_t last_ms_{0};
};

// ------ Encoder class -----
class Ky040 {
public:
  void begin()
  {
    pinMode(ENC_PIN_A, INPUT_PULLUP);
    pinMode(ENC_PIN_B, INPUT_PULLUP);
    pinMode(ENC_PIN_SW, INPUT_PULLUP);

    lastAB_ = readAB();
    lastSw_ = digitalRead(ENC_PIN_SW);
    lastSwChangeMs_ = millis();
    swPressedMs_ = 0;
    clickPending_ = false;
    longPressPending_ = false;
  }

  // Call frequently from loop()
  void update()
  {
    updateRotation();
    updateButton();
  }

  // Rotation delta since last call (-N..+N). Consumes the delta.
  int8_t consumeDelta()
  {
    int8_t d = delta_;
    delta_ = 0;
    return d;
  }

  // Button events (edge-based). Consume on read.
  bool consumeClick()
  {
    bool v = clickPending_;
    clickPending_ = false;
    return v;
  }

  bool consumeLongPress()
  {
    bool v = longPressPending_;
    longPressPending_ = false;
    return v;
  }

private:
  // --- Rotation (simple state table) ---
  uint8_t readAB() const
  {
    // A is MSB, B is LSB
    const uint8_t a = digitalRead(ENC_PIN_A) ? 1 : 0;
    const uint8_t b = digitalRead(ENC_PIN_B) ? 1 : 0;
    return (a << 1) | b;
  }

  void updateRotation()
  {
    const uint8_t ab = readAB();
    if (ab == lastAB_) return;

    // Gray-code transition table:
    // index = (lastAB_ << 2) | ab
    static constexpr int8_t kTable[16] = {
      0, -1, +1,  0,
      +1, 0,  0, -1,
      -1, 0,  0, +1,
      0, +1, -1,  0
    };

    const uint8_t idx = (lastAB_ << 2) | ab;
    const int8_t step = kTable[idx];
    lastAB_ = ab;

    detentAcc_ += step;

    if (detentAcc_ >= kStepsPerDetent) {
      delta_ += 1;
      detentAcc_ = 0;
    } else if (detentAcc_ <= -kStepsPerDetent) {
      delta_ -= 1;
      detentAcc_ = 0;
    }
  }

  // --- Button (debounced) ---
  void updateButton()
  {
    const uint32_t now = millis();
    const uint8_t sw = digitalRead(ENC_PIN_SW); // pull-up: LOW = pressed

    if (sw != lastSw_) {
      // debounce
      if (now - lastSwChangeMs_ >= kDebounceMs) {
        lastSw_ = sw;
        lastSwChangeMs_ = now;

        if (sw == LOW) {
          // pressed
          swPressedMs_ = now;
          longPressFired_ = false;
        } else {
          // released
          if (swPressedMs_ != 0 && !longPressFired_) {
            clickPending_ = true;
          }
          swPressedMs_ = 0;
        }
      }
    }

    // long press detection
    if (lastSw_ == LOW && swPressedMs_ != 0 && !longPressFired_) {
      if (now - swPressedMs_ >= kLongPressMs) {
        longPressPending_ = true;
        longPressFired_ = true;
      }
    }
  }

private:
  static constexpr uint32_t kDebounceMs  = 25;
  static constexpr uint32_t kLongPressMs = 1200;

  uint8_t lastAB_{0};
  int8_t  delta_{0};
  int8_t detentAcc_{0};

  uint8_t lastSw_{HIGH};
  uint32_t lastSwChangeMs_{0};
  uint32_t swPressedMs_{0};
  bool longPressFired_{false};

  bool clickPending_{false};
  bool longPressPending_{false};
};

// ------ Global instances --------
RelayConfig::Store gCfg;
RelayLogic gRelayLogic;

enum class UiField : uint8_t { Ton, Toff };
static UiField uiField = UiField::Ton;
static bool uiEditing = false;

Ky040 enc;
static uint32_t pendingTonS = 0;
static uint32_t pendingToffS = 0;

static void updateAndRestart()
{
  gRelayLogic.stop();
  gRelayLogic.setTimers(gCfg.getTonS(), gCfg.getToffS());
  gRelayLogic.start();
}

// ------ LCD update logic ------
static void updateLcd()
{
  const uint32_t now = millis();
  if (now - lastLcdUpdateMs < LCD_UPDATE_MS)
    return;

  lastLcdUpdateMs = now;

  // -------- Line 1: RUN/STOP + state + remaining --------
  lcd.setCursor(0, 0);

  lcd.print(uiEditing ? "*" : " "); // editing indicator

  // RUN/STOP + optional edit marker
  if (gRelayLogic.isRunning()) {
    lcd.print("RUN ");
  } else {
    lcd.print("STOP");
  }

  // Relay state (ON/OFF/--)
  lcd.print(" S:");
  switch (gRelayLogic.state()) {
    case RelayLogic::State::On:   
      lcd.print("ON "); 
      break;
    
    case RelayLogic::State::Off:  
      lcd.print("OFF"); 
      break;

    default:                      
      lcd.print("-- "); 
      break;
  }

  // Remaining time (xyzs)
  lcd.print(" ");
  uint32_t rem = gRelayLogic.remainingTimeS();
  if (rem > 999) rem = 999;

  if (rem < 100) lcd.print('0');
  if (rem < 10)  lcd.print('0');
  lcd.print(rem);
  lcd.print('s');

  // Ensure rest of line is blank (avoid leftover chars)
  lcd.print(" ");

  // -------- Line 2: editable Ton/Toff with selection marker --------
  lcd.setCursor(0, 1);

  // Render using pending values (what user is editing), not committed EEPROM values
  // Format: ">ON:080 OFF:160" or " ON:080>OFF:160"
  if (uiField == UiField::Ton) 
    lcd.print('>');
  else                         
    lcd.print(' ');

  lcd.print("ON:");
  print3(pendingTonS); // helper below

  lcd.print(' ');

  if (uiField == UiField::Toff) 
    lcd.print('>');
  else                          
    lcd.print(' ');

  lcd.print("OF:");
  print3(pendingToffS);

  // Pad out remainder to avoid leftover characters from previous renders
  lcd.print("    ");
}

// ------ Encoder logic  ------
static void handleEncoder()
{
  enc.update();

  // Rotation
  int8_t d = enc.consumeDelta();
  if (d != 0) {
    // If it feels too fast, divide by 4 using an accumulator.
    if (!uiEditing) {
      // Change selection
      if (d > 0) uiField = (uiField == UiField::Ton) ? UiField::Toff : UiField::Ton;
      if (d < 0) uiField = (uiField == UiField::Ton) ? UiField::Toff : UiField::Ton;
    } else {
      // Edit selected field
      auto clamp = [](int32_t v) -> uint32_t {
        if (v < 1) return 1;
        if (v > 3600) return 3600;
        return static_cast<uint32_t>(v);
      };

      if (uiField == UiField::Ton) {
        pendingTonS = clamp(static_cast<int32_t>(pendingTonS) + d);
      } else {
        pendingToffS = clamp(static_cast<int32_t>(pendingToffS) + d);
      }
    }
  }

  // Click: toggle editing
  if (enc.consumeClick()) {
    uiEditing = !uiEditing;
  }

  // Long press: save + apply
  if (enc.consumeLongPress()) {
    if (gCfg.setTonS(pendingTonS) && gCfg.setToffS(pendingToffS)) {
      gCfg.save();
      updateAndRestart();
    }
    uiEditing = !uiEditing;
  }
}

// ------ Arduino standard functions --------
void setup()
{
  Serial.begin(9600);

  // Wait for serial (max 2s). Useful during dev; doesn't block forever in the field.
  const unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 2000)) { }

  // Start LCD configurations
  Wire.begin();
  lcd.init();
  lcd.backlight();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Configurable");
  lcd.setCursor(0, 1);
  lcd.print("Relay");
  delay(1000);
  lcd.clear();

  // Pin out configuration for relay
  pinMode(RELAY_PIN, OUTPUT);
  relayWrite(false);

  // Load persistent configuration
  gCfg.begin();

  // Initialize relay logic with loaded values
  gRelayLogic.begin(gCfg.getTonS(), gCfg.getToffS());
  gRelayLogic.start();

  // Initialize encoder class
  pendingTonS = gCfg.getTonS();
  pendingToffS = gCfg.getToffS();

  enc.begin();

  Serial.println(F("Configurable Relay Started"));
  Serial.print(F("Ton(s): "));  Serial.println(gCfg.getTonS());
  Serial.print(F("Toff(s): ")); Serial.println(gCfg.getToffS());
}

void loop()
{
  gRelayLogic.update();
  updateLcd();
  handleEncoder();

  // Serial commands are just for testing before LCD/encoder UI:
  // start | stop | show | ton <s> | toff <s>
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd.startsWith("start")) {
      gRelayLogic.start();
      Serial.println(F("OK start"));

    } else if (cmd.startsWith("stop")) {
      gRelayLogic.stop();
      Serial.println(F("OK stop"));

    } else if (cmd.startsWith("ton ")) {
      const long val = cmd.substring(4).toInt();
      if (val <= 0) {
        Serial.println(F("ERR invalid value"));
        return; // or 'continue;' if you wrap this in a loop; here return exits loop() early
      }

      const uint32_t v = static_cast<uint32_t>(val);
      if (gCfg.setTonS(v)) {
        gCfg.save();
        updateAndRestart();
        Serial.println(F("OK ton saved"));
      } else {
        Serial.println(F("ERR ton out of range"));
      }

    } else if (cmd.startsWith("toff ")) {
      const long val = cmd.substring(5).toInt();
      if (val <= 0) {
        Serial.println(F("ERR invalid value"));
        return;
      }

      const uint32_t v = static_cast<uint32_t>(val);
      if (gCfg.setToffS(v)) {
        gCfg.save();
        updateAndRestart();
        Serial.println(F("OK toff saved"));
      } else {
        Serial.println(F("ERR toff out of range"));
      }

    } else if (cmd.startsWith("show")) {
      Serial.print(F("Ton(s): "));   Serial.println(gCfg.getTonS());
      Serial.print(F("Toff(s): "));  Serial.println(gCfg.getToffS());
      Serial.print(F("Running: "));  Serial.println(gRelayLogic.isRunning() ? F("yes") : F("no"));

    } else {
      Serial.println(F("Commands: start | stop | show | ton <s> | toff <s>"));
    }
  }
}