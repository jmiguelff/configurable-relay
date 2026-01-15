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

// ------ I/O for relay --------
static constexpr uint8_t RELAY_PIN = 7;
static constexpr bool RELAY_ACTIVE_HIGH = true;

static inline void relayWrite(bool on)
{
  if (RELAY_ACTIVE_HIGH) {
    digitalWrite(RELAY_PIN, on ? HIGH : LOW);
  } else {
    digitalWrite(RELAY_PIN, on ? LOW : HIGH);
  }
}

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

// ------ Global instances --------
RelayConfig::Store gCfg;
RelayLogic gRelayLogic;

static void updateAndRestart()
{
  gRelayLogic.stop();
  gRelayLogic.setTimers(gCfg.getTonS(), gCfg.getToffS());
  gRelayLogic.start();
}

// ------ Arduino standard functions --------
void setup()
{
  Serial.begin(9600);

  // Wait for serial (max 2s). Useful during dev; doesn't block forever in the field.
  const unsigned long t0 = millis();
  while (!Serial && (millis() - t0 < 2000)) { }

  pinMode(RELAY_PIN, OUTPUT);
  relayWrite(false);

  // Load persistent configuration
  gCfg.begin();

  // Initialize relay logic with loaded values
  gRelayLogic.begin(gCfg.getTonS(), gCfg.getToffS());
  gRelayLogic.start();

  Serial.println(F("Configurable Relay Started"));
  Serial.print(F("Ton(s): "));  Serial.println(gCfg.getTonS());
  Serial.print(F("Toff(s): ")); Serial.println(gCfg.getToffS());
}

void loop()
{
  gRelayLogic.update();

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