# configurable-relay

A configurable cyclic relay controller built for the **Arduino Nano Every**, intended for reliable long‑running operation and safe integration with external power switching hardware.

The device switches a relay **ON for a configurable time (Ton)** and **OFF for a configurable time (Toff)**, repeating continuously. Configuration is stored in persistent memory and survives power loss.

The project is implemented as a clean, minimal embedded system using modern C++ features, while remaining deterministic, readable, and suitable for unattended use.

---

## Features

* Cyclic relay control (ON / OFF loop)
* Ton / Toff configurable in **seconds**
* Configuration stored in EEPROM (flash emulation on Nano Every)
* Versioned and checksummed persistent data
* Non‑blocking timing based on `millis()`
* Robust startup validation with safe defaults
* Clear separation between:

  * Persistent configuration logic
  * Relay state machine
  * User interface layer
* **16×2 I²C LCD status display**
* **Rotary encoder (KY‑040) for local configuration**
* Edit / select / save interaction without blocking
* Serial command interface retained for testing and debug

---

## Target Hardware

* **Microcontroller:** Arduino Nano Every (ATmega4809)
* **Display:** 16×2 HD44780‑compatible LCD with I²C backpack
* **User Input:** KY‑040 rotary encoder with push button
* **Relay control:** External relay or contactor (logic‑level controlled)
* **Power:** Depends on relay/contactor and control PSU used

> ⚠️ **Important – Mains Safety**
> This firmware only implements control logic.
>
> When switching mains voltage or motor loads:
>
> * Use a properly rated relay or contactor (AC‑3 rated for motors)
> * Ensure adequate isolation, enclosure, strain relief, and fusing
> * Follow local electrical and safety regulations
> * Never drive mains loads directly from Arduino I/O pins

---

## Relay Logic

The relay operates as a simple deterministic state machine:

```
[ ON  ] --(Ton elapsed)--> [ OFF ]
[ OFF ] --(Toff elapsed)-> [ ON  ]
```

* Timing is handled using `millis()`
* No blocking delays are used
* Relay can be started or stopped at runtime
* Configuration changes are only applied when explicitly saved

---

## Persistent Configuration

The following parameters are stored in EEPROM:

* Structure version
* Ton (seconds)
* Toff (seconds)
* Reserved byte (future use)
* Checksum (XOR)

On startup:

* EEPROM data is validated (version, checksum, sane limits)
* If invalid or corrupted, safe default values are restored automatically

### Default values

* **Ton:** 80 seconds (1 min 20 s)
* **Toff:** 160 seconds (2 min 40 s)

---

## LCD + Rotary Encoder UI

The system includes a minimal, non‑blocking local user interface:

### Display

* **Line 1:**

  * RUN / STOP status
  * Current relay state (ON / OFF)
  * Remaining time in current state

* **Line 2:**

  * Ton and Toff values (seconds)
  * Visual selection marker (`>`)
  * Edit mode indicator (`*`)

### Encoder interaction

* **Rotate (not editing):** Select Ton or Toff
* **Short press:** Toggle edit mode
* **Rotate (editing):** Adjust selected value
* **Long press:** Save configuration to EEPROM and apply immediately

Edits are performed on **pending values** and only committed on save, preventing accidental changes.

The KY‑040 encoder is decoded using a Gray‑code state table with detent normalization (4 transitions per click → 1 logical step).

---

## Serial Commands (Testing Interface)

The serial interface is retained for testing and debugging:

* `start` → start relay cycle
* `stop` → stop relay cycle
* `show` → show current configuration and state
* `ton <seconds>` → set ON time
* `toff <seconds>` → set OFF time

Example:

```
ton 120
toff 300
```

---

## Software Structure

```
configurable-relay/
├── configurable-relay.ino
└── README.md
```

Key components:

* `RelayConfig::Store`
  Handles EEPROM persistence, validation, defaults

* `RelayLogic`
  Implements the relay state machine

* `Ky040`
  Rotary encoder driver with debouncing and detent handling

* LCD update logic
  Periodic, non‑blocking rendering of system state

---

## Design Principles

* No dynamic memory allocation
* No blocking delays in runtime logic
* No exceptions or STL containers
* Deterministic behavior
* Clear ownership of state
* Defensive startup behavior
* Embedded‑friendly modern C++ ("C with guardrails")

---

## Possible Extensions

* Additional menu entries (auto‑start, reset, presets)
* Acceleration / coarse‑fine encoder control
* Backlight timeout or dimming
* External enable / inhibit input
* Status LED or buzzer output

---

## License

MIT License. See `LICENSE` file for details.
