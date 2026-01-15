# configurable-relay

A configurable cyclic relay controller built for the **Arduino Nano Every**.

The device switches a relay **ON for a configurable time (Ton)** and **OFF for a configurable time (Toff)**, repeating continuously. Configuration is stored in persistent memory and survives power loss.

This project is designed as a clean, minimal embedded system using modern C++ features while remaining fully deterministic and suitable for long-term unattended operation.

---

## Features

- Cyclic relay control (ON / OFF loop)
- Ton / Toff configurable in **seconds**
- Configuration stored in EEPROM (flash emulation on Nano Every)
- Versioned and checksummed persistent data
- Non-blocking timing based on `millis()`
- Clean separation between:
  - Persistent configuration logic
  - Relay state machine
- Serial command interface for testing
- Architecture ready for LCD + rotary encoder UI

---

## Target Hardware

- **Microcontroller:** Arduino Nano Every (ATmega4809)
- **Relay:** External relay module (logic-level input)
- **Power:** Depends on relay module used

> ⚠️ **Important**  
> When switching mains voltage:
>
> - Use a relay module rated for the load
> - Ensure proper isolation, enclosure, strain relief, and fusing
> - Follow local electrical safety regulations

---

## Relay Logic

The relay operates as a simple state machine:
[ ON ] –(Ton elapsed)–> [ OFF ]
[ OFF ] –(Toff elapsed)-> [ ON ]

- Timing is handled using `millis()`
- No blocking delays are used
- Relay can be started or stopped at runtime

---

## Persistent Configuration

The following parameters are stored in EEPROM:

- Structure version
- Ton (seconds)
- Toff (seconds)
- Reserved byte (future use)
- Checksum (XOR)

On startup:

- EEPROM data is validated (version, checksum, sane limits)
- If invalid, safe default values are restored automatically

Defaults:

- **Ton:** 80 seconds (1 min 20 s)
- **Toff:** 160 seconds (2 min 40 s)

---

## Serial Commands (Testing Interface)

Until the LCD + encoder UI is implemented, configuration can be tested via Serial:

- **start** -> start relay cycle
- **stop** -> stop relay cycle
- **show** -> show current configuration and state
- **ton** -> set ON time
- **toff** -> set OFF time

Example:
ton 120
toff 300

---

## Software Structure

configurable-relay/
|── configurable-relay.ino
|── README.md

Key components:

- `RelayConfig::Store`  
  Handles EEPROM persistence, validation, defaults
- `RelayLogic`  
  Implements the relay state machine
- `loop()`  
  Updates state machine and handles test interface

---

## Design Principles

- No dynamic memory allocation
- No blocking delays
- No exceptions or STL containers
- Clear ownership of state
- Defensive startup behavior
- Readable, maintainable embedded C++ ("C with guardrails")

---

## Planned Extensions

- 16x2 LCD display (I²C)
- Rotary encoder for local configuration
- Menu-based UI (edit / apply / cancel)
- Optional auto-start configuration
- Optional factory reset

---

## License

MIT License. See `LICENSE` file for details.
