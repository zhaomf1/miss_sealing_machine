# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

厌氧型封膜机 (Anaerobic Film Sealing Machine) firmware — an embedded control system for an automated laboratory film-sealing device. Controls motors, cylinders, suction cups, temperature, and sensors through a coordinated workflow.

- **MCU**: STM32F405xx (ARM Cortex-M4, 168 MHz)
- **RTOS**: FreeRTOS V10.3.1 via CMSIS-RTOS V2 wrapper (CMSIS-RTOS API like `osThreadNew`, `osDelay`, `osMutexAcquire`)
- **Build**: MDK-ARM (Keil uVision 5), configured via `MDK-ARM/mini_evol.uvprojx`
- **Codegen**: Initialized from STM32CubeMX (`mini_evol.ioc`); peripheral init files (`Core/Src/peripheral/`) are auto-generated but have been customized

## Commands

- **Build**: Open `MDK-ARM/mini_evol.uvprojx` in Keil uVision 5 and press F7 (Build).
- **Flash/Debug**: J-Link debugger; download with F8, debug with Ctrl+F5 in Keil.
- **No CLI build**: There is no Makefile or CMake setup — this is a Windows-only MDK-ARM project.

## Architecture

### Communication Architecture (Two-Tier Modbus)

The MCU acts as both a **Modbus Slave** and a **Modbus Master**:

1. **Host ↔ MCU (USART1, 115200 baud)**: Modbus RTU Slave. Split into three layers:
   - `usart_comm.c` — Protocol framing only: CRC check, function code dispatch (0x03/0x06/0x10), response building, FreeRTOS task. Contains **no business logic** (~260 lines).
   - `modbus_slave_reg.c` — Register business logic: register map, read dispatch (`modbus_reg_read_value()`), write dispatch (`modbus_reg_write_execute()` / `modbus_reg_write32_execute()`), action status tracking (0x00A0), fault bitmap (0x0002), temperature/compound-action stubs (~360 lines).
   - `app_control.c` — Application layer: 485 peripheral init sequence (`appInitTask`), EEPROM read/write wrappers (~150 lines).

2. **MCU ↔ Peripherals (USART3, RS-485)**: Modbus RTU Master. The MCU communicates with four slave devices on the RS-485 bus: suction cup (addr 0x01), suction cylinder (0x02), seal cylinder (0x03), and Fujun stepping motor (0x04). Implemented in `modbus_rtu.c` with mutex-guarded transactions, retry logic (5 retries), CRC validation, and error logging.

Additional serial peripherals: PH board (USART2), temperature controller (USART4), OD sensor (USART6).

### Application Layer (`Core/Src/app/`)

| File | Role |
|------|------|
| `main.c` | Entry point, peripheral init, FreeRTOS launch |
| `freertos.c` | Task creation (5 tasks), inter-task queue setup |
| `app_control.c` | Power-on initialization sequence for 485 peripherals (suction cup → cylinders → Fujun motor, each max 3 retries), AT24C02 EEPROM r/w wrappers |
| `usart_comm.c` | Modbus RTU Slave handler — register table, function codes 0x03/0x06/0x10, compound actions (吸膜/铺膜/封膜/取放孔板) |
| `dev_bldc_ctrl.c` | Brushless DC motor control via RS-485 Modbus |
| `dev_cylinder.c` | Dual electric cylinders (suction + seal) via RS-485 Modbus |
| `dev_suction_cup.c` | Electric suction cup via RS-485 Modbus |
| `dev_fujun_motor.c` | Fujun stepping motor (丝杆模组) via RS-485 Modbus |
| `dev_temp_ctrl.c` | Temperature controller via USART4 |
| `dev_od_ctrl.c` | OD (optical density) sensor |
| `dev_ph_ctrl.c` | PH sensor board |

### FreeRTOS Task Layout

| Task | Priority | Role |
|------|----------|------|
| `uartCommTask` | `osPriorityRealtime` | Modbus Slave handler — receives frames from queue, dispatches commands |
| `appInitTask` | `osPriorityNormal` | One-shot init of 485 peripherals, self-deletes when done |
| `tempAlarmTask` | `osPriorityNormal` | Polls temperature alarm status every 60s |
| `defaultTask` | `osPriorityNormal` | Idle loop with IWDG refresh; test code gated by `TEST_CODE` macro (default 0) |
| `timerTask` | `osPriorityNormal` | Timer task (defined but not always active) |

### EEPROM Layout (AT24C02, I2C1)

Stores persistent parameters as uint32_t values:
- `0x00`: Zero/归零 position (film box position)
- `0x04`: Suction/seal position
- `0x08`: Film-laying position
- `0x0C`: Well plate pick/place position
- `0x10`: Temperature control params
- `0x14`: Press time
- `0x18`: Total sealing count

### Protocol Details

- **Host protocol document**: `封膜机下位机通讯协议V1.0.docx` (extracted in `docx_output.txt`)
- **Modbus registers**: System commands (0x0000-0x0002), workflow control (0x0010-0x0012), debug actions (0x0020-0x0064), EEPROM params (0x0070-0x0079), action status (0x00A0)
- **Register 0x0020 (temperature control)**: WO register — write to set target temperature in 0.1°C units (0~2000). Calls `PID_SetTemperature()` + `temp_ctrl_start()`.
- **Register 0x0021 (temperature query)**: RO register — reads current temperature in 0.1°C units via `temp_ctrl_get()`.
- **Registers 0x0070-0x0079 (EEPROM params)**: R/W registers backed by AT24C02 EEPROM. 0x0070/0x0072/0x0074/0x0076 are uint32 (use 0x10 function code), 0x0078/0x0079 are uint16 (use 0x06/0x03). Mapping: 0x0070→EEPROM_ADDR_ZERO, 0x0072→EEPROM_SUCK_SEAL, 0x0074→EEPROM_PAVE, 0x0076→EEPROM_GET_PLACE, 0x0078→EEPROM_TEMP_CTRL, 0x0079→EEPROM_PRESS_TIME.
- **Register 0x00A0 (action status)**: High byte = last debug action address (0x20/0x30/0x40/0x41/0x50/0x60-0x64), low byte = result (0x00=success, 0x01=failure, 0xFF=no history). Host polls this after writing any debug action register.
- **Registers 0x0060-0x0064**: Placeholder stubs — accept commands, record failure status to 0x00A0, print placeholder message. Implementation stubs in `cmd_fujun_home()` etc.
- **Error codes**: Bit-mapped in register 0x0002 — bit0=temperature, bit1=suction cylinder, bit2=seal cylinder, bit3=lead screw, bit4=suction cup. Automatically updated by debug action handlers.
- **Firmware version**: `FIRMWARE_VERSION "BetaV1.2.2"` defined in `main.h`
- **JSON protocol** (largely deprecated in favor of Modbus): Defined in `json_key.h` and `usart_comm.h` — JSON over USART1 with command types for motors, valves, PH, OD, temperature, and RGB light

### Key Design Patterns

- **`TEST_CODE` macro** in `freertos.c`: All hardware test code is gated behind `#if TEST_CODE`. Set to 0 for production, 1 for debugging individual peripherals.
- **DMA double-buffering**: USART1 and USART3 use DMA with a backup buffer scheme (`host_rx_backup`, `modbus_rtu_rx_backup`) to prevent DMA overwrites during processing.
- **Modbus mutex**: All RS-485 transactions are serialized through a mutex with priority inheritance (`osMutexPrioInherit`).
- **Retry on all errors**: `modbus_rtu.c` retries any failed transaction up to 5 times (no distinction between error types).
- **Action status tracking**: Debug action commands (0x0030/0x0040/0x0041/0x0050/0x0060-0x0064) execute asynchronously via compound action queue, record status to register 0x00A0, and auto-update fault bitmap 0x0002. Temperature register 0x0020 is a synchronous parameter write (no action tracking).
