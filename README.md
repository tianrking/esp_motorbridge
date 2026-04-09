# ESP MotorBridge (ESP-IDF 5.5)

CAN motor bridge firmware for ESP32, with:
- multi-vendor motor protocol routing (`damiao` + `robstride`)
- serial CLI control and diagnostics
- Wi-Fi AP + UDP host bridge
- web control panel (`/api/control`) and WebSocket endpoint (`/ws`)

## Architecture (Current Code)

```text
main/
  app/
    app_main.c
    command_router.c
    command_router_admin.c
    command_router_damiao_reg.c
    command_router_feedback.c
    serial_cli.c
    serial_cli_commands.c
    serial_cli_watch.c
    serial_cli_output.c

  config/
    app_config.c/h

  core/
    control_loop.c
    motor_manager.c/h
    safety_manager.c/h

  net/
    wifi_udp_bridge.c/h
    web_control.c
    web_control_server.c/h
    web_control_page.c/h
    web_control_actions.c/h
    web_actions_demo.c/h
    web_actions_id_vendor.c/h
    web_actions_motor.c/h
    web_control_demo.c/h
    web_control_dispatch.c/h

  protocol/
    host_protocol.c/h

  storage/
    param_store.c/h

  transport/
    can_manager.c/h
    can_manager_internal.h
    can_tx.c
    can_rx.c
    can_recovery.c
    can_scan.c

  vendors/
    motor_vendor.c/h
    damiao/
      damiao_protocol.c/h
      damiao_admin_codec.c
      damiao_control_codec.c
      damiao_register_codec.c
      damiao_feedback_parser.c
      damiao_vendor.c
    robstride/
      robstride_protocol.c/h
      robstride_vendor.c
```

## Key Runtime Behavior

- Motor slots are initialized to `1..127`.
- Default vendor binding at boot:
  - IDs `1..7` -> `damiao`
  - IDs `8..127` -> `robstride`
- Optional Damiao register remap (`rid 7/8`) at boot is controlled by Kconfig:
  - `CONFIG_MOTORBRIDGE_APPLY_DEFAULT_ID_MAP_AT_BOOT`
  - if enabled, maps `1..7` feedback to `0x11..0x17`
- Control loop sends per-motor command frames through each motor's bound vendor encoder.
- Feedback RX is parsed per-motor by each bound vendor parser.
- Safety policy (current code): offline timeout marks motors offline; offline E-STOP is disabled for burn-in.

## Interfaces

- Wi-Fi AP: from `CONFIG_MOTORBRIDGE_WIFI_SSID` / `CONFIG_MOTORBRIDGE_WIFI_PASSWORD`
- UDP bridge: port `CONFIG_MOTORBRIDGE_UDP_PORT` (default typically `9000`)
- Web UI: `http://192.168.4.1/`
- HTTP control API: `GET /api/control?...`
- WebSocket endpoint: `ws://192.168.4.1/ws`

Note:
- The server supports WS control endpoint.
- Current built-in web page uses HTTP `/api/control` requests.

## Serial CLI Commands

In `idf.py monitor`, type:

- `help`
- `scan [vendor] <min_id> <max_id>`
- `show`
- `show_state`
- `state <id>`
- `watch <on|off> [ms] [poll]`
- `poll <id|all> [wait_ms]`
- `clear_online`
- `candump <on|off>`
- `enable <id|all>`
- `disable <id|all>`
- `mode <id|all> <0..4>` (`1=MIT 2=PosVel 3=Vel 4=ForcePos`)
- `setid <old_id> <new_id>` (Damiao only, store + verify)

## Build

```bash
cd /home/w0x7ce/Downloads/dm_candrive/ESP_motorbridge
source /home/w0x7ce/Downloads/esp_idf_5.5/export.sh
idf.py set-target esp32
idf.py build
```

Flash + monitor:

```bash
idf.py -p <PORT> flash monitor
```

For ESP32-S3:

```bash
idf.py set-target esp32s3
idf.py build
```

## Quick Validation

1. Boot and enter monitor.
2. CLI smoke test:
   - `scan damiao 1 7`
   - `show_state`
   - `enable 1`
   - `mode 1 1`
   - `poll 1 30`
3. Open `http://192.168.4.1/` and test:
   - `Enable All` / `Disable All`
   - per-motor mode switch (`disable -> clear_error -> set_mode -> enable -> apply`)
   - demo start / stop / reset
4. Optional WS test with custom client on `/ws`.

## License

MIT. See [LICENSE](./LICENSE).
