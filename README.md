# ESP MotorBridge (ESP-IDF 5.5)

A clean, layered ESP-IDF architecture for CAN motor bridge firmware.
Current vendor plugin: `damiao`.

## Project Structure

```text
main/
  app/
    app_main.c           # boot / task wiring only
    command_router.c     # host command dispatch + feedback routing
  config/
    app_config.c/h       # runtime config from sdkconfig
  core/
    motor_manager.c/h    # motor state cache + command cache
    control_loop.c/h     # periodic command sender (no PID math)
    safety_manager.c/h   # timeout/offline safety
  protocol/
    host_protocol.c/h    # host CAN protocol parsing
  transport/
    can_manager.c/h      # TWAI driver + rx/scan tasks
  vendors/
    motor_vendor.h       # vendor abstraction interface
    damiao/
      damiao_protocol.c/h
      damiao_vendor.c
  storage/
    param_store.c/h      # NVS init and param hooks
```

## Design Principles

- app layer only orchestrates, no protocol/math details
- host protocol decoupled from vendor protocol
- vendor implemented behind `motor_vendor_ops_t`
- core logic is vendor-agnostic as much as possible
- adding new vendor should be additive, not rewrite existing files

## How to Add Another Motor Vendor

1. Add `main/vendors/<vendor>/...` protocol encode/decode implementation.
2. Implement `motor_vendor_ops_t` in `<vendor>_vendor.c`.
3. Switch `motor_vendor_active()` to desired vendor (or make runtime-selectable later).
4. Keep `host_protocol` and `core/*` unchanged.

## Build

```bash
cd /home/w0x7ce/Downloads/dm_candrive/ESP_motorbridge
source /home/w0x7ce/Downloads/esp_idf_5.5/export.sh
idf.py set-target esp32
idf.py build
```

For ESP32-S3:

```bash
idf.py set-target esp32s3
idf.py build
```

## Status

- `esp32` build: pass
- `esp32s3` build: pass

## Runtime Defaults

- AP mode: `ESP_motorbridge` / `12345678`
- Web control: `http://192.168.4.1/`
- UDP control port: `9000`
- Default motor count: `7`
- Default Damiao mapping: motor `1..7` -> feedback `0x11..0x17`
- Default MIT gains: `kp=0.5`, `kd=0.08`

## License

MIT. See [LICENSE](./LICENSE).
