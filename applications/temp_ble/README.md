# BLE Temperature and humidity sensor

Reads temperature and humidity from the onboard Sensirion SHT4x sensor and exposes it over BLE
as a standard Environmental Sensing Service. Retrieve it with the Nordic nRFConnect app (or any
other generic BLE ESS-capable app).

## Project Hardware

| Item | Details |
|------|---------|
| Board | CoffeeCaller nRF52840 |
| Sensor | Onboard Sensirion SHT4x (I2C0, address `0x46`) |

## Build

This application targets NCS v3.2.1 and builds in the main CoffeeCaller workspace.
See [`CoffeeCaller/README.md`](../../README.md) for workspace setup.

```bash
west build -b coffeecaller_nrf52/nrf52840 ./CoffeeCaller/applications/temp_ble -p always
```

## Flash

```
lsusb   # expect: 1915:c0ca Nordic Semiconductor ASA CoffeeCaller nRF52 (CDC ACM)
west flash --runner openocd
```

This project does not include the UC2 bootloader, flashing was done with FTDI2232H.
See [`zigbee_temp/README.md`](../zigbee_temp/README.md#flash-ft2232h-via-openocd) for FTDI2232H
wiring diagram and flashing instructions.

## Application behaviour

### BLE

The board advertises as **"CoffeeCaller"**, general discoverable and connectable, with no
pairing or bonding required (open read access). It exposes the standard Environmental Sensing
Service (UUID `0x181A`) with a Temperature (`0x2A6E`) and a Humidity (`0x2A6F`) characteristic,
both readable and notifiable.

To use it with nRFConnect: scan for devices, connect to "CoffeeCaller", open the Environmental
Sensing Service, then read or enable notifications on the Temperature and Humidity
characteristics to see live values.

Values refresh from the onboard SHT4x every 2 seconds, unconditionally. Up to 4 simultaneous
BLE connections are supported.

## Serial console

```
screen /dev/ttyACM0 115200
```

Logs each sensor reading and BLE connect/disconnect event.
