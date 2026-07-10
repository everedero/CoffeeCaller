# OpenTheDamnWindowCaller

Zigbee coordinator and BLE gateway, built for the CoffeeCaller nRF52840 board.

Ventilation alarm: it buzzes when the inside room is warmer than outside, to wake you up so you
go open the window to cool the room down. It also warns you when it is time to close the windows
when the sun is back.

The buzzer can be disabled, and an LED indicator tells you if it’s fresher inside (orange LED)
or outside (blue LED).

The board receives temperature and humidity from two Sonoff SNZB-02P Zigbee sensors, and
re-broadcasts them over BLE as an Environmental Sensing Service (ESS), readable by any
standard BLE app (nRF Connect, Home Assistant, but I could not find any F-Droid open-source
equivalent app).

CoffeeCaller onboard temperature and humidity sensor is not used for this project yet.

This application was completely vibe-coded, do not use it for reference.

## Project Hardware

| Item | Details |
|------|---------|
| Board | CoffeeCaller nRF52840 |
| Sensors | [Sonoff SNZB-02P](https://sonoff.tech/product/gateway-and-sensors/snzb-02p/) Zigbee sensor x2 |
| Programmer | FTDI FT2232H mini module |

Since I am cheap and could not make the UC2 bootloader work, I use a FTDI FT2232 mini module, with
SWD resistor hack, as a flashing probe.

## Workspace setup

This application targets NCS v2.8.0, the last nRF Connect SDK release that still includes the
ZBOSS Zigbee stack. It needs its own, separate west workspace, do not reuse a workspace already
set up for a different NCS version.

### Prerequisites

- Python 3 with `pip` and `venv`
- The [Zephyr SDK](https://docs.zephyrproject.org/latest/develop/toolchains/zephyr_sdk.html)
  (`arm-zephyr-eabi` toolchain)
- `ninja` and `device-tree-compiler` (Debian/Ubuntu: `apt install ninja-build device-tree-compiler`)

### Initialize the workspace

If west is not installed:

```bash
python3 -m venv .venv
. .venv/bin/activate
pip install west
```

Now bring in the Nordic nRF Connect SDK:
```bash
west init -m https://github.com/nrfconnect/sdk-nrf --mr v2.8.0 ncs/v2.8.0
cd ncs/v2.8.0
west update

pip install -r nrf/scripts/requirements.txt -r zephyr/scripts/requirements.txt
```

### Apply the required Zigbee/BLE AES fix

NCS v2.8.0 has a crypto-selection bug: when both BLE and Zigbee are enabled, it picks the BLE
controller's AES routine (`bt_encrypt_be()`) over Zigbee's own, even with
`CONFIG_ZIGBEE_USE_SOFTWARE_AES=y` set. `bt_encrypt_be()` byte-reverses its input/output
(big-endian convention), which is wrong for ZBOSS, Zigbee network-key transport silently fails
and sensors can never join. This lives in the vendored `nrf/` checkout, not in this repo, so it
must be reapplied after every fresh `west init`/`west update` of this workspace:

```bash
cd ncs/v2.8.0/nrf
git apply /path/to/CoffeeCaller/applications/zigbee_temp/patches/zb_nrf_crypto.patch
```

See [`patches/zb_nrf_crypto.patch`](patches/zb_nrf_crypto.patch) for the exact diff.

### Clone this repository

CoffeeCaller is not part of the NCS manifest, clone it as a sibling of `ncs/v2.8.0`:

```bash
git clone <your-fork-url> CoffeeCaller
```

This is the resulting tree:

```bash
ncs
├── CoffeeCaller
├── bootloader
├── modules
├── nrf
├── nrfxlib
├── v2.8.0
├── zephyr
[...]
```

## Build

```bash
cd ./ncs/v2.8.0
west build -b coffeecaller_nrf52/nrf52840 ../CoffeeCaller/applications/zigbee_temp -p always -- -DZEPHYR_EXTRA_MODULES="$(realpath ../CoffeeCaller)"
```

## Flash (FT2232H via OpenOCD)

### Wiring (SWD)

| FT2232H pin | Signal | Target |
|-------------|--------|--------|
| ADBUS0 | TCK | SWCLK |
| ADBUS1 | TDI (through 270 Ω) | SWDIO |
| ADBUS2 | TDO (direct) | SWDIO |

Connect FTDI2232H GND to CoffeeCaller GND. Both USB, from FTDI and target board, should be plugged.

Verify the programmer is detected:
```
lsusb | grep FT2232
```

Note: do not connect nRST on ADBUS4, for some reason it does not work.
CoffeeCaller do not provide a proper nRST pin anyway.

### Flash command

```bash
west flash --runner openocd
```

If flashing fails, check that the OpenOCD config uses `ftdi` as the interface (not `jlink`).

## Pairing sensors

The board acts as a Zigbee coordinator. It opens the network on boot and keeps it open for
3 minutes.

Only the outside sensor is pinned by its IEEE (MAC) address (see `outside_ieee_addr` in
`src/main.c`). The inside sensor needs no hardcoding: whichever other Sonoff joins the
network is automatically assigned to the inside slot.

### Finding the outside sensor's MAC address to hard-code

This is only needed when pairing a new or replacement outside sensor: its IEEE address isn't
known ahead of time, so retrieve it from the coordinator's serial console:

1. Connect: `screen /dev/ttyACM0 115200`.
2. Pair the sensor (see "Standard pairing" below) or power-cycle it if already paired, either
   triggers a `ZB_ZDO_SIGNAL_DEVICE_UPDATE` signal.
3. Look for a pair of log lines like:
   ```
   I: TC update: short=0xebd5 status=3 tc_action=0 parent=0x0000
   I:   MAC: 18:69:0a:ff:fe:68:bf:16
   ```
   The `MAC: ...` line is the sensor's IEEE address.

**Byte order gotcha**: that log line prints the address MSB-first (human-readable order), but
`outside_ieee_addr` in `src/main.c` stores it as a `zb_uint8_t[8]` array in the SDK's index
order, which is the *reverse* of the printed string. E.g. `18:69:0a:ff:fe:68:bf:16` printed
becomes `{ 0x16, 0xbf, 0x68, 0xfe, 0xff, 0x0a, 0x69, 0x18 }` in the array literal. Reverse the
byte order when transcribing, update `outside_ieee_addr`, and rebuild.

### Standard pairing

1. Power on the CoffeeCaller (or press **SW1** to reopen the network).
   The Zigbee LED (LED3) lights up while the network is open.
2. On the SNZB-02P, press and hold the small button on the side for **5 seconds** until the
   LED blinks rapidly: the sensor enters pairing mode.
3. Wait up to 30 seconds for the join to complete. The sensor LED will blink once to confirm.

Repeat for the second sensor (press **SW1** again to reopen the network if needed).

### Battery-removal trick (if the sensor was previously paired elsewhere)

If the sensor is stuck on a previous network (e.g. Zigbee2MQTT, ZHA) and the button press
does not trigger a new join:

1. Remove the battery
2. Reinsert the battery
3. The sensor will perform an unsecured join and pair to whatever coordinator is open.

### Device reboot

If you reboot the CoffeeCaller board, sensors will automatically rejoin after a while.
However, it is much faster to force the joining process by pressing the sensors button
after CoffeeCaller has started.


## Application behaviour

### BLE

The board advertises as "CoffeeCaller" and exposes an ESS service (UUID 0x181A) with two
Temperature + Humidity characteristic pairs labelled "Sensor 1" and "Sensor 2".

Use nRF Connect app, or any BLE ESS-compatible app to read and subscribe to
notifications.
Values update whenever the sensor reports (every 10–300 s or on >=0.5 °C / >= 1 % change).

### LEDs

| LED | Meaning |
|-----|---------|
| LED1 (white) | Off in normal operation; blinks once per second if the inside or outside sensor hasn't reported in over 10 minutes |
| LED3 (white) | Zigbee network open, on while pairing window is active |
| LED4 (white) | Identify mode, blinks fast during ZCL Identify |
| RGB LED 0 (red) | Ventilation alarm enabled |
| RGB LED 1 (blue/orange) | Outside vs inside temperature: blue when outside is cooler, orange when outside is hotter |

### Buttons

| Button | Function |
|--------|---------|
| SW0 | Toggle ventilation alarm on/off |
| SW1 | Reopen Zigbee network for pairing (3-minute window) |
| SW4 (hold) | Factory reset the coordinator |

### Ventilation alarm

Press SW0 to enable. The first RGB LED turns red.

The board monitors the difference between indoor (Sensor 1) and outdoor (Sensor 2)
temperatures. Every 60 seconds a sample is taken; the alarm triggers when:

- The 5-minute rolling average of (inside - outside) > 0.2 °C, and
- Inside temperature > 20 °C, and
- The alarm has not triggered in the last hour.

When triggered, the buzzer beeps in short pulses (200 ms on/off) for 10 seconds. Press SW0 again
to disable.

The 5-minute warm-up window means the alarm will not fire until at least 5 samples have
been collected after boot.

## Serial console

```
screen /dev/ttyACM0 115200
```

The log prints sensor readings and Zigbee events as they arrive.
