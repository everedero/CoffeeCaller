# Temperature and humidity sensor on CoffeeCaller

## Build board

### temp_ble (BLE ESS with onboard SHT4x) — NCS v3.2.1 workspace

```bash
# From /home/eve/ncs/
west build -b coffeecaller_nrf52/nrf52840 ./CoffeeCaller/applications/temp_ble -p always
```

### zigbee_temp (Zigbee coordinator + BLE ESS gateway) — NCS v2.8.0 workspace

NCS v2.8.0 is the last NCS release with the ZBOSS Zigbee stack (nrfxlib).

```bash
cd /home/eve/ncs/v2.8.0
west build -b coffeecaller_nrf52/nrf52840 \
  /home/eve/ncs/CoffeeCaller/applications/zigbee_temp \
  -p always -- -DZEPHYR_EXTRA_MODULES=/home/eve/ncs/CoffeeCaller
```

## Check board presence

> lsusb
1915:c0ca Nordic Semiconductor ASA CoffeeCaller nRF52 (CDC ACM)

## Communicate with the board

screen /dev/ttyACM0 115200

## Flashing the board

### With UF2 (invalid for now)
If UF2 bootloader present

west flash -r uf2

### With FTDI2232 mini module

Using a FT2232 FTDI in SWO mode (270R resistor hack)
FT2232 pin	SWDIO  Connect to target
ADBUS0	SWCLK	SWCLK
ADBUS1	SWDIO (through ~470 Ω resistor)	SWDIO
ADBUS2	SWDIO (directly)	SWDIO
ADBUS3	Not used
ADBUS4	nSRST (optional, not connected)	nRESET

#### Check for FTDI 2232 presence

> lsusb
ID 0403:6010 Future Technology Devices International, Ltd FT2232C/D/H Dual UART/FIFO IC

#### Flash

west flash --runner openocd

#### Note

If flashing failed, make sure the target openocd script is set up to use "ftdi" and not "jlink".
