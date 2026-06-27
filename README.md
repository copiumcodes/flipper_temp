# DS18B20 Temp (Flipper Zero)

The Summer-heat is hitting and I wanted to know how hot it is in my room, so I made a little app for my Flipper Zero.

![DS18B20 Temp app screenshot](images/Screenshot.png)

## Features

- Large main temperature display with one decimal place
- Secondary page with short pinout + logging controls
- CSV logging to SD card (`/ext/temp_log.csv`)

## Wiring

Use a 4.7k pull-up resistor between DQ and 3.3V.

- DS18B20 pin 1 (GND) -> Flipper GND
- DS18B20 pin 2 (DQ) -> Flipper A7
- DS18B20 pin 3 (VCC) -> Flipper 3.3V
- 4.7k resistor: DQ -> 3.3V

## App Controls

Main page:

- Right: open pinout/logging page
- Back: exit app

Pinout + logging page:

- Left: back to main page
- OK: toggle periodic logging on/off
- Right: append one sample manually

## Log Output

CSV file path:

- `/ext/temp_log.csv`

Format:

```csv
uptime_s,temperature_c
123,24.50
124,24.56
```
