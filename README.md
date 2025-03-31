# nRF52810
Basic HW and FW to setup nRF developer environment

# Instructions

1. Clone the repository:
```bash
git clone <repository-url>
```

2. Build the Docker image (requires sudo permissions):
```bash
sudo ./docker_build.sh
```
3. Start the Docker container:

```bash
./docker_start.sh
```

4. Enter the project root folder (you should already be in the correct directory).

5. Initialize the West workspace:
```bash
west init -l app
```

6. Update west modules (this step can take a few minutes)
```bash
west update
```

7. Export Zephyr environment:
```bash
west zephyr-export
```

8. Build the application for the nRF52840 DK:
```bash
west build -b nrf52840dk/nrf52840 app
```


## Release notes

### Version: v2.0-test phase
Changelog:
- 2.4 GHz antenna design
- LED for blinky funtionality
- Plug of nails changed to pluggable
- Optical sensor added
- 3V3 and 1V9 power rails
- KiCAD library commit: f8f82b3228b9b2e6a28c75499dfac7be01fe0e7f

### Version: v1.0
Changelog:
- Minimal env. for bringing an nRF52810 up
- LED for blinky functionality
- Tactile button for user input
- LDO power adjustment
- Plug of nails footprint faulty              
