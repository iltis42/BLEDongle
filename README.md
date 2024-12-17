| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- |

# NimBLE UART Bridge

## Overview

This project is derived from NimBLE Connection Example and realizes a UART to BLE Bridghe with autobaud feature

## Try It Yourself

### Set Target

Before project configuration and build, be sure to set the correct chip target using:

``` shell
idf.py set-target <chip_name>
```

For example, if you're using ESP32-C3, then input

``` Shell
idf.py set-target esp32c3
```

### Build and Flash

Run the following command to build, flash and monitor the project.

``` Shell
idf.py -p <PORT> flash monitor
```

For example, if the corresponding serial port is `/dev/ttyACM0`, then it goes

``` Shell
idf.py -p /dev/ttyACM0 flash monitor
```

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://idf.espressif.com/) for full steps to configure and use ESP-IDF to build projects.

## Code Explained

### Overview

1. Initialization
    1. Initialize NVS flash, NimBLE host stack, GAP service, UART
    2. Initialize GATT service and add nordic serial services to registration queue
    3. Configure NimBLE host stack and start NimBLE host task thread, GATT services will be registered automatically when NimBLE host stack started
    4. Start heart rate update task thread
2. Wait for NimBLE host stack to sync with BLE controller, and start advertising; wait for connection event to come
3. After connection established, wait for GATT characteristics access events to come
    2. On incoming serial data, this data is 1:1 forwared to BLE 
    3. On indication of BLE data, this data is forwarded to UART
  
   

