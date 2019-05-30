# Overview
Apache Mynewt offers the world’s first fully open-source Bluetooth Low Energy (BLE) or Bluetooth Smart stack fully compliant with Bluetooth 5 specifications with support for Bluetooth Mesh. It is called NimBLE.

In this project, Nimble and FreeRTOS are combined for evaluation. Host runs on STM32 and controller runs on nRF51822. Two chips communicate with each other using UART. For now, this repository only contains code for host subsystem.