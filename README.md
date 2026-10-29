# Accelerometer Calibration Using Embedded Neural Networks

Bachelor's thesis completed at the University of Bologna in collaboration with [IRMA L@b](https://irmalab.org/), under the supervision of Prof. Edoardo Ida' and co-supervision of Matteo Dall'Olio.

The work investigates analytical accelerometer calibration, approximation with a compact neural network and deployment on an STM32 microcontroller. A separate Python study examines physically constrained learning and adaptation to a simulated change in sensor gain and bias.

**[Read the detailed technical report (PDF)](docs/Accelerometer_Calibration_Detailed_Report.pdf)**

## What this repository contains

This is a focused release of the embedded inference application and the thesis report. It includes the STM32CubeIDE project, SCL3300 sensor driver, analytical calibration coefficients, input standardisation coefficients, generated network code, trained weights and required vendor libraries.

It does not include training datasets, standalone Python training/adaptation scripts, the original Keras/ONNX models or historical acquisition logs. The report describes these studies and includes code listings in its appendices. This repository is not a complete reproduction package for the training and continual-learning experiments.

## Reported results

- A 3–10–3 supervised network uses 73 floating-point parameters, occupying 292 bytes for weights alone.
- On twelve static test poses, pose-mean norm RMSE decreased from 0.059047 m/s² for Factory conversion to 0.006681 m/s² for the network (88.68%). Analytical calibration retained a lower RMSE of 0.002028 m/s².
- On ten additional static poses, embedded network outputs achieved an overall component RMSE of 0.004481 m/s² relative to analytical calibration.
- Verification on 18,000 identical input triplets found a maximum STM32–Keras discrepancy of 0.000002338 m/s².

These metrics have different references: agreement with the analytical model, agreement between implementations and deviation from the gravitational norm are not interchangeable measures of absolute sensor accuracy.

## Repository contents

```text
accelerometer-calibration/
├── README.md
├── .gitignore
├── docs/
│   └── Accelerometer_Calibration_Detailed_Report.pdf
└── firmware/
    └── STM32_Project/
        ├── .project
        ├── .cproject
        ├── .settings/
        ├── Accelerometer_Calibration.ioc
        ├── STM32H743ZITX_FLASH.ld
        ├── Core/          Application, sensor driver and generated network
        ├── Drivers/       STM32 HAL and CMSIS dependencies
        ├── Middlewares/  ST neural-network runtime and headers
        └── LICENSE.txt
```

Vendor components and their licence files are retained. They are dependencies, not additional project examples or experimental results.

## Hardware and software

- STM32 NUCLEO-H743ZI2 board and Murata SCL3300 sensor hardware compatible with the experimental setup described in the report.
- STM32CubeIDE with an STM32 Arm GCC toolchain and ST-LINK support.
- A serial terminal or acquisition tool for the board's USART3 output.

The configuration references STM32Cube FW_H7 V1.12.1. The report documents model generation with STM32Cube AI Studio 1.3.0 and ST Edge AI Core 4.0.1. Generated inference code and the runtime are included, so model conversion is not part of the initial build procedure. No Python environment is required to run this firmware.

### Signal assignment in the supplied firmware

| Signal | MCU pin |
|---|---|
| SPI1 clock | PA5 |
| SPI1 MISO | PA6 |
| SPI1 MOSI | PB5 |
| Sensor chip select | PD14 |
| USART3 TX / RX | PD8 / PD9 |

Use the board connector mapping, sensor supply and grounding arrangement documented in the report and hardware documentation; MCU pin names are not connector pin numbers. The stored calibration coefficients and network weights correspond to the original sensor and acquisition campaign, not arbitrary SCL3300 devices.

## Import, build and run

1. Select **Code → Download ZIP** and extract the repository.
2. Open STM32CubeIDE with a workspace separate from the source directory.
3. Choose **File → Import → General → Existing Projects into Workspace**.
4. Select `firmware/STM32_Project` as the root directory and import `Accelerometer_Calibration`.
5. Preserve the existing project configuration for the initial build. Import the complete project, rather than generating a new project from the `.ioc` file: regeneration can change the manually integrated inference setup.
6. Select the **Debug** build configuration and use **Project → Build Project**.
7. Connect the documented hardware and use the IDE's ST-LINK debug/run workflow to program the board and start execution.
8. Open the appropriate serial port at **115200 baud, 8 data bits, no parity, 1 stop bit, no flow control**.

See the [STM32CubeIDE user guide](https://www.st.com/resource/en/user_manual/um2609-stm32cubeide-user-manual-stmicroelectronics.pdf) for project import and debugging. Preserve `.project`, `.cproject`, the linker script, drivers and AI runtime. A fresh build and hardware run should be checked in the receiving environment; packaging does not constitute a new board-level validation.

## Serial output

After initialisation, the application emits a CSV header followed by measurement rows:

```text
raw_x,raw_y,raw_z,ref_x,ref_y,ref_z,nn_x,nn_y,nn_z,max_abs_error,mean_abs_error
```

- `raw_*`: signed sensor counts.
- `ref_*`: analytical calibration outputs.
- `nn_*`: fixed-weight network outputs.
- `max_abs_error`, `mean_abs_error`: maximum and mean absolute network/reference differences across the three axes for that sample.

Calibrated outputs and discrepancy fields use the gravitationally normalised representation. Multiply these fields by 9.80665 to express them in m/s². The firmware does not update weights online; `tinyml_online` refers to inference during acquisition.

## Scope and future work

The embedded network approximates analytical calibration without demonstrating an advantage over direct evaluation of that correction. The results concern static orientations, with no independent absolute angular reference. The Python adaptation study in the report is separate from this firmware and is not on-device continual learning. Dynamic measurements, temperature effects, independent angular validation and embedded weight updates remain further work.

## Third-party components

STMicroelectronics and Arm components retain their original notices and licence files in the firmware tree. The ST AI runtime is supplied under its included SLA0104 terms. No blanket licence is applied to the entire repository; refer to the notices accompanying each component.

## Author

Francesco De Tursi — MSc student in Mechanical Engineering, University of Bologna.

[LinkedIn](https://www.linkedin.com/in/francescodetursi/) · [GitHub](https://github.com/francesco-detursi) · [Email](mailto:ffrancesco.detursi@gmail.com)
