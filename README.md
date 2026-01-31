# FRC Team 9155 - 2026 Robot Code (Rebuilt)

This repository contains the code for the FRC 9155 robot for the 2026 season.

## ⚡ Hardware Specifications

### Swerve Drive (YAGSL)
* **Drive Motors:** NEO Brushless (controlled by Spark Max)
* **Angle Motors:** NEO Brushless (controlled by Spark Max)
* **Absolute Encoders:** CTRE CANCoders
* **Gyro (IMU):** NavX-MXP (SPI)

### Lift Mechanism
* **Motors:** 2x Brushed Motors (CIM / 775pro / Bag)
* **Controllers:** 2x REV Spark Max
* **Mode:** Leader-Follower (Brushed Mode)

---

## 💻 Software Installation Prerequisites

To deploy or edit this code, you must install the following software:

### 1. Game Tools (Required)
* **[WPILib 2026](https://github.com/wpilibsuite/allwpilib/releases)** - The core development environment (VS Code, Shuffleboard, Driver Station).
* **[NI FRC Game Tools 2026](https://www.ni.com/en-us/support/downloads/drivers/download.frc-game-tools.html)** - Drivers for the roboRIO and Driver Station.

### 2. Vendor Libraries (Already included in `vendordeps`, but tools are needed)
You need these tools to configure IDs and update firmware:

* **[REV Hardware Client](https://docs.revrobotics.com/rev-hardware-client/)**
    * *Used for:* Configuring Spark Max controllers (CAN IDs 1-10), setting Motor Type to Brushed/Brushless.
* **[Phoenix Tuner X](https://v6.docs.ctr-electronics.com/en/stable/docs/tuner/index.html)**
    * *Used for:* Configuring CANCoders (IDs 10-13).
* **[PathPlanner](https://pathplanner.dev/)**
    * *Used for:* Creating autonomous paths.

---

## 📦 Installed Vendor Dependencies

The project automatically downloads these libraries (defined in `vendordeps/`):

1.  **REVLib (2026.0.1)** - For Spark Max motor controllers.
2.  **YAGSL (2026.1.26)** - Swerve Drive logic.
3.  **Phoenix 6 & 5** - For CTRE devices (CANCoders).
4.  **PathPlannerLib** - For Autonomous routines.
5.  **ReduxLib, Studica, ThriftyLib** - Required for YAGSL.

---

## 🎮 Controller Inputs (Xbox Port 0)

* **Left Stick:** Drive 
* **Right Stick:** Rotate
* **Button X:** Lift Up
* **Button B:** Lift Down
* **Button A:** Reset Gyroscope 