# 🐏 🤖 ShearForce Software TO-DO Tasks 🤖 🐏

This file tracks the upcoming software tasks.

*Check off items as they are completed and tested on the robot.*

## 🚀 Drive System and Auto Routing 🚀
- [✅] **Finish Tuning PedroPathing**
- [ ] **Test PedroPathing example routes**
    - [ ] Verify basic path following works as expected

## 📷 Vision & Localization 📷
- [ ] **Figure out why Camera isn't working well anymore**
  - See if the knurled lockring on the camera lens barrel can be manually adjusted to focus the camera while a monitor is hooked up, optimize for 7 feet
  - Additional changes have been made that need to be tested:
    - Resolution changed to 1280x800
    - Decimation changed to 1.0
    - teamwebcamcalibrations.xml has been updated
- [ ] **Test camera April Tag localization distances**
    - [ ] Measure the distance from likely starting position on wall to closest April Tag Cluster and to the April Tag on the opposite side of the hive
    - [ ] Measure accuracy at 4 feet, 6 feet, 7 feet, 10 feet, and 12 feet.
    - [ ] Document the maximum reliable detection threshold (in code and in log book)
- [ ] **Add methods to return bearing and distance to closest April Tag cluster**
- [ ] **Add methods to return state of hive (Audience up/down)**
- [ ] **Add methods to utilize blob detection to orient robot towards biggest piles of pollen**

### 📷 Camera Distance Test Data 📷

| Real Distance | Camera Distance | Difference | Exposure | Gain | Notes |
| :---: | :---: | :---: | :---: | :---: | :--- |
| 24 in. | 30 in. | 6 in. | 1 ms | 100 | worked great |
| 36 in. | 41 in. | 5 in. | 1 ms | 100 | worked pretty good |
| 48 in. | 51 in. | 3 in. | 1 ms | 100 | detected almost all of the time |
| 54 in. | 57 in. | 3 in. | 1 ms | 100 | worked about half of the time |
| 57 in. | 60 in. | 3 in. | 1 ms | 100 | rarely worked |
| 60 in. | N/A | N/A | 1 ms | 100 | did not detect |
| 72 in. | N/A | N/A | 1 ms | 100 | did not detect |
| 84 in. | N/A | N/A | 1 ms | 100 | did not detect |

*Note: Consistent detection threshold cuts off around 48 in.*

## ⚙️ Subsystems & Mechanisms ⚙️
- [✅] **Finish Tuning the Shooter mechanism**
    - Find the "F" value that makes the average velocity match the target velocity
    - Find the "P" value that minimizes the time to reach the target velocity from a slower speed without significantly overshooting
    - Update the defaults in ShooterConfig file with the new calculated values
- [ ] **Develop Intake Testing OpMode**
  - Need to be able to control speed of intake motor
  - Need to be able to control direction of intake motor
  - Need to be able to command a servo (flower ramp thing)
  - Need to be able to adjust the servo up position and down position
  - Needs telemetry showing all of those values, and informing user on GamePad Controls to use
  - Needs an XML config file stored in code
- [ ] **Develop Transfer System Testing OpMode**
  - Needs a subsystem and config file created in same style as other subsystems
  - Needs to utilize a Continuous Servo (control direction and speed)
  - Needs to support an external encoder to track position of transfer device 
    - gamepad controls in test opMode to adjust ticks per degree
    - telemetry showing current position
  - Needs to support 4 color sensors
    - telemetry showing color of each sensor and if a ball is present and detected distance of detected ball
    - gamepad controls can adjust minimum and maximum distance to detect a ball is present

## 📝 Reference Notes
* **Active Config:** Stored in `TeamCode/res/xml/`

