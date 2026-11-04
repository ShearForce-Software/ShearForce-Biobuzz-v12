# 🐏 🤖 ShearForce Software TO-DO Tasks 🤖 🐏

This file tracks the upcoming software tasks.

*Check off items as they are completed and tested on the robot.*

## 🚀 Drive System, TeleOp, and Auto Routing 🚀
- [X] **Update biobuzz.xml with actual locations things are plugged in**
  - Verify _ExampleRedAuto can be initialized without crashing (it has all subsystems defined in it)
- [ ] **Tune new BIOBUZZ robot for PedroPathing**
- [ ] **Test PedroPathing example routes**
    - Verify basic path following works as expected
- [ ] **Test BiobuzzTeleOp for basic manual drive controls**
- [ ] **Update names of existing Auto routes to show color, plan set (1,2, or 3), and start position**
  - make similar to "red_plan3_Audience_side" and "red_plan3_TV_side" style naming
- [ ] **Create Plan 1 RED - Audience route**
- [ ] **Create Plan 1 RED - TV side route**
- [ ] **Create Plan 2 RED - Audience route**
- [ ] **Create Plan 2 RED - TV side route**
- [ ] **Create Plan 3 RED - Audience route**
- [ ] **Create Plan 3 RED - TV side route**
- [ ] **Create Plan 1 BLUE - Audience and TV side routes**
- [ ] **Create Plan 2 BLUE - Audience and TV side routes**
- [ ] **Create Plan 3 BLUE - Audience and TV side routes**
- [ ] **Update Subsystem gamepad controls in BiobuzzTeleOp as needed**
- [ ] **Update subsystem and sensor telemetry messages to BioBuzzTeleOp as needed**

## 📷 Vision & Localization 📷
- [ ] **Add methods to utilize blob detection to orient robot towards biggest piles of pollen**
- [ ] **Verify camera is working on new BIOBUZZ robot** 
  - Using the biobuzz.xml config file and the CameraTunerOpMode
  - May need to update biobuzz.xml config file for the actual camera ID**
- [ ] **Create Test OpMode to calculate distance to target based on aprilTab size**
  - Watch YouTube video explaining concept:  https://www.youtube.com/watch?v=Ap1lBywv00M
  - If works, then incorporate calculation into CameraSubsystem

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
- [ ] **Intake Testing**
  - [ ] Determine optimum speed for intake motor
  - [ ] Test depollinator control, determine optimum up/down values for servo
  - [ ] Verify ramp sensors working to detect presence of a ball (nectar or pollen)
  - [ ] Verify ramp sensors working to detect difference between Nectar versus Pollen
  - [ ] Add ability to detect wrong color Nectar on ramp and auto eject it
- [ ] **Transfer System Testing**
  - [ ] Test encoder to determine correct tick/revolution value
  - [ ] Test ability to move forward or reverse 1 hole spot
  - [ ] Test ability to detect presence of a ball (nectar or pollen) in each slot
  - [ ] Test ability to detect difference of color between Nectar or Pollen in each slot
- [ ] **Intake + Transfer Combo Testing**
  - [ ] Add ability to auto advance transfer subsystem to next slot when intake is on and ball detected in bottom spot
  - [ ] Add ability to auto turn off intake when all slots are filled
  - [ ] Test and verify wrong color Nectar never slips in
  - [ ] Test and verify never hold more than 4 game elements
- [ ] **Intake + Transfer + Shooter Combo Testing**
  - Test shooter on actual field
  - Determine best velocities for several distances
  - See if can determine an optimum "curve" algorithm to use for shooting based on distance
    - https://mycurvefit.com/
    - Reference: https://www.youtube.com/watch?v=Ap1lBywv00M (aprilTag distance algorithm but relevant method)

## 📝 Reference Notes
* **Active Config:** Stored in `TeamCode/res/xml/biobuzz.xml`
* **Strategy Notes about auto routes (pictures):** Stored in `Strategy-Notes/`


