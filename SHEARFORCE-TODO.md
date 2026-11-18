# 🐏 🤖 ShearForce Software TO-DO Tasks 🤖 🐏

This file tracks the upcoming software tasks.

*Check off ✅ items as they are completed and tested on the robot.*

## 🚀 Drive System, TeleOp, and Auto Routing 🚀
- [✅] Update biobuzz.xml with actual locations things are plugged in
- [✅] Tune new BIOBUZZ robot for PedroPathing
- [✅] Test PedroPathing example routes
- [✅] **Move Color sensor logic to separate thread to minimize latency**
- [✅] **Reduce latency of telemetry messages by slowing them down**
- [ ] **Test BiobuzzTeleOp for Robot Centric manual drive controls for > 4 minutes**
- [ ] Test BiobuzzTeleOp for Field Centric manual drive controls for > 4 minutes
- [✅] Update names of existing Auto routes to show color, plan set (1,2, or 3), and start position
- [ ] Save all autonomous routes in turtle format
- [ ] **Create and Verify MVP parking route for all 4 positions**
- [ ] Create Plan 1 RED - Audience route
- [ ] Create Plan 1 RED - TV side route
- [ ] Create Plan 2 RED - Audience route
- [ ] Create Plan 2 RED - TV side route
- [ ] Create Plan 3 RED - Audience route
- [ ] Create Plan 3 RED - TV side route
- [ ] Create Plan 1 BLUE - Audience and TV side routes
- [ ] Create Plan 2 BLUE - Audience and TV side routes
- [ ] Create Plan 3 BLUE - Audience and TV side routes

## ⚙️ Subsystems & Mechanisms ⚙️
- [ ] **Intake Testing**
  - [✅] Test depollinator control, determine optimum up/down values for servo
  - [✅] Verify ramp sensors working to detect presence of a ball (nectar or pollen)
  - [ ] Verify ramp sensors working to detect difference between Nectar versus Pollen
  - [ ] Add ability to detect wrong color Nectar on ramp and auto eject it
  - [ ] Update the de-pollinator up position to be tighter to the chassis
- [ ] **Transfer System Testing**
  - [ ] **Add PIDF tuning for transfer system rotate CRServo**
  - [ ] Move auto transfer logic from BiobuzzTeleOp to TransferSubsystem so can be reused in autonomous routes
  - [ ] Update color sensor logic to not update while the transfer subsystem is running (stop false arm detects)
  - [✅] Test encoder to determine correct tick/revolution value
  - [✅] Test ability to move forward or reverse 1 hole spot
  - [✅] Test ability to detect presence of a ball (nectar or pollen) in each slot
  - [✅] Test ability to detect difference of color between Nectar or Pollen in each slot
- [ ] **Intake + Transfer Combo Testing**
  - [✅] Add ability to auto advance transfer subsystem to next slot when intake is on and ball detected in bottom spot
  - [✅] Add ability to auto turn off intake when all slots are filled
  - [ ] Test and verify wrong color Nectar never slips in
  - [ ] Test and verify never hold more than 4 game elements
- [ ] **Intake + Transfer + Shooter Combo Testing**
  - [✅] **Fix Shooter Nectar/Pollen Servo in BiobuzzTeleOp**
  - Test shooter on actual field
  - Determine best velocities for several distances
  - See if can determine an optimum "curve" algorithm to use for shooting based on distance
    - https://mycurvefit.com/
    - Reference: https://www.youtube.com/watch?v=Ap1lBywv00M (aprilTag distance algorithm but relevant method)

## 📷 Vision & Localization 📷
- [ ] Add methods to utilize blob detection to orient robot towards biggest piles of pollen
- [ ] **Verify camera is working on new BIOBUZZ robot**
  - Using the biobuzz.xml config file and the CameraTunerOpMode
  - May need to update biobuzz.xml config file for the actual camera ID**
- [ ] Create Test OpMode to calculate distance to target based on aprilTab size
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


## 📝 Reference Notes
* **Active Config:** Stored in `TeamCode/res/xml/biobuzz.xml`
* **Strategy Notes about auto routes (pictures):** Stored in `Strategy-Notes/`


