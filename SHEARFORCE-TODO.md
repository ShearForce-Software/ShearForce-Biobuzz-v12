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
- [ ] Test and verify teleOp knows the correct alliance after autonomous
- [ ] Add manual ability to change alliance from gamepad1
- [ ] Test and verify teleOp knows correct position and heading after autonomous
- [ ] Create MVP+ routes to shoot 4 balls before parking
- [ ] Create MVP++ routes to shoot 4 balls and intake from garden or flower then shoot again and park
- [ ] Create Full Plan 1 RED - Audience route
- [ ] Create Full Plan 1 RED - TV side route
- [ ] Create Full Plan 2 RED - Audience route
- [ ] Create Full Plan 2 RED - TV side route
- [ ] Create Full Plan 3 RED - Audience route
- [ ] Create Full Plan 3 RED - TV side route
- [ ] Create Full Plan 1 BLUE - Audience and TV side routes
- [ ] Create Full Plan 2 BLUE - Audience and TV side routes
- [ ] Create Full Plan 3 BLUE - Audience and TV side routes

## ⚙️ Subsystems & Mechanisms ⚙️
- [ ] **Intake Subsystem (and De-Pollinator)**
  - [✅] Test depollinator control, determine optimum up/down values for servo
  - [✅] Verify ramp sensors working to detect presence of a ball (nectar or pollen)
  - [ ] Verify ramp sensors working to detect difference between Nectar versus Pollen
  - [ ] Add support and logic for **NEW** intake gate servos (logic should be in a new intakeSubsystem.Update() method)
    - if front-ramp ball detected and < 4 balls in transfer
      - if correct color, then open front gate, close back gate
      - else if wrong color then eject intake and close both gates
    - else if back-ramp ball detected and < 4 balls in transfer
      - if correct color, then open back gate, close front gate
      - else if wrong color then eject intake and close both gates
    - else if front-ramp ball detected OR back-ramp ball detected AND 4 balls already in transfer
      - then eject intake, close both gates
   - [ ] Update the de-pollinator up position to be tighter to the chassis
- [ ] **Transfer Subsystem**
  - [✅] Update color sensor logic to not update while the transfer subsystem is running (stop false arm detects)
  - [✅] Test encoder to determine correct tick/revolution value
  - [✅] Test ability to move forward or reverse 1 hole spot
  - [✅] Test ability to detect presence of a ball (nectar or pollen) in each slot
  - [✅] Test ability to detect difference of color between Nectar or Pollen in each slot
  - [ ] **Add PIDF tuning for transfer system rotate CRServo**
  - [ ] Test and verify ability to move **exactly** 1 slot without overshooting
  - [ ] Move auto transfer logic from BiobuzzTeleOp to TransferSubsystem.update() method so can be reused in autonomous routes
  - [ ] Add logic to auto switch Nectar/Pollen servo to the correct position based on ball color in each slot
    - For mixed loads will need to auto shoot 1 at a time probably to allow time to switch servo positions 
- [ ] **SHOOTER Subsystem**
  - [✅] **Fix Shooter Nectar/Pollen Servo in BiobuzzTeleOp**
  - [ ] Tune shooter nectar/pollen servo positions after next redesign
  - [ ] Determine best velocities and hood angles for several distances
    - See if can determine an optimum "curve" algorithm to use for shooting based on distance
      - https://mycurvefit.com/
      - Reference: https://www.youtube.com/watch?v=Ap1lBywv00M (aprilTag distance algorithm but relevant method)
      - **DOCUMENT this in the log**
  - [ ] Implement curve algorithm for auto setting shooting speeds and hood angles based on distance to target
  - [ ] Integrate turret rotation motor and controls
  - [ ] Implement a PIDF tuner for the turret rotation motor
  - [ ] Implement an auto tracker capability to always have the turret pointed at the target
    - will need special logic to know when to switch to the other target (have to use the camera to know) 
- [ ] **Intake + Transfer Combo Testing**
  - [✅] Add ability to auto advance transfer subsystem to next slot when intake is on and ball detected in bottom spot
  - [✅] Add ability to auto turn off intake when all slots are filled
  - [ ] Test and verify wrong color Nectar never slips in
  - [ ] Test and verify never hold more than 4 game elements
- [ ] **Intake + Transfer + Shooter Combo Testing**


## 📷 Vision & Localization 📷
- [ ] **Verify camera is working on new BIOBUZZ robot**
  - Using the biobuzz.xml config file and the CameraTunerOpMode
  - May need to update biobuzz.xml config file for the actual camera ID**
- [ ] Add update logic to TeleOp to periodically update the Hive Up/Down info without introducing lag to the system
- [ ] Create Test OpMode to calculate distance to target based on aprilTab size
  - Watch YouTube video explaining concept:  https://www.youtube.com/watch?v=Ap1lBywv00M
  - If works, then incorporate calculation into CameraSubsystem
- [ ] Add methods to utilize blob detection to orient robot towards biggest piles of pollen in auto

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


