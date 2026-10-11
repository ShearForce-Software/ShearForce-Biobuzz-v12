# 🐏 🤖 ShearForce Software TO-DO Tasks 🤖 🐏

This file tracks the upcoming software tasks.

*Check off items as they are completed and tested on the robot.*

## 🚀 Drive System and Auto Routing 🚀
- [ ] **Finish Tuning PedroPathing**
- [ ] **Test PedroPathing example routes**
    - [ ] Verify basic path following works as expected

## 📷 Vision & Localization 📷
- [ ] **Test camera April Tag localization distances**
    - [ ] Measure the distance from likely starting position on wall to closest April Tag Cluster and to the April Tag on the opposite side of the hive
    - [ ] Measure accuracy at 4 feet, 6 feet, 7 feet, 10 feet, and 12 feet.
    - [ ] Document the maximum reliable detection threshold (in code and in log book)
- [ ] **Add methods to return bearing and distance to closest April Tag cluster**
- [ ] **Add methods to return state of hive (Audience up/down)**
- [ ] **Add methods to utilize blob detection to orient robot towards biggest piles of pollen**

## ⚙️ Subsystems & Mechanisms ⚙️
- [ ] **Finish Tuning the Shooter mechanism**
    - [ ] Find the "F" value that makes the average velocity match the target velocity
    - [ ] Find the "P" value that minimizes the time to reach the target velocity from a slower speed without significantly overshooting
    - [ ] Update the defaults in ShooterConfig file with the new calculated values

## 📝 Reference Notes
* **Active Config:** Stored in `TeamCode/res/xml/`

