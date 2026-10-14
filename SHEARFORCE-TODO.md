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
- Camera test data:
  - Real distance: 24 in.	camera distance: 30 in.	difference: 6 in.	exposure: 1 ms	gain: 100		notes: worked great
    Real distance: 36 in.	camera distance: 41 in.	difference: 5 in.	exposure: 1 ms	gain: 100		notes: worked pretty good
    Real distance: 48 in.	camera distance: 51 in.	difference: 3 in.	exposure: 1 ms	gain: 100		notes: detected almost all of the time
    Real distance: 54 in.	camera distance: 57 in.	difference: 3 in.	exposure: 1 ms	gain: 100		notes: worked about half of the time
    Real distance: 57 in.	camera distance: 60 in.	difference: 3 in.	exposure: 1 ms	gain: 100		notes: rarely worked
    Real distance: 60 in.	exposure: 1 ms	gain: 100		notes: did not detect
    Real distance: 72 in.	exposure: 1 ms	gain: 100		notes: did not detect
    Real distance: 84 in.	exposure: 1 ms	gain: 100		notes: did not detect
    Stopped detecting it all the time at 48 in.

## ⚙️ Subsystems & Mechanisms ⚙️
- [ ] **Finish Tuning the Shooter mechanism**
    - [ ] Find the "F" value that makes the average velocity match the target velocity
    - [ ] Find the "P" value that minimizes the time to reach the target velocity from a slower speed without significantly overshooting
    - [ ] Update the defaults in ShooterConfig file with the new calculated values

## 📝 Reference Notes
* **Active Config:** Stored in `TeamCode/res/xml/`

