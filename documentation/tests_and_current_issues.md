# Test Description: System Verification

Full playlist of all 40 uploaded test videos: https://www.youtube.com/watch?v=alZEPmOZxNE&list=PLyqti-zvsEaH48GBObaMNQcK6wwJ_p7y3&index=1

## Table of Contents

1. **[Test ID 1: Manipulator System](#test-id-1)**
    - *Recommended:* [Video 8: Tag-Relative Manipulator Movements (Map Frame) 2](https://www.youtube.com/watch?v=zHVklbptjsk)
    - *Recommended:* [Video 10: Manipulator Recording & Playback (Odom Frame)](https://www.youtube.com/watch?v=6oHCDRIGcXg)
2. **[Test ID 2: Base Movement](#test-id-2)**
3. **[Test ID 3: Mapping & Localization](#test-id-3)**
    - *Recommended:* [Video 27: Manual Map Building and Refinement](https://youtu.be/c8lwK_ZCB3M)
    - *Recommended:* [Video 28: Localization, Waypoints, and Obstacle Avoidance](https://youtu.be/kJ5P2cTgtOM)
4. **[Test ID 4: Full System Integration](#test-id-4)**
    - *ESSENTIAL:* [Video 33: Full System Validation](https://youtu.be/pEeT_Krulio)
    - *ESSENTIAL:* [Video 33b: Sequence Recording and Reliability](https://youtu.be/EIX_GhtDRFI)
5. **[Issues Found While Testing](#issues-found-while-testing-anomalies--todos)**

---

## Test ID: 1

## Overview

Comprehensive verification of the manipulator system, including basic state commands, complex recording/playback scenarios, combination commands, and relative
movements across multiple reference frames (Body, Odometry, and Map).

## Verified Functions

| Category              | Function             | Frame | Description                                                    | Result | Evidence   |
|-----------------------|----------------------|-------|----------------------------------------------------------------|--------|------------|
| **Basic Controls**    | Ready / Stow / Reset | N/A   | Verifies state transitions (Operational, Stowed, Default)      | ✅ Pass | Video 1    |
| **Basic Controls**    | Gripper Toggle       | N/A   | Verifies open/close functionality                              | ✅ Pass | Video 1    |
| **Relative Move**     | Directional Move     | Body  | Verifies movement in all axes (flat_body frame)                | ✅ Pass | Video 2, 3 |
| **Relative Move**     | Orientation Offsets  | Body  | Verifies orientation adjustments (flat_body frame)             | ✅ Pass | Video 2, 3 |
| **Relative Move**     | Orientation Modes    | Body  | Verifies specific orientation mode resets                      | ❌ Fail | Video 3    |
| **Relative Move**     | Directional Move     | Odom  | Verifies movement in all axes (odometry frame)                 | ✅ Pass | Video 4    |
| **Relative Move**     | Orientation Offsets  | Odom  | Verifies orientation adjustments (odometry frame)              | ✅ Pass | Video 4    |
| **Tag-Relative Move** | Directional Move     | Body  | Verifies movement relative to tag, aligned to robot chassis    | ✅ Pass | Video 5    |
| **Tag-Relative Move** | Directional Move     | Odom  | Verifies movement relative to tag, aligned to odom grid        | ✅ Pass | Video 6    |
| **Tag-Relative Move** | Directional Move     | Map   | Verifies movement relative to detected tag in Map frame        | ✅ Pass | Video 7, 8 |
| **Tag-Relative Move** | Orientation Offsets  | Map   | Verifies orientation adjustments in Map frame                  | ✅ Pass | Video 7, 8 |
| **Automation**        | Record & Playback    | Body  | Verifies playback of mixed tag-relative and relative movements | ✅ Pass | Video 9    |
| **Automation**        | Record & Playback    | Odom  | Verifies playback consistency from different robot positions   | ✅ Pass | Video 10   |
| **Automation**        | Emergency Stop       | N/A   | Verifies safety interrupt during playback                      | ❌ Fail | Video 12   |
| **Combination**       | Scan All In Range    | N/A   | Verifies sequential scanning and dynamic filtering of tags     | ✅ Pass | Video 11   |

## Observations & Notes

- **Unreachable Orientations (Video 2, 4:00):** If an orientation target is unreachable, the arm enters an error state. This is expected behavior; stowing the
  arm or navigating to a reachable state successfully resets the system.
- **Orientation Mode Failure:** The system fails to support non-custom orientation modes during relative movements. Regardless of the mode selected, the system
  applies the orientation as if it were in "custom" mode.
- **Odometry Scope:** Orientation mode testing was excluded from the Odometry frame tests (Video 4) due to the known failure identified during the Body frame
  tests.
- **Map Frame Visual Reference (Videos 7 & 8):** In the top-right map view, the colored points indicate position relative to the map frame (Red: Up/Down, Green:
  Left/Right).
- **Playback Robustness (Video 10):** Recording in the Odometry frame is highly robust. The system correctly executes the recorded path regardless of the
  robot's starting position or orientation.
- **Scan Filtering (Video 11):** The `scan_all_in_range` command successfully dynamically filters targets. If a tag is moved out of physical range or view
  during the sequence, the system correctly skips it.
- **CRITICAL SAFETY ISSUE (Video 12):** The Emergency Stop behavior was mixed. It successfully cleared the command buffer and stopped the overall playback
  sequence (preventing future commands from issuing). However, it **failed to immediately halt the currently executing manipulation movement**. The arm
  continued to finish its current trajectory rather than stopping instantly.

## Test Result

**Status: PARTIAL FAIL** ❌

While basic commands, complex recording features, and directional movements function correctly, the test is marked as a failure due to two significant issues:

1. Incorrect implementation of non-custom orientation modes.
2. **Safety Latency:** The Emergency Stop does not instantly arrest the active arm movement, despite correctly canceling the queue.

## Evidence (Videos)

- **Video 1:** [Basic Commands & Playback](https://www.youtube.com/watch?v=alZEPmOZxNE)
- **Video 2:** [Spot Relative Manipulator  Movements 2](https://www.youtube.com/watch?v=KdT4qz2j12c)
- **Video 3:** [Spot Relative Manipulator  Movements](https://www.youtube.com/watch?v=gIyWvL-bvts)
- **Video 4:** [Relative Manipulator Movements (Odometry Frame)](https://www.youtube.com/watch?v=mhiu-o7ZIrc)
- **Video 5:** [Tag-Relative Manipulator Movements (Body Frame)](https://www.youtube.com/watch?v=mUfcMAlV1Gk)
- **Video 6:** [Tag-Relative Movements (Odom Frame)](https://www.youtube.com/watch?v=914p4vLLi4A)
- **Video 7:** [Tag-Relative Manipulator Movements (Map Frame)](https://www.youtube.com/watch?v=G-N23ofvmuw)
- ***Video 8:** [Tag-Relative Manipulator Movements (Map Frame) 2 -- Recommended Watch](https://www.youtube.com/watch?v=zHVklbptjsk)
- **Video 9:** [Manipulator Recording & Playback (Body)](https://www.youtube.com/watch?v=aqEQCNawdbc)
- ***Video 10:** [Manipulator Recording & Playback (Odom Frame) -- Recommended Watch](https://www.youtube.com/watch?v=6oHCDRIGcXg)
- **Video 11:** [Scan All In Range Command](https://www.youtube.com/watch?v=Xi_xJR1xhAs)
- **Video 12:** [Manipulator Emergency Stop during Playback](https://www.youtube.com/watch?v=pgF93NtzF0g)

---

## Test ID: 2

## Overview

Verification of the Base Movement subsystem. This includes frame-specific movements (Body, Odom, Map), Tag-Relative base movements, and base-only
recording/playback.

## Verified Functions

| Category              | Function            | Frame | Description                                                | Result | Evidence         |
|-----------------------|---------------------|-------|------------------------------------------------------------|--------|------------------|
| **Safety**            | Collision Avoidance | Body  | Verifies halt on obstacle detection during relative move   | ✅ Pass | Video 13         |
| **Safety**            | Emergency Stop      | Odom  | Verifies immediate halt of base motion during movement     | ✅ Pass | Video 15         |
| **Relative Move**     | Directional Move    | Body  | Verifies movement aligned to robot chassis                 | ✅ Pass | Video 16         |
| **Relative Move**     | Directional Move    | Odom  | Verifies movement aligned to fixed startup grid            | ✅ Pass | Video 15         |
| **Relative Move**     | Directional Move    | Map   | Verifies movement aligned to persistent global coordinates | ✅ Pass | Video 17         |
| **Relative Move**     | Map Frame Update    | Map   | Verifies coordinate system update after `SWAP_MAP`         | ✅ Pass | Video 18         |
| **Relative Move**     | Obstacle Nav        | Body  | Verifies local planner pathing around small obstacles      | ✅ Pass | Video 14         |
| **Tag-Relative Move** | Directional Move    | Body  | Verifies approach to tag aligned to chassis                | ✅ Pass | Video 19, 21     |
| **Tag-Relative Move** | Directional Move    | Odom  | Verifies approach to tag aligned to fixed grid             | ✅ Pass | Video 19, 20, 21 |
| **Tag-Relative Move** | Directional Move    | Map   | Verifies approach to tag aligned to global map             | ✅ Pass | Video 20, 21     |
| **Automation**        | Record & Playback   | N/A   | Verifies relative base movement recording                  | ✅ Pass | Video 22         |
| **Automation**        | Record & Playback   | Odom  | Verifies consistent cardinal direction playback            | ✅ Pass | Video 23         |
| **Automation**        | Record & Playback   | Tag   | Verifies playback relative to dynamic tag position         | ✅ Pass | Video 24         |

## Observations & Notes

- **Persistent Obstacle Memory (Video 14):** The local planner exhibits a "memory" of obstacles. Even after passing an impediment, the robot may move cautiously
  or hesitate, though final accuracy is maintained.
- **Map Frame Consistency (Video 18):** Swapping maps correctly re-orients the coordinate system. A move command yields different physical paths depending on
  the loaded map's origin, confirming correct frame handling.
- **Vertical Tag Handling (Video 21):** The system successfully handles vertical tags (e.g., on walls), validating inspection capabilities for non-floor
  targets.
- **Small Tag Variance (Video 24):** Recording tag-relative moves with small markers works, but pose estimation fluctuations can cause minor shifts in the final
  position.

## Test Result

**Status: PASS** ✅

The Base Movement system functions reliably across all coordinate frames. Safety features (E-Stop) work correctly for the base, and frame updates upon map swaps
are consistent.

## Evidence (Videos)

- **Video 13:** [Move Base by Offset - Collision Avoidance](https://www.youtube.com/watch?v=kmIGajR4FEQ)
- **Video 14:** [Base Relative Navigation (Small Obstacle Avoidance)](https://youtu.be/tKo_FRbDPDU)
- **Video 15:** [Base Relative Movement (Odom Frame)](https://youtu.be/qOGxr4mBnEk)
- **Video 16:** [Base Relative Movement (Body Frame)](https://youtu.be/Ys1Ek5TvNAw)
- **Video 17:** [Base Relative Movement (Map Frame)](https://youtu.be/5fGmqwxZITc)
- **Video 18:** [Base Relative Movement: Changing Maps](https://youtu.be/HS-zb3hkU5I)
- **Video 19:** [Tag-Relative Base Movements (Body & Odom Frames)](https://youtu.be/YKUhEbXovGw)
- **Video 20:** [Tag-Relative Base Movements (Odom & Map Frames)](https://youtu.be/2_7tFcaCzvM)
- **Video 21:** [Tag-Relative Base Movements: Vertical Tag](https://youtu.be/xUuFxPpCQqo)
- **Video 22:** [Recording Relative Base Movements (Short)](https://youtu.be/0jNru3_U9Bo)
- **Video 23:** [Recording Relative Base Movements (Odom Frame)](https://youtu.be/qaM7J8g8fFE)
- **Video 24:** [Recording Tag-Relative Base Movements](https://youtu.be/I0vtRGLE6p8)

---

## Test ID: 3

## Overview

Verification of the Mapping and Localization subsystems. This includes map creation, expansion, waypoints, and relocalization stability.

## Verified Functions

| Category         | Function             | Frame | Description                                                | Result     | Evidence |
|------------------|----------------------|-------|------------------------------------------------------------|------------|----------|
| **Mapping**      | Load/Expand Map      | N/A   | Verifies loading existing map and adding new data          | ✅ Pass     | Video 26 |
| **Mapping**      | Build Map            | N/A   | Verifies manual map creation and loop closure              | ✅ Pass     | Video 27 |
| **Mapping**      | Stability            | N/A   | Verifies long-term mapping stability                       | ❌ Fail     | Video 35 |
| **Localization** | Custom Waypoints     | Map   | Verifies adding and navigating to user-defined points      | ✅ Pass     | Video 28 |
| **Localization** | Locomotion Stability | N/A   | Verifies localization hold during high/crawl walking modes | ✅ Pass     | Video 29 |
| **Localization** | Relocalization       | N/A   | Verifies recovery from lost state via Landmarks            | ⚠️ Partial | Video 31 |
| **Localization** | Self-Relocalization  | N/A   | Verifies recovery from lost state via Visual features only | ✅ Pass     | Video 34 |

## Observations & Notes

- **Mapping Refinement (Video 27):** The map may appear distorted during creation but self-corrects upon loop closure. However, the current camera setup
  requires above-average passes to achieve a high-quality map.
- **Navigation Behavior (Video 28):** While the robot reliably reaches waypoints, the pathing behavior can occasionally appear erratic or "odd" while
  maneuvering around dynamic obstacles.
- **Landmark Relocalization Jump (Video 31):** Using a fiducial marker to re-localize works as a coarse recovery mechanism, but the initial position "jump" is
  imprecise (visible at 2:33). RTAB-Map is required to refine the position afterwards.
- **Mapping Instability (Video 35):** A critical, random crash of the RTAB-Map process was captured during standard operation. The cause is unknown.

## Test Result

**Status: CONDITIONAL PASS** ⚠️

Localization performance is generally good, with successful recovery and stability across locomotion modes. However, the **random mapping crash** prevents a
full pass.

## Evidence (Videos)

- **Video 26:** [Loading and Expanding a Map](https://youtu.be/lcRYtxdA05o)
- **Video 27:** [Manual Map Building and Refinement](https://youtu.be/c8lwK_ZCB3M)
- **Video 28:** [Localization, Waypoints, and Obstacle Avoidance](https://youtu.be/kJ5P2cTgtOM)
- **Video 29:** [Localization Stability across Locomotion Modes](https://youtu.be/ZJyWUbtSMzs)
- **Video 31:** [Landmark-Based Re-localization](https://youtu.be/fgDDEhUxREY)
- **Video 34:** [Landmark-Free Self-Relocalization](https://youtu.be/K4TAXxeLxfM)
- **Video 35:** [ERROR - Unexpected Mapping Crash](https://youtu.be/_4CgIrlgDQU)

---

## Test ID: 4

## Overview

Full System Integration Verification. This phase verifies the combined operation of base and manipulator, reliability of inspection sequences from random start
points (video 33 and 33b), and system error handling (UI).

## Verified Functions

| Category        | Function            | Frame | Description                                                   | Result     | Evidence     |
|-----------------|---------------------|-------|---------------------------------------------------------------|------------|--------------|
| **Integration** | Body + Arm Playback | Odom  | Verifies combined base and arm recording playback             | ✅ Pass     | Video 32     |
| **Integration** | Arm Camera Nav      | Body  | Verifies navigation using manipulator camera (Tag Relative)   | ✅ Pass     | Video 30     |
| **Integration** | Blind Scanning      | Body  | Verifies arm articulation to scan areas hidden from body cams | ✅ Pass     | Video 30     |
| **Full System** | Fault Detection     | Map   | Verifies end-to-end inspection task (Base to Target -> Probe) | ✅ Pass     | Video 33     |
| **Full System** | Sequence Robustness | Map   | Verifies consistent convergence from random start positions   | ✅ Pass     | Video 33, 25 |
| **System**      | UI Error Handling   | N/A   | Verifies failsafes for invalid inputs                         | ⚠️ Partial | Video 36     |
| **System**      | Fiducial Range      | N/A   | Verifies detection range vs. marker size correlation          | ✅ Pass     | Video 37     |

## Observations & Notes

- **[ESSENTIAL] Reliable Convergence (Video 33):** This is the primary validation of the system. Prerecorded fault detection sequences were played back from
  random starting positions. Despite variance in the quadruped's standing position, the manipulator consistently compensated to reach the exact probe points.
- **Arm-Aware Collision Avoidance (Video 33):** At 2:12, the robot successfully navigated while accounting for the extended arm, avoiding a low ceiling. This
  proves the collision model includes the manipulator state.
- **Arm Camera Utility (Video 30):** The manipulator camera effectively extends the robot's sensing range, allowing it to find tags obscured from the main body
  cameras.
- **UI Robustness (Video 36):** While basic failsafes exist, testing was not exhaustive. Triggering failsafes generates excessive console noise.

## Test Result

**Status: PASS** ✅

The core robotic functionality, navigation, manipulation, and the integration of both for inspection tasks, is highly reliable and accurate. The "Essential
Demo" proves the system can perform its intended industrial task.

## Evidence (Videos)

- **Video 25:** [Recording Playback: Tag Interaction from Variable Start Positions](https://youtu.be/naVVEo8N9Y4)
- **Video 30:** [Manipulator Camera for Tag Interaction and Scanning](https://youtu.be/5axo3dUOJNc)
- **Video 32:** [Base and Arm Recording and Playback (Map/Odom Frame)](https://youtu.be/lJjbSmZ4tlM)
- ***Video 33:** [ESSENTIAL DEMO - Full System Validation](https://youtu.be/pEeT_Krulio)
- ***Video 33b:** [Process: Recording the Essential Demo Sequence](https://youtu.be/EIX_GhtDRFI)
- **Video 36:** [UI Error Handling and Failsafes](https://youtu.be/5Aw_TOcMqUM)
- **Video 37:** [Fiducial Marker Size vs. Detection Range](https://youtu.be/0tRXEuROshA)

---

## Issues Found While Testing (Anomalies & TODOs)

The following issues were identified during the verification process and require development attention:

### 1. Critical Stability Issues

- **Random Mapping Crash (Video 35):** RTAB-Map process terminated unexpectedly during standard mapping. Root cause unknown; requires investigation logs from
  the time of failure.
- **Manipulator E-Stop Latency (Video 12):** The Emergency Stop clears the command queue but fails to immediately arrest the *active* arm movement. This is a
  safety hazard.

### 2. Localization & Mapping Anomalies

- **Localization Map Refresh:** When extending a map and immediately switching to "Localization Only" mode, sometimes *only* the newly added data appears in the
  localization map.
    - *Workaround:* Closing the mapping session completely before restarting in localization mode seems to prevent this.
- **Map Quality/Effort (Video 27):** Generating a high-quality map requires an above-average number of passes due to the current camera setup.
- **Landmark Precision (Video 31):** Relocalization via single landmarks causes a significant position "jump" that is not precise.
    - *Suggestion:* Implement multi-marker constellations for a single landmark pose to average out errors.

### 3. Navigation & Motion Quality

- **Erratic Pathing (Video 28):** While the robot reaches goals reliably, the movement behavior during navigation (especially around obstacles) can look odd or
  jerky. Path smoothing is recommended.
- **Orientation Mode Failure (Test ID 1):** Non-custom orientation modes for the manipulator are not functioning; the system defaults to "custom" behavior.

### 4. User Interface & Developer Experience

- **Ubuntu Recording Behavior:** Anomalies noted in robot behavior specifically when using the Ubuntu screen recording tools (potential resource contention or
  lag).
- **Console Clutter (Video 36):** Triggering UI failsafes generates excessive exception logs, making debugging difficult. Error handling needs to be cleaner.
- **UI Edge Cases:** While basic failsafes exist, the testing was not exhaustive. Input validation needs expansion to cover complex edge cases.