# Robot Guidance Challenge Using Assembly

## Overview
This project implements an embedded control system for a robot (eebot) using Assembly language on the HCS12 microcontroller. The system processes sensor data, controls motor movement, and manages an LCD to navigate a maze efficiently. The robot uses advanced state management and real-time feedback mechanisms to complete its tasks.

---

## Course Information
- **Course Title:** Microprocessor Systems  
- **Course Number:** COE 538  
- **Instructor:** Prof. Lev Kirischian  
- **Semester/Year:** Fall 2024  
- **Assignment Title:** Robot Guidance Challenge  
- **Submission Date:** Tuesday, November 26, 2024  
- **Due Date:** Monday, December 2, 2024  

---

## Team Members
- **Kamal Dabban** 
- **Ali El-Hage** 
- **Muhammad Wajeeh Ul Hassan** 

---

## Features
- **Maze Navigation:**  
  - Tracks black lines using five photoresistor sensors.  
  - Detects dead ends with bumpers and executes a 180-degree turn to resume navigation.  
  - Handles intersections by processing sensor input and executing planned turns.  

- **Real-Time Feedback:**  
  - Displays robot state, sensor data, and battery voltage on the LCD.  
  - Converts binary sensor data to readable ASCII values for display.  

- **Interrupt Management:**  
  - Uses timer overflow interrupts for precise motor adjustments and sensor readings.  

---

## System Components
1) **Initialization:**  
   - Initializes ports, ADC modules, and the LCD for motor and sensor operations.  

2) **Sensor Reading and Processing:**  
   - Collects data from multiple sensors (line, bow, port, mid, starboard).  
   - Processes sensor data to determine the robot's next steps.  

3) **State Machine for Movement:**  
   - Maintains various robot states like `FWD`, `LEFT_TRN`, and `REV_TRN`.  
   - Executes subroutines to handle motor operations based on the current state.  

4) **LCD Operations:**  
   - Updates the LCD with information such as "Battery Voltage" and "State."  

5) **Interrupt Service Routine:**  
   - Handles time-sensitive tasks using timer overflow interrupts.  

---

## Key Code Snippet
```assembly
MAIN
  JSR G_LEDS_ON      ; Turn guider LEDs on
  JSR READ_SENSORS   ; Fetch sensor data
  JSR G_LEDS_OFF     ; Turn guider LEDs off
  JSR UPDT_DISPL     ; Update LCD display with current data
  LDAA CRNT_STATE    ; Load current state
  JSR DISPATCHER     ; Transition to appropriate state
  BRA MAIN           ; Repeat loop
```
---

## Observations

   - Success: The eebot successfully completed the maze on its first run.
   - Challenge: On subsequent runs without restarting, the robot lost track of the maze midway. Restarting resolved the issue, indicating potential state retention or reinitialization problems.

---

## Setup and Usage
Prerequisites:

   - HCS12 microcontroller
   - Assembly IDE (e.g., CodeWarrior)
   - Hardware setup with sensors, motors, and LCD

---

## Steps:

1) Clone the repository:
```bash
    git clone https://github.com/KamalD-glitch/Robot-Guidance-Challenge-Using-Assembly.git
```
2) Open the project in your Assembly IDE.
3) Upload the program to the HCS12 microcontroller.
4) Connect the microcontroller to the robot hardware and run the program.

---

## Future Improvements:

1) Implement state retention and reinitialization to ensure consistent performance across multiple runs.
2) Expand functionality for more complex maze layouts.
3) Integrate obstacle detection and avoidance mechanisms.

---
## Acknowledgments

   - Prof. Lev Kirischian for his guidance and mentorship.
   - Collaborators Ali El-Hage and Muhammad Wajeeh Ul Hassan for teamwork and contributions.
