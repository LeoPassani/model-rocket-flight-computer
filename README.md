# Flight Computer & Rocket Design and Testing (MATLAB, SolidWorks, ANSYS Fluent)
**June – August 2026**

I Designed and tested an original ESP32-based flight computer on a custom 3D-printed rocket. The objective was to expose myself to various aspects of aerospace problem solving, including attitude tracking of flight systems, modeling and aerodynamic design, data processing, and experimental validation of models. The flight computer integrates an IMU, barometer, and microSD card for onboard flight-data logging. MATLAB was used to model the rocket trajectory using motor thrust curves and aerodynamic drag data generated in ANSYS Fluent.

## Rocket Design

The **Peter Trifin** is a custom 3D-printed rocket designed in SolidWorks and powered by an estes D-12 motor. The design features an integrated tri-fin configuration and motor mount and consists of three primary printed components:

- Main body tube with integrated fins
- Nosecone
- Bulkhead

<p align="center">
  <img src="https://github.com/user-attachments/assets/e660307b-423b-49b6-ba19-d59ba67f8ddc" width="70%">
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/27a286ae-1bfc-4267-ac07-75a1b4e9fed0" width="28%">
  &nbsp;&nbsp;&nbsp;
  <img src="https://github.com/user-attachments/assets/308295bc-b0d1-43fd-9301-8d90710e20d3" width="28%">
</p>

<p align="center">
  <em>SolidWorks design and 3D-printed Peter Trifin rocket</em>
</p>

## ESP-32 Flight Computer Design

The Flight computer houses an Adafruit LSM6DSOX IMU, Adafruit BMP280 Barometer, ESP32-DEVKITC-32UE-ND, and an Adafruit MicroSD breakout board, as well as voltage protection switches and voltage regulator components that connect the 3.7V lithium-polymer battery 500mAh. At a rate of 104Hz, the IMU collects accelerations and angular velocities in the body frame of the rocket. The flight computer went through a rigorous and long prototyping process as there was some new concepts for me to learn about communication protocols, soldering, and using the arduino IDE on the esp32. While I wasn't completely new to arduino-style projects, this was a large step compared to previous high school projects. I began with breadboard tests of each sensor, ensuring that the values coming out were plausible. I used processing to live test the madgwick filter that is part of the arduino IDE library. Before I knew it, I had a live feed on the orientation of the computer, however the euler angles were limited in that a full turn would cause the model to do an additional turn in the opposite direction. This is due to roll pitch and yaw maxing at certain values and needing to remain within a certain range. I switched to quaternion based orientation to solve for this (another new concept to me!). 
<p alight="center">
  <img src="https://github.com/user-attachments/assets/056d5936-8b60-441b-8bba-fd24b30d608e" width="28%">
  &nbsp;&nbsp;&nbsp;
  <img src="https://github.com/user-attachments/assets/74565068-12cb-41aa-99e0-bb654063825e" width = "50%">
</p>

Once I felt confident in the accelerometer capabilities, I incorporated the barometer and microSD logging board and prepared a simple output into the SD card. This is a sample of what the csv file being written into the SD card looks like. 
| timestamp | qw   | qx   | qy   | qz    | altitude | ax    | ay  | az   | gx    | gy   | gz   | temp |
|-----------|------|------|------|-------|----------|-------|-----|------|-------|------|------|------|
| 4948167   | 0.99 | 0.11 | 0.07 | -0.01 | -0.03    | -1.37 | 2.1 | 9.66 | -0.01 | -0.03 | -0.02 | 27.65 |
| 4957782   | 0.99 | 0.11 | 0.07 | -0.01 | -0.03    | -1.37 | 2.1 | 9.66 | -0.01 | 0.04 | -0.02 | 27.65 |

From there, it was a matter of making a more robust and compact prototype that could fit into the nose cone. It took a lot of failed soldering before I finally had an accomplished prototype. The design fit sleek into the nose cone, although I unfortunately had to scrap the bulk head as it couldn't fit anymore. I relied on heavy taping to prevent hot gas from entering the nose cone during flight. Thankfully, the vent holes I drilled into the nose cone shoulders were sufficient in pressurizing the nosecone to match outside conditions.
<p alight="center">
  <img src="https://github.com/user-attachments/assets/5c4d1069-525d-411c-8a9d-1cc06d569905" width="28%">
  &nbsp;&nbsp;&nbsp;
  <img src="https://github.com/user-attachments/assets/b3d2e13f-38fc-4785-9531-e04906e2cfaf" width ="28%">
</p>

After the switch is activated, the battery begins to power the whole circuit. The flight computer spends the first 3 seconds calibrating the sensors, and then immediately begins collecting data. After the flight, the microSD card is collected and the raw data is imported into my MATLAB program. 

## MATLAB Processing
RocketFlight.m trims the data taken while the rocket was still on the launchpad or already touched down, determines apogee height and time, and performs simple quaternion linear algebra to convert from body frame to earth frame. In the earth frame, vertical velocity can easily be integrated from the IMU readings, although of course there is drift that cannot be accounted for. Future projects could determine how the altitude values from the barometer could help adjust and reduce the accelerometer drift. I developed a simple iterative trajectory model that interpolates drag results from ANSYS simulations of the rocket and thrust curves from a rocket performance site to predict the trajectory of the rocket. The goal was to develop a prediction and test it's accuracy against experimental results. Observing the data, the D-12 Motor had a stronger liftoff power than expected, resulting in a faster initial rise, however this resulted in a shorter burn time. Overall, the expected maximum velocity was only off by 1.8%, while the maximum apogee was off by 6.5%.

<p alight="center">
  <img src="https://github.com/user-attachments/assets/3e6695c1-52cf-4f35-9c40-df0bdff24ada" width="100%">
  &nbsp;&nbsp;&nbsp;
  <img src="https://github.com/user-attachments/assets/4dde7c43-92bb-4637-8ebf-280eaa4c6e5d" width ="100%">
</p>

| **Parameter** | **Calculated** | **Recorded** | **Difference** | **Percent Error** |
|---|---:|---:|---:|---:|
| Apogee (m) | 88.79 | 83.39 | 5.40 | 6.50% |
| Time to Apogee (s) | 4.957 | 4.461 | 0.496 | 11.10% |
| Maximum Velocity (m/s) | 34.39 | 35.00 | -0.61 | 1.80% |
| Burn Time (s) | 1.65 | 1.31 | 0.342 | 26.20% |

## ANSYS Drag Computations
