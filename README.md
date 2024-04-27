
# ESE 3500 Final Project - Spring 2024

    * Team Name: PostureGuard: Real-time Posture Monitoring and Correction System
    * Team Members: Tobby Zhu, Steven Walxim
    * Github Repository URL: https://github.com/ese3500/final-project-spine
    * Description of hardware: Atmega328PB, ESP32 Feather, MPU6050, buzzer


![ESE 3500 Final Project Block Diagram](https://github.com/ese3500/final-project-spine/assets/16732316/76a91e80-e5a7-4b1b-82af-96471e74eef5)

### 1. Video

[Link to video](https://drive.google.com/file/d/1KKnS5WusZEDf0ZXcu7LeC7Q3t4eq5MJ0/view?usp=drive_link)

### 2. Images

![IMG_0531](https://github.com/ese3500/final-project-spine/assets/16732316/16737da9-3458-4fd0-84e9-781d0c3a3523)


### 3. Results

Our final solution closely follows our initial design. We connect 4 MPU 6050 IMUs to Atmega328PB using I2C protocol. The Atmega328PB calculates the roll, pitch, and yaw angles of each of the IMU, and runs a pose detection algorithm to alert the user of bad posture. The IMU readings and the posture alerts are sent to the Blynk webpage via an ESP32 Feather, which is connected with Atmega328PB via UART. We specify our SRS and HRS results below.

#### 3.1 Software Requirements Specification (SRS) Results

1. Processing data from multiple IMUs and convert into real-time acceleration and pitch, yaw angles, with accuracy of <2 degrees for the angles  
   We meet this requirement. By our measurement, all angles are within 2 degrees with respect to our iPhone IMU readings. However, the Mahony algorithm we use leads to significant drifting of the readings after 30 seconds after reset.
   
2. Reconstruct the relative position of wearer's spine and show an understandable visual representation to user  
   We do not meet this requirement. Reconstruction of the spine based on angle data proved difficult. We were also unable to run Python script on Blynk.
   
3. Send software alert if certain dimension of the spine's position is beyond a preset threshold  
   We meet this requirement. We tested for "Neck angle too flat", "Hunch back", and "Pelvic tilt", which were accurately alerted for by our system.
   
4. A reliable and secure wireless communication protocol between the IMUs/ESP32 modules and the server hosting the webpage  
   We meet this requirement. We tested continuously running our system for 30 minutes and our end-to-end connection, including hard-wired pins, I2C connection, UART connection, and WiFi connection remained stable.

#### 3.2 Hardware Requirements Specification (HRS) Results

1. IMUs (Inertial Measurement Units): IMUs with 6 degrees of freedom, with minimum sampling rate >100Hz  
   We tested that all of our IMUs have sampling rates >10k Hz. We are using a 61Hz sampling rate in this project.
2. ESP32 Modules for wireless data transmission   
   This requirement is trivially met.
3. A 5V DC vibrater that produces a noticeable vibration when worn on body  
   We decided to replace the vibrater with an LED and an audible buzzer. The LED and buzzer work as desired.  
4. Power Supply: 5V and 3.3V battery packs or power solutions that can sustain the operation of the IMUs, ESP32 modules, and buzzer for extended periods.  
   We opted for a 5V DC powerbank with USB-A and USB-C ports instead of battery packs. The powerbank is able to systain the operation of our system.  
5. Wearable Vest: A comfortable, adjustable vest designed to hold the IMUs in the correct positions along the user’s spine.  
   We made a wooden movable spine in lieu of the vest. This is because our system is unstable when operating on real human bodies due to robustness issues of the Mohony algorithm.

### 4. Conclusion

We are able to construct a baseline system that accurately measures the body pose of a human, which we simulated by a wooden structure, and outputs alerts when bad posture is detected. Through this process, we learned the algorithms that convert IMU raw data to usable angular data, which in turn empower endless devices in our life. We were able to digest a library designed for other AVR chips and adapt it to fit (1) our Atmega328PB chip and (2) our more complicated use case with multiple IMUs on different buses and with different addresses. 

A few key obstacles we anticipated are: 
1. The challenge in finding a suitable library or starter code for the IMU sensors
2. The lower than expected robustness of the IMU angle readings, especially the significant drifting  
3. The difficulty in fitting the physical system to a wearable vest and make it versatile

The main next step for this project is incorporating a more robust algorithm for the IMU. Algorithms that accurately convert IMU raw data to raw, pitch, and yaw angles were an active area of research in the 2010s. Currently the Madgwick / Fusion algorithm is the best open source algorithm for IMUs. We expect to achieve significantly better robustness after replacing the Mahony algorithm with the Fusion algorithm.

## References

We referenced a Mohony algorithm library by Davide Gironi. The code is mostly based on a port of the arduino mpu6050 library by Jeff Rowberg (https://github.com/jrowberg/i2cdevlib). We also benefited from the resources about Mahony complementary filter for attitude estimation (http://www.x-io.co.uk).




