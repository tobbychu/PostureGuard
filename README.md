[![Review Assignment Due Date](https://classroom.github.com/assets/deadline-readme-button-24ddc0f5d75046c5622901739e7c5dd533143b0c8e959d652212380cedb1ea36.svg)](https://classroom.github.com/a/2TmiRqwI)
# final-project-skeleton

    * Team Name: PostureGuard: Real-time Posture Monitoring and Correction System
    * Team Members: Tobby Zhu, Steven Walxim
    * Github Repository URL: https://github.com/ese3500/final-project-spine
    * Github Pages Website URL: [for final submission]
    * Description of hardware: Atmega328PB, ESP32 Feather, MPU6050, buzzer


![ESE 3500 Final Project Block Diagram](https://github.com/ese3500/final-project-spine/assets/16732316/76a91e80-e5a7-4b1b-82af-96471e74eef5)

## Final Project Report

Don't forget to make the GitHub pages public website!
If you’ve never made a Github pages website before, you can follow this webpage (though, substitute your final project repository for the Github username one in the quickstart guide):  <https://docs.github.com/en/pages/quickstart>

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



    
## Final Project Proposal

### 1. Abstract

PostureGuard is a novel, wearable system designed to combat the prevalent issue of poor body posture, exacerbated by the modern lifestyle of prolonged sitting and screen time. Utilizing an array of IMUs placed on the user's back, the system accurately monitors spinal orientation and movement in real-time. Data transmitted via ESP32 chips to a webpage allows for immediate visual feedback on the user’s posture. If unhealthy posture is detected, the system promptly alerts the user through a buzzer, facilitating instant correction. This project combines hardware integration and web development to offer a non-intrusive, cost-effective solution for real-time posture monitoring and correction, aiming to contribute to long-term health improvements and increased awareness about posture.

### 2. Motivation

Poor body posture can lead to long-term health issues, including back pain, spinal dysfunction, joint degeneration, and reduced flexibility. With the increasing amount of time people spend sitting at desks and looking at screens, there's a critical need for a proactive solution to monitor and correct posture in real-time. Most existing solutions are either intrusive, expensive, or fail to provide instant feedback. Our project, PostureGuard, aims to bridge this gap by offering a non-intrusive, cost-effective, and immediate feedback mechanism to encourage healthier body posture.

### 3. Goals

Our project aims to develop a lightweight, wearable system using multiple IMUs placed on a person's back to accurately monitor their posture in real-time. This data will be transmitted via an ESP32 module to a webpage that visually represents the user's current posture. If unhealthy posture is detected, the system will immediately alert the user through a buzzer. By the project's conclusion, we aim to achieve:  
•	Real-time posture monitoring with visual feedback on a webpage  
•	Instantaneous alert system for posture correction   


### 4. Software Requirements Specification (SRS)

1. Processing data from multiple IMUs and convert into real-time acceleration and pitch, yaw angles, with accuracy of <2 degrees for the angles
2. Reconstruct the relative position of wearer's spine and show an understandable visual representation to user
3. Send software alert if certain dimension of the spine's position is beyond a preset threshold
4. A reliable and secure wireless communication protocol between the IMUs/ESP32 modules and the server hosting the webpage.


### 5. Hardware Requirements Specification (HRS)

1. IMUs (Inertial Measurement Units): IMUs with 6 degrees of freedom, with minimum sampling rate >100Hz.
2. ESP32 Modules for wireless data transmission
3. A 5V DC vibrater that produces a noticeable vibration when worn on body
4. Power Supply: 5V and 3.3V battery packs or power solutions that can sustain the operation of the IMUs, ESP32 modules, and buzzer for extended periods.
5. Wearable Vest: A comfortable, adjustable vest designed to hold the IMUs in the correct positions along the user’s spine.

### 6. MVP Demo

We expect to demo the following functionalities for MVP:
1. IMU data collection and transmission: the IMUs should accurately collect posture data and transmit them through Internet to the server
2. Preliminary body pose reconstruction: the wearer's body pose should be reconstructed from the IMU data; this information can be printed on serial monitor or the webpage


### 7. Final Demo

By the final demonstration, we aim to achieve:

Advanced Posture Monitoring: Full implementation of real-time data processing and analysis with a detailed 3D visualization of the user's posture on the webpage.
Refined Alert System: A fully integrated alert system with adjustable thresholds for posture correction, providing instantaneous feedback through the buzzer.
Comprehensive Webpage Interface: A fully developed webpage interface that not only displays real-time and historical posture data but also offers user-specific recommendations for posture improvement.

### 8. Methodology

The project is naturally divided into 4 phases, which we plan to complete in sequence and test each module for functionality before moving onto the next one.
1.	IMU Integration and Data Collection: We will start by integrating multiple IMUs onto a wearable vest to collect posture data. This setup will capture the orientation and movements of the wearer's spine.
2.	Data Transmission and Web Development: Utilizing ESP32 modules, the collected data will be sent to a server that processes the information and updates a webpage in real-time to display the wearer's posture.
3.	Feedback System: If the system detects unhealthy posture, it will trigger a buzzer attached to the vest, alerting the wearer to correct their posture.
4.	Analysis and Improvement: We plan to collect and analyze posture data to provide users with insights and recommendations for long-term posture improvement.


### 9. Components

•	IMU Sensors: To measure orientation and acceleration, providing data on the wearer's posture. 

•	ESP32 Modules: For wireless data transmission between the IMUs and the web server.

•	Buzzer: For immediate feedback to the wearer when incorrect posture is detected.

•	Webpage: To display real-time posture visualization and long-term posture analytics.


### 10. Evaluation

•	Accuracy of the IMU sensors: the sensors can precisely capture 3D posture of the body as per SRS.

•	User experience of webpage: the webpage or any other visual representation of body posture is intuitive to the user to understand

•	Effectiveness of the feedback mechanism: the warning system against bad posture is effective for real use cases


### 11. Timeline

This section is to help guide your progress over the next few weeks. Feel free to adjust and edit the table below to something that would be useful to you. Really think about what you want to accomplish by the first milestone.

| **Week**            | **Task** | **Assigned To**    |
|----------           |--------- |------------------- |
| Week 1: 3/24 - 3/31 |  find equipments        |   steven                 |
| Week 2: 4/1 - 4/7   |  finish programming the first IMU        |         BOTH           |
| Week 3: 4/8 - 4/14  | assemble multiple         |          BOTH          |
| Week 4: 4/15 - 4/21 | disconnected usage (wifi)         |       BOTH             |
| Week 5: 4/22 - 4/26 |  Final Product        |       BOTH             |

### 12. Proposal Presentation

Add your slides to the Final Project Proposal slide deck in the Google Drive.


