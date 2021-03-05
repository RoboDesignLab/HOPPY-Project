# ------------------------HOPPY-Project------------------------
## HOPPY: HOPPY: An Open-source Kit for Education with Dynamic Legged Robots
![HOPPY 2](https://user-images.githubusercontent.com/72820863/109865385-9af23500-7c29-11eb-8a4e-80609e3c8e9d.png)
----------------------------------------------------------------------------------------------------------------------------
## CONTENTS OF THIS FILE
---------------------
* Introduction
* General Notes
* Setting up CCS 
* CAD 

## Introduction
------------
The project is an open-source, low-cost, robust, and modular kit for robotics education. The robot, HOPPY, dynamically hops around a fixed gantry. The kit targets lowering the barrier for studying dynamic robots and legged locomotion in real systems. Controlling dynamic motions in real robots present unique challenges to the software and hardware which are often overlooked in conventional robotics courses. This project describes the topics which can be studied using the kit, lists its components, discusses best practices for implementation, presents results from experiments with the simulator and the real system, and suggests further improvements. HOPPY was utilized as the topic of a semester-long project for the Robot Dynamics and Control course at the University of Illinois at Urbana-Champaign. Students provided a positive feedback from the hands-on activities during the course and the instructors will continue to develop and improve the kit for upcoming semesters.

Find out more in our overview paper: https://arxiv.org/pdf/2010.14580.pdf

Assembly instructions can be found here: https://youtu.be/CDxhdjob2C8

See the robot in action here: https://youtu.be/_lbKIpiRWKI

-----------------------------------------------------------------------------------------------------------------------------
## General Notes
-------------
DO NOT run the given code to the motors while they are attached to the leg. This code is meant to show a given code flow and test the motors, board, and program setup. Clamp the motors down and run the code on them externally. 

Depending on how the leg is assembled, you may need to adjust the lengths used in the code. More instructions re inside the general definitions section of the program.

----------------------------------------------------------------------------------------------------------------------------
## Setting up Code Composer Studio (CCS) and the programming enviroment
--------------------------------------------------------------------
CCS is the IDE for TI products and is an easy to sude system. A introduction to CCS can be found at the following link: https://www.youtube.com/watch?v=11lsNYW7zkw&ab_channel=CodeComposer

Hoppy was implemented using version 8 of CCS which can be downloaded at the following link: http://software-dl.ti.com/ccs/esd/documents/ccs_downloads.html. The instructions for installing the software can be found at the second link. 

Once CCS in installed, unzip the Blinky_rtos_FLASHCCS8.zip folder and open the project in CCS. This project should blink an LED on the board after it is flashed onto the board. cpu01_main.c is the main file that you will be working in; this is also the file that you should be in when you build and debug the program.

Once the programming enviroment is working, the file cpu01_main_EXAMPLE.c can be coppied into cpu01_main.c. This program is a starting point for the user to program. It includes an intuitive code structure, working PD control on the hip and knee motors, and contact sensing (analog and binary). The instructions to use the code setup and the different sensing options is in the file with line numbers for each part in the code. 

The wiring diagram that shows how hook up the system is also included in the folder.

----------------------------------------------------------------------------------------------------------------------------
## CAD
---

The CAD models of the entire setup are inlcuded in SolidWorks and .STEP formats. Video  instructions for mechanical assembly can found at the following link: https://youtu.be/CDxhdjob2C8

The Bill of Materials (BOM), a complete list of components and quantities, is also listed in the folder. 

-----------------------------------------------------------------------------------------------------------------------------
## Simulator
---------

Finally, the MATLAB-based dynamic simulator code and instructions are included in its own folder. Further instructions are included within. 
![](https://i.imgur.com/Ck73nsp.gif)
-----------------------------------------------------------------------------------------------------------------------------
