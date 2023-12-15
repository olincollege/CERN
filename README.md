# CERN
ENGR 2110 Final Project: Centripetal Electronic Robotic Navigator (CERN)

#### Final Project by Ari Bobesh (Electrical), Venkadesh Eswaranandam (Electrical), Angela Huang (Mechancial), Sohum Kothavade (Mechanical), and Kevin Lie-Atjam (Firmware/Software)

Welcome to the Centripetal Electronic Robotic Navigator (CERN) project! In this repository, we embark on our exciting journey to build our own version of a Sphero-like robot—a small, spherical robot capable of dynamic movements and engaging interactions. Our goal was to create a Sphero-like robot with inductive charging, a custom PCB, modular and elegant design, and coherent software to control the robot.

## Dependencies and Repository Overview
This software runs on both C++ and Python. Our C++ infrastructure includes firmware for an ESP32 that can run the robot's control algorithms to drive the robot. The Python infrasture include code that is required to connect to the ESP32 over Bluetooth serial to control the robot.

## Repository Structure
The repository has a set of 4 folders which each have important information regarding the items inside of the repository

### app_interface
`app_interface` contains the Python script file required to run our control interface for our robots

### kicad
`kicad` contains a set of schematics and PCB board designs utilized in our project. Inside this folder, the `kicad.kicad_sch` file contains the transmitting circuit for the inductive charging base station. `robot.kicad_sch` and `robot.kicad_pcb` contain the circuit and PCB respectively for the receiving end of the charging circuit and the control circuitry placed inside our robot.

### motor_communicate
`motor_communicate` contains the C++ firmware required to run the robot.

### SymFoot
`SymFoot` contains footprints and symbols for our schematic with unique components not readily available in KiCAD.

### Library installation and Dependencies
To run the software, you will need the following:
* A Unix/Linux Operating System
* A Python Environment (Ex. Anaconda)
* PySerial
* Pygame
* PyBluez

To install the all of the necessary libraries, we recommend doing the following:
* Ensure that you're running a version of Python that is 3.9 or above
* Open a bash terminal on your machine
* Once it loads, I recommend creating a new Anaconda environment. This can be done by doing the following
```
conda create --name <Environment Name> python=3.9
```
Where you may name your environment whatever you would like
* Be patient and make sure that you type y to create your environment after a dialog box appears
* After the environment has been created, run the following command
```
conda activate <Environment Name>
```
To activate your Anaconda environment
* You're now ready to install the libraries to use this repository. Ensure that your file directory is at the root of this repository. By typing `ls` in the repository directory, you should see a file called `requirements.txt`. Finally, you can run the following

```
pip install -r requirements.txt
```
* Be patient and wait for the libraries to finish installing

## Robot Information
We built a set of 3 CERN Robots. The connection information and motor pinouts have been provided

| Name | MAC Address | Drive Pins | Turn Pins |
| ------- | ----- | ----- | ----- |
| Stubby | 78:21:84:B9:0A:1E | A1A: 33<br /> A1B: 32<br /> B1A: 2<br /> B1B: 23<br /> Sleep: 25| A1A: 12<br /> A1B: 14<br /> B1A: 26<br /> B1B: 27<br /> Sleep: 13|
| Girthy | E0:5A:1B:E4:66:56 | A1A: 26<br /> A1B: 27<br /> B1A: 14<br /> B1B: 12<br /> Sleep: 13| A1A: 2<br /> A1B: 23<br /> B1A: 32<br /> B1B: 33<br /> Sleep: 25
| Kirby | E0:5A:1B:E4:9D:D6 | A1A: 32<br /> A1B: 33<br /> B1A: 2<br /> B1B: 23<br /> Sleep: 25| A1A: 12<br /> A1B: 14<br /> B1A: 27<br /> B1B: 26<br /> Sleep: 13

## Connecting To The Robot

These steps work best with Linux based systems

Firstly, make sure that the robot you want to connect to is on. Next, you'll want to connect to the robot by Bluetooth using the systems settings to connect to your robot. You will briefly connect to the robot, but you will then disconnect to the robot. This doesn't mean that you've lost connection, but this only means that you've placed the connection to the robot as a familiar device. Next, you're going to want to run friendly_control.py under the app_interface folder which can be executed by using the following command in a bash terminal: `python friendly_control.py` which will then ask you what device you would like to connect to. You may also open the Python script in a code editor and run the code manually. Once you run the code, it will then ask you to enter your password to because the controls will enter in root. This is because the following code is ran `sudo rfcomm bind <PORT NUMBER> <MAC ADDRESS>` which is when a radio frequency communication port will be established by your computer and robot to the Bluetooth MAC address of your robot. This is important since data is sent over serial for bluetooth. This then opens up a Pygame experience which should allow you obtain live data of the robot, control the lights of the robot, and control the robot using the WASD keys. Once you've finished driving the robot, you may close the Pygame interface, and the following command is ran: `sudo rfcomm release <PORT NUMBER>` which is when the communication port established will be dissolved, and the robot can then be reconnected or connected to another client.

If you have trouble connecting to the robot, typically running `sudo rfcomm release <PORT NUMBER>` to ensure that there doesn't exist a port that already exists that the robot tries to talk to on your computer to is typically recommended.

If you have any questions about any aspect about our project, please feel free to reach out to any of us: Ari Bobesh (abobesh@olin.edu), Kevin Lie-Atjam (klieatjam@olin.edu), Venkadesh Eswaranandam (veswaranandam@olin.edu), Sohum Kothavade (skothavade@olin.edu), and Angela Huang (ahuang3@olin.edu)
