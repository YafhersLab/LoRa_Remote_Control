# 📡 LoRa Point-to-Point Remote Monitoring and Control System

## 📌 Project Overview
This project consists of the development of a **point-to-point LoRa communication system** using two custom electronic boards: a transmitter and a receiver.

The transmitter board controls a **water pump via relay** and reads environmental parameters such as **water temperature and humidity**. Sensor data is transmitted wirelessly using **LoRa** to the receiver board. An **ESP32** on the receiver side forwards the information via **Bluetooth** to a mobile application developed with **MIT App Inventor**.

This project demonstrates skills in:
- LoRa wireless communication  
- Embedded systems development  
- Remote monitoring and control  
- Mobile application integration  

---

## 🎯 Motivation
The objective of this project was to design a **low-power, long-range wireless monitoring and control system** capable of operating in remote environments.

By combining LoRa communication with Bluetooth and a custom mobile application, the system enables real-time monitoring and remote actuation of physical devices such as water pumps.

---

## 🧠 System Description
- The transmitter board:
  - Reads water temperature and humidity sensors
  - Controls a water pump using a relay module
  - Sends sensor data via LoRa

- The receiver board:
  - Receives LoRa data
  - Processes information using an ESP32
  - Transmits data via Bluetooth to a mobile application

- A custom mobile app displays sensor data and system status

---

## 🧩 Project Structure

1. **code/**  
   Firmware for both transmitter and receiver boards, including LoRa communication, sensor acquisition, and Bluetooth transmission.

2. **apk/**  
   Compiled Android application developed using MIT App Inventor.

---

## 🛠️ Hardware Used
- **Microcontroller:** ESP32 WROOM 32  
- **Custom PCB:** Prototype board (hand-fabricated PCB)  
- **Additional Components:**  
  - LoRa modules  
  - Relay module  
  - Water temperature sensor  
  - Humidity sensor  

---

## 💻 Software & Tools
- **Arduino IDE** – Firmware development  
- **MIT App Inventor** – Mobile application development  
- **SolidWorks** – Enclosure (case) design  

---

## 🚧 Project Status
**Experimental**

The system is currently in an experimental phase, serving as a functional prototype for remote monitoring and control applications.

---

## 📚 Future Work
- Bidirectional LoRa communication
- Improved power management for field deployment
- Secure communication protocols
- Cloud integration for remote data logging

---

## 📄 License
This project is intended for **educational and research purposes**.  
License to be defined.

---

## ✍️ Author
**Yafhers Mendoza**  
Embedded Systems & Wireless Communication Design