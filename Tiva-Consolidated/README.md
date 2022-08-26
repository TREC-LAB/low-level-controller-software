![TREC LAB LOGO](https://uploads-ssl.webflow.com/5eea7449e8feaa669ef05b29/621010e435b9e48b50133758_TREC.png)
<!-- Headings -->
# Pandora Tiva Low-Level Code

## Purpose:
Designed for the [Virginia Tech TREC Lab](https://trecvt.org) by [Nick Tremaroli](nicktrem@vt.edu)

Low-level code which allows for the controlling of pandora
---

<!-- Vertsions of code -->
### *v2.1:*
* Added software E-stop reset
* Re-initialization
* Software E-stop enable feature now supported in initialization
* Better EtherCAT structure for sending and receiving data
	* Union of structures
* improved initialization procedure
	* no longer using dynamic memory allocation
	* each initalization frame corresponds to a particular peripheral
* Moved LED pins
	* no longer a conflict with SSI1
---
### *v2.0:*
* Better organization of structures and functionality for peripherals
* Encoder zeroing algorithm was put in place
* Initialization procedure expanded significantly
	* Utilized dynamic memory allocaiton for initialization
	* Only EtherCAT and the location pins get enabled
		* The rest get initialized during the beginning of master communication
---
### *v1.0:*
* HAL established
* Basic organization of structures for peripherals
* Location Pins Programmed
* minimal initialization put in place
* EtherCAT functionality
	* Using bitwise shift operations for reading/writing to the frame
	* Initial EtherCAT signals put in place
		* Halt
		* Location
		* Control
		* Idle
* Initial control and communication loop working
	* Data is able to be sent, read, and processed
	* Master Process ID Established

---
#### TODO:
* Sensor Programming
	* Program IMU into formal implementation
	* Program CAN ATI into formal implementation
* Framework
	* Abstract code more
	* Add programming optimizations
	* Improve initialziation to expand to a various other sensors
* Oddities to look into
	* When hardware E-stop is presses and unpressed, the Tiva sends random PWM signals
