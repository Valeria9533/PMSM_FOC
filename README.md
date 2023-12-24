# PMSM_FOC
Field-oriented control of PMSM with 15 pole pairs, halls and absolute encoder. Max current = 20 A, max voltage = 27 V.

Code is written for STM32F407 MCU, using HAL library.

For now only direct torque control is implemented. Speed loop is coming soon...

Freemaster software is used (but not nessesarily) for useful control of motor and see plots and transients of currents, voltages, etc. 
