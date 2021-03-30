# Parcel-locker_Zbox
Entrance project for the company Zásilkovna. Implementation of parcel locker for Z-box. NUCLEO-G070RB implementation target platform. Used HAL STM32CubeIDE.

The repository contains two projects:
  - ParcelLocker_DISCO
  - ParcelLocker_NUKLEO
  
Software development and test was done on board : https://docs.rs-online.com/43fc/0900766b814410f7.pdf
After the development of sw. a project has been created for the NUCLEO board by required from the client.

The differences between the processors mounted on the NUCLEO-STM32G070RBT6 and DISCO-STM32L476VGT6 dps are:
  - DISCO has M4 core, NUCLEO has M0 core.
  - DISCO is more power economical.
  - NUCLEO has UART Rx FIFO buffer - is not used in sw.
  - NUCLEO UART2 is connect to PA2, PA3, DISCO UART2 is connect to PD5, PD6. 

Source code which was developt is located in: 
Core |-> Src |-> parcel_locker.c
     |       |-> main.c  
     |-> Inc -> parcel_locker.h
