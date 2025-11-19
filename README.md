# stm32-bootloader-flasher
Custom UART bootloader for STM32L4R5ZIT6 with a Python Tkinter GUI tool to flash application firmware. The bootloader receives frames, verifies CRC, writes to Flash, and jumps to the App SW. The GUI handles HEX parsing, frame creation, serial communication, and provides a simple firmware update interface.
