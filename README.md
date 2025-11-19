📝 STM32L4R5ZIT6 Custom Bootloader with Python-Based Flashing GUI

This project contains a custom UART bootloader for the STM32L4R5ZIT6 microcontroller, designed to reliably receive, validate, and flash application firmware into internal Flash memory.
It also includes a Python-based GUI tool that automates firmware flashing, offering an easy and user-friendly interface.

🚀 Key Features
1. Custom Bootloader (STM32L4R5ZIT6)
2. Supports firmware download via UART
3. Handles Intel HEX or binary frame formats
4. Implements: Frame length validation, Command decoding, Data CRC verification, Page buffering and Flash programming
5. Uses STM32 HAL Flash drivers for reliable Flash operations
6. Supports reset-and-jump to Application Software (App SW)
7. Compatible with Dual Bank Flash mode

🔹 Python GUI Flashing Tool
A desktop flashing utility built using Python Tkinter.
GUI Features:
1. COM port auto-detection
2. Baud-rate selection
3. HEX file loading
4. Frame packetization & CRC calculation
5. Real-time log window
6. Start Flashing / Abort controls
7. Automatic ACK/NACK handling with retries
8. End-of-frame verification

Communication Protocol:
[ Frame Length | Command | Address | Payload | CRC ]

🧰 Tools & Technologies
> STM32CubeIDE
> STM32 HAL Drivers
> Python 3.x
> Tkinter
> pySerial


📦 How It Works (High-Level)
Bootloader starts → checks for a flash request or jumps to App SW
GUI sends flash command and waits for ACK
HEX file is split into frames with CRC

Bootloader:
-> Receives data
-> Validates CRC
-> Buffers data into Flash pages
-> Programs the page into Flash

After completion → bootloader resets and jumps to the application

🎯 Use Cases
This project is ideal for:
-> Field firmware updates for embedded systems
-> Lightweight alternative to STM32CubeProgrammer
-> Custom UART-based communication protocols
-> Training, demos, internal toolchains
