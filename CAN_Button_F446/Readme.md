# Button Press CAN Communication Summary

Implemented bidirectional CAN communication between an STM32F446 microcontroller and a computer using the mjbots FDCANUSB adapter. 
The STM32 board uses the MCP2542FD CAN transceiver connected to CAN1 peripheral operating in classic CAN mode at 1 Mbps. 
A button connected to GPIO PA2 triggers an external interrupt that sends an 8-byte CAN message with standard ID 0x100 containing "BTN" followed by a 32-bit counter, while simultaneously toggling an LED on PC5 for visual feedback. 
The FDCANUSB adapter required initial configuration via Python script to set the bitrate to 1 Mbps (`can.bitrate 1000000`), disable FD mode (`can.fdcan_frame 0`), disable bit rate switching (`can.bitrate_switch 0`), and enable 120Ω termination (`can.termination 1`). 
After running `configure_classic_can.py` once to save these settings, the `test_can_minimal.py` script connects to the FDCANUSB, displaying "BUTTON PRESSED" messages with timestamps each time the button is pressed.
