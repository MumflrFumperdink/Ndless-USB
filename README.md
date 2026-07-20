# Ndless USB 

These are projects meant to develop a system for USB Communication on TI Nspire devices without the use of the [NavNet API](https://www.hackspire.org/Syscalls/#navnet).

## Projects

1. USB_computer - projects concerning calc-to-computer communication 
    1. USB_computer_test1_connecting - initial testing of the USB OTG controller

## Background

This was necessary because the NavNet API and previous Ndless USB apps (such as [HID drivers](https://github.com/ndless-nspire/nsptools-history/tree/master/hidn/trunk)) used OS interrupts. When testing NavNet apps which I programmed, it was thus necessary to use [TCT_Local_Control_Interrupts(0)](https://www.hackspire.org/Syscalls/#nucleus) in order to reenable OS interrupts within the Ndless program. These OS interrupts would force redraws on the screen, which depending on the device caused either a clock waiting cursor to constantly blink or for the entire screen to redraw over the program's draw, causing extremely strong screen tearing with the OS screen.