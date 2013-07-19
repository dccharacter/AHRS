AHRS
====

This is a test project for STM32F3-Discovery. It works as the discovery board spatial positioning visualization system. It uses all the three sensors that are on board of STMF3-Discovery.

FUSION
======
Fusion code: either of
 -Madgwick algorythm
 -Mahony algorythm

I found them here: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
Please let me know if I give a wrong reference or fail to properly identify authors

You can switch between algorythms by commenting/uncommenting appropreate lines in file sensors.c:
  #define USE_MADGWICK_AHRS
  //#define USE_MAHONY_AHRS

OUTPUT
======
The firmware can use USART (Quaternion) or USB-HID(Euler angles) for data output

Either USART_USE_USB or USART_USE_USART must be defined for this

I do this in project settings (external pre-defined macro)

VISUALIZATION
=============
There's a Python visualization script myVisualisation.py
To use this one, you need:
 -Python 2.7
 -pywinusb (to use usb)
 -VPython
 -Windows (I guess - for python libraries???)
To switch between USART and USB, you have to change the following line in main():

comm = CommunicationDevice('joystick')
change to 
comm = CommunicationDevice('serial')
