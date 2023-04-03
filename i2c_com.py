#####
# 
# This class is used to facilitate communication to an STM32 microcontroller via I2C.  
# and is licensed under the MIT License. 
# It uses the Python smbus library to access the Raspberry Pi peripherals.
# The STM32 has been given an address of 0x10 and has a digital input that is connected to
# an IR sensor. When the IR sensor is triggered, due to the proximity of an obstacle, it sends
# a 1 over the communication channel, otherwise it sends a 0. 
# Add logic to this code in order to perform an action once a '1' signal is received. In this case, 
# it prints the output.

import smbus
import time

class i2c_com(object):
    """
    This class provides the implementation of communication between a Raspberry Pi and a 
    STM32 sensor via I2C. The microcontroller address is initialized here as well as the a SMBus object with 
    the I2C port 1. This is the port that is exposed for users to use the I2C interface on a Raspberry Pi.

    """
    def __init__(self) -> None:
        self.i2c_addr = 0x10

        self.i2c_bus = smbus.SMBus(1)
       

    def generateTelemetry(self)->int:
        """
        This function facilitates the interaction via I2C in order to obtain a value from the 
        microcontroller. It writes a byte of data (in this case 0b00000010 to the I2C address 0x10)
        The Microcontroller is programmed to respond with the IR sensor value once it receives this 
        I2C trigger. 
        @return incoming_data This is the data that is sent from the microcontroller.

        """
        self.i2c_bus.write_byte_data(self.i2c_addr,0x0,0x02)
        incoming_data = self.i2c_bus.read_byte(self.i2c_addr)

        return incoming_data
    


if __name__ == '__main__':
    """
    Entry point of program. It prints the data received from the I2C line.

    """
    i2c = i2c_com()

    while(True):
        print(i2c.generateTelemetry())
        
        time.sleep(0.1)
