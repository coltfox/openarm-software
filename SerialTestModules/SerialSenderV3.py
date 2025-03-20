


'''
=============================================================

/ Engineer:   Pacheck Nicholas
// Date Created: 11/16/23
// Module Description:

This module is a test script for establishing relaible serial connections
between a microcontroller and a high level computer, hence this computer.
This partuclar version serves as a test for the handshake stage.



// TODO:

1. Add baud rate, zfill, and port communications to insure compatibility across devices
2. embed the self.conenciton into the status bits being sent over the serial line
3. Implement serial parser on arduino side
4. implemmt start bit and stop bit parity checking
5. solution to reseting arudino if arudino senses that computer is trying to handshake while arduinohas already broken handshak lobby



// Modifications:
    Date:    Engineer:     Modification Description
    11/18/23 Pacheck: Start bits and stop bits added during all message sending and more effeicient and relaible handshake

=============================================================
'''


import serial.serialutil
from serial import Serial
import keyboard
import numpy as np
import time




class Arudino:


    def __init__(self, port, baud, timeout = 0.1):

        self.verbose = True

        # Serial connection parameters
        self.baud = int(baud)
        self.port = str(port)
        self.timeout = timeout

        # Boolean for if a handshake has been made
        self.shaken = False
        # Boolean for if a standard serial connection is made
        self.connected = False

        # Establish standard serial connection
        self.connect2Arudino()


    def connect2Arudino(self):

        '''
        Usage:
            Called upon init of any Arduino instance

        Description:
            Attempts to establish a standard serial connection. This "standard" serial
             connection needs to be established prior to and handshake process is executed
             since the handshake relys on the standard connection.
        '''


        try:
            # Attempt to establish standard serial connection
            self.ser = Serial(self.port, self.baud, timeout=self.timeout)

            #while no serial connection has been estabished, attempt to estblish connection
            while not self.ser.is_open:
                #open the connection
                self.ser.open()

            # Upon breaking from the loop, check to make sute the serial connection is still established
            if self.ser.is_open:
                if self.verbose: print(" SERIAL CONNECITON ESTABLISHED, READY TO HANDSHAKE")
                self.connected = True

        # If this exception is thrown then there is most likely already an established serial connection between the uC and some OTHER devie
        except serial.serialutil.SerialException as SerialError:
            print("Serial Error:", SerialError)
            #retry to init serial
            self.__init__(self.port, self.baud)
            pass
        pass
    # Establishes elementary connection between computer and micro controller

    def send_data(self, data, reset):

        '''Method Description:

            Sends a serial message to micro-controller, if reset boolena i high then the previous message will delete
            itself and be replaced by th enew message, if the reset boolena is low then the previous message will
            remain and the new message will be placed in the serial buffer.

            This is used to update the serial bus line effecinently instead of interpolating the values as they come in
            whcoih would requitee multithreading


        Inputs:
                Data: The string desred to be sent by utf-8 serial protocal over serial line
                Reset: Bool to reset the buffer or not reset the buffer

        Operation:
                Sends data through the established serial connection

        '''

        try:
            # Convert the string type data into bytes such that it can be sent through a serial bus
            data = bytes(data, 'utf-8')

            # resets both thte input and output buffers for the serial connection established betweem
            # this computer and the micor-controller
            if reset:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()

            # write the data to the serial line
            self.ser.write(data)

        except TypeError as TE:
            print(TE)

        pass
    # Sends a string of "data" to a serial buffer, reset chooses if instruction is appended to buffer or becomes the new buffer

    def readSerialPort(self):
        '''

        This method reads one line from the serial monitor and

        :return: The serial string sent, ( one line )
        '''
        my_string = self.ser.readline().decode('utf-8')
        #if self.verbose: print(my_string, "i got", (my_string.decode('utf-8')))
        return str(my_string)
    #return string read from serial bus line





    def handshake(self, keyword):

        # inicial condition for serial message
        echoMessage = str("poop scoop")
        # type checking
        keyword = str(keyword)

        # Send shaking keyword, read echo, if echo == keyword, break
        while keyword not in echoMessage:

            if self.verbose:
                print(f" SHAKING: I sent {keyword}, I recived {echoMessage}")

            # Send the shaking keyword through serial connection
            self.send_data(keyword, True)

            # Read response from micro-controller, updating flag
            echoMessage = str(self.readSerialPort())

            time.sleep(self.timeout)
        if self.verbose: print(f" SHAKING: I sent {keyword}, I recived {echoMessage}")

        #breaks

        # If the code makes it to this point and self.ser still exists, handshake has been established
        if self.ser is not None:
            print(" SHAKING COMPLETED SUCCESSFULLY ")
            self.shaken = True
        else:
            self.shaken = False
        print(f"SHAKING STATUS: {self.shaken} for {keyword} SHAKE")
        return self.shaken


if __name__ == "__main__":

    # instance of arduino
    ardu = Arudino('COM3', 115200)

    # shake hands with arduino
    ardu.handshake("SHAKING")







