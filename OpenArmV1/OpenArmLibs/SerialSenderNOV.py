
'''
=============================================================

/ Engineer:   Pacheck Nicholas
// Date Created: 11/16/23

// Module Description:

// TODO:

1. Add baud rate, zfill, and port communications to insure compatibility across devices
2. embed the self.conenciton into the status bits



// Modifications:
    Date:    Engineer:     Modification Description

=============================================================
'''



import serial.serialutil
from serial import Serial
import keyboard
import numpy as np
import time





class Arduino:


    def __init__(self, port, baud):
        self.verbose = False


        # Serial connection parameters
        self.baud = int(baud)
        self.port = str(port)
        try:
            self.ser = Serial(self.port, self.baud, timeout=0.1)
        except serial.serialutil.SerialException as Permissions:
            print(Permissions, "Probrally Permissions Error, Arduino not connected")



        # init connection status
        self.shaken = False
        self.connected = False # inicial condition for conneciton status

        if self.verbose:
            print(f"BAUD RATE: {self.baud}"
                  f"PORT: {self.port}"
                  f"SERIAL: {self.ser}")
        # The number of characters each substring will have
        self.zfill_num = 4

        #attempt to connect unril connected
        self.connect2Arudino()


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
            data =  bytes(data, 'utf-8')

            #resets both thte input and output buffers for the serial connection established betweem
            #this computer and the micor-controller
            if reset:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()

            #write the data to the serial line
            self.ser.write(data)

        except TypeError as TE:
            print(TE)




        pass


    def inLoopSerialSender(self, requestList, reset):

        '''
                    This function should be called as the main serial sending
                    function as it handles all serial stuff through the ussage of self.class//self.__name__

                    input the requestList length 6 for 6 dof and check for sending emergecies from hgh level
                passes the reset variable to the send funciton descirbed above


                TODO: FEEDBACK FROM ARDUINO

                '''

        # FILL ANGLES WITH ZEROS TO MATCH THE CUSTOM ARUDINO SERIAL PROTOCAL
        formatted_numbers = [str(num).zfill(self.zfill_num) for num in requestList]

        # COMBINE INTO STRING
        finalString = ''.join((formatted_numbers))
        # PARSE FOR CUSTOM SERIAL COMMUNICATIONS, CUSTOM START AND STOP BYTES
        finalString = finalString + 'B'
        if self.verbose: print(f"               Sending final string{finalString}")

        # SEND DATA
        self.send_data(str(finalString), reset)



    def readSerialPort(self):
        my_string = self.ser.readline().decode('utf-8')
        #if self.verbose: print(my_string, "i got", (my_string.decode('utf-8')))
        return str(my_string)
    #return string read from serial bus line
    def connect2Arudino(self):

        try:
            #while no serial connection has been estabished, attempt to estblish connection
            while not self.ser.is_open:

                #open the connection
                self.ser.open()

            if self.ser.is_open:
                if self.verbose: print(" SERIAL CONNECITON ESTABLISHED, BEGINING HANDSHAKE")
                self.connected = True

        except serial.serialutil.SerialException as SerialError:
            print("Serial Error:", SerialError)
            #retry to init serial
            self.__init__(self.port, self.baud)
            pass
        pass
    # Establishes elementary connection between computer and micro controller
    def dual_handshake(self):
        # establish handshake with arduino
        if self.connected:
            self.shaken = self.handshake("SHAKING")
            if self.shaken:
                self.shaken = self.handshake("READY")

        return self.shaken
    # Establishes strong and reliable serial connection, breaking through 2 waiting/testing rooms succesfully
    def handshake(self, myBreakWord):

        # Shake hands with tht earudino until both systms are aligned
        try:

            myBreakWord = str(myBreakWord)

            #read the string from serial bus
            ardu_string = (self.readSerialPort())

            #while the serial bus doesnt contain the ready message
            while not myBreakWord in ardu_string:
                #Send a handshake message as shaking
                self.send_data(myBreakWord, True)
                if self.verbose:
                    print(f"Looking for {myBreakWord}: I got: {ardu_string} from {self.port}")

                time.sleep(1)
                #read the serial buffer again and update stirng for the next loop
                ardu_string = (self.readSerialPort())


            if self.ser is not None: self.shaken = True
            else: self.shaken = False
            print("SHAKING COMPLETED: ", self.shaken, myBreakWord)
            return self.shaken




        except AttributeError as AE:
            print(AE, "probrally no serial device avalaible")
            pass

        pass
    # Defines a singular waitng room, handshake through a break word

    def send_instructionSet(self, instructionsSet, reset):

        for instruction in instructionsSet:

            self.inLoopSerialSender(instruction, reset)
            time.sleep(.3)
            print("im Reading")
            print([self.readSerialPort() for _ in range(7)])





class Switch:
    def __init__(self, key):
        self.value = False
        self.key = str(key)
        keyboard.on_press_key(self.key, self.toggle)

    def toggle(self, event):
        if event.name == self.key:
            self.value = not self.value
            print('Switch is now', self.value)





instructionSet1 = [[0,0,0,0,0,0,1111],
                   [90, 90, 90, 90, 90, 90, 0 ],
                   [ 0, 0, 0, 0, 0, 0, 0 ],
                   [-90, -90, -90, -90, -90, -90, 0 ],
                   [-90, 90, -90, 90, -90, 90, 0 ],
                   [90, -90, -90, 90, -90, 90, 0 ],
                   [0, 0, 0, 0, 0, 0, 0 ]]

if __name__ == "__main__":



    # Init the arudino
    ARDU = Arduino("COM3", 115200)

    # Shake hands with arduino through both waiting rooms, SHAKING and READY
    shaken = ARDU.dual_handshake()


    if shaken:
        print("SHAKING PROCESS DONE")

        ARDU.send_instructionSet(instructionSet1, False)

