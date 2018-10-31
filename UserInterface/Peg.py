#!/usr/bin/env python

import sys
import time
import pygame
import serial

from PyQt4 import QtCore, QtGui
import Controls

pygame.init() #initiaise pygame
controls = pygame.joystick.Joystick(0) # call the joystic controls
clock = pygame.time.Clock()           # intialise pygame refresh and call it clock
controls.init()                        # initialise the controls

arduino =  serial.Serial('/dev/ttyUSB0', 9600,timeout = 1) #connect to the arduino's serial port
time.sleep(2)


class OutputWindow(QtGui.QMainWindow,Controls.Ui_MainWindow):
    def __init__(self, parent=None):
        super(OutputWindow,self).__init__(parent)
        self.setupUi(self)
        self.pushButton.clicked.connect(self.myButtonSlot)
        self.pushButton_2.clicked.connect(self.myButtonSlot2)
        self.passval = passval(self)
        self.passval.valueUpdate.connect(self.handleValue)
        self.passval.valueUpdate2.connect(self.handleValue2)
        self.passval.valueUpdate3.connect(self.handleValue3)

    def myButtonSlot(self):
        self.passval.method()
        
    def myButtonSlot2(self):
        self.passval.close()
        
    def handleValue(self, value):
        self.progressBar.setValue(value)
        QtGui.qApp.processEvents()
        
    def handleValue2(self, value):
        self.progressBar_2.setValue(value)
        QtGui.qApp.processEvents()
        
    def handleValue3(self, value):
        self.progressBar_3.setValue(value)
        QtGui.qApp.processEvents()
        

class passval(QtCore.QObject):
    valueUpdate = QtCore.pyqtSignal(int)
    valueUpdate2 = QtCore.pyqtSignal(int)
    valueUpdate3 = QtCore.pyqtSignal(int)

    def close(self):
        pygame.quit()
        quit()  
        
    
    def method(self):
        EXIT = False                          # set exit to false
        while not EXIT:

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    EXIT = True
                
            old_value1 = controls.get_axis(0)  # X-axis
            old_value2 = controls.get_axis(1)  # Y-axis
            old_value3 = controls.get_axis(3)  # Rudder   ,,,,,,,,,,,
            old_min = -1
            old_max = 1
            new_min = 100
            new_max = 355
                                                # take original values (-1 to 1) and map it to a new range between 100 and 355***
            new_value1 = ( ( old_value1 - old_min ) / (old_max - old_min) ) * (new_max - new_min) + new_min
            new_value2 = ( ( old_value2 - old_min ) / (old_max - old_min) ) * (new_max - new_min) + new_min
            new_value3 = ( ( old_value3 - old_min ) / (old_max - old_min) ) * (new_max - new_min) + new_min

            self.valueUpdate.emit(int(round(new_value1)))
            self.valueUpdate2.emit(int(round(new_value2)))
            self.valueUpdate3.emit(int(round(new_value3)))
            
            a = str(int(round(new_value1)))   
            b = str(int(round(new_value2)))  # converts values from floats to integers and then to strings
            c = str(int(round(new_value3)))
            h = '000'
            
            for x in range(0,11):
                if controls.get_button(x) == 1:
                    h = '0' + str(x+1)
                    if x < 9:
                        h = '00' + str(x+1)
                        
            
            control = ['<' + a,b,c,h + '>']    # save strings in a list
            cstring = ",".join(control)     # convert list to a single string with commas seperating values
                
            #print(cstring)
            arduino.write(cstring)          # print string to shell and write data to arduino with a 0.1ms delay
            time.sleep(0.0001)
                
            if controls.get_button(11) == 1:
                pygame.quit()              # emergency shut down
                quit()
                
            clock.tick(1000)                  # refresh at 1000 fps      

def main():
    app = QtGui.QApplication(sys.argv)
    form = OutputWindow()
    form.show()
    sys.exit(app.exec_())
    
if __name__ == '__main__':
        main()
        
pygame.quit()
quit()                             ## end program if while loop is broken
close()
