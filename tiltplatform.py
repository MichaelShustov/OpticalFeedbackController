           
import serial
import array

class TiltPlatformClass():

    
    def __init__(self, port_name = 'COM5', x_channel = 3, y_channel = 5, 
                 x_range_tup = (80,100), y_range_tup = (80,100)):
        """ Initialization of the object"""
        
        import serial
        import array
        
        self.usbport = port_name
        
        # set digital pins for X and Y coords
        digital_pins = {3,5,6,9,10,11}
        if ((x_channel != y_channel) and (x_channel in digital_pins) and (y_channel in digital_pins)):
            self.x_channel = x_channel
            self.y_channel = y_channel
        else:
            self.x_channel = 3
            self.y_channel = 5
            print('Pins for X and Y are set to default: X - 3, Y - 5 ')
        
        # Set angular ranges for X and Y
        x_min, x_max = x_range_tup
        y_min, y_max = y_range_tup
        
        if ((x_min < x_max) and (x_min in range(181)) and (x_max in range(181))):
            self.x_min = x_min
            self.x_max = x_max
        else:
            self.x_min = 80
            self.x_max = 100
            print('X angles range was set to default')
        
        if ((y_min < y_max) and (y_min in range(181)) and (y_max in range(181))):
            self.y_min = y_min
            self.y_max = y_max
        else:
            self.y_min = 80
            self.y_max = 100
            print('Y angles range was set to default')
            
               
        # initialize serial port
        self.ser = serial.Serial(self.usbport, 9600, timeout=1)
        

        
    def move(self, direction, percent_full_range):

        """Moves the specified servo to the supplied angle"""

        # Arguments:
        # direction
        #    'x' or 'y' ('X','Y' are also valid)
        # percent_full_range
        #  percent of the full range of this axis
        #  i.e. 10% of (80,110) is int((100-80)*0.1 + 80) = 82

        ok = True

        if (((direction == 'x') or (direction == 'X')) and 
           (percent_full_range in range(101))):
        
            servo = self.x_channel
            angle = int((self.x_max-self.x_min)*0.01*percent_full_range + self.x_min)
            
        elif (((direction == 'y') or (direction == 'Y')) and 
             (percent_full_range in range(101))):
        
            servo = self.y_channel
            angle = int((self.y_max-self.y_min)*0.01*percent_full_range + self.y_min)
        else:
            ok = False
        
        # send comand to serial port
        if ok:
            ar = array.array('B', [255, servo, angle]).tobytes()
            self.ser.write(ar)
            # print('ok')
        
        else:
            print('Move parameters (percent or direction) are wrong')
    
    
    
    def close_serial(self):
        self.ser.close()
        print('port released')
    