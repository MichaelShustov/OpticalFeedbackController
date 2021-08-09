# Unit implements class for a PID controller

import collections

class PIDClass():
    
    #######################################################################################
    def __init__(self, coefs_tup, setpoint_value, range_tup, integration_samples = 5, diff_filter_samples = 4):
        """Constructor of the PID controller
        coefs_tup - tuple of (Kp, Ki, Kd)
        setpoint_value - setpoint
        range_tup - tuple of maximal and minil output values of PID controler (val_min,val_max)
        integration_samples - number of samples for integration
        diff_filter_samples - number of samples to filter signal before differencial member (to remove noise)"""
        
        self.started = False
        self.Kp, self.Ki, self.Kd = coefs_tup
        
        if integration_samples < 3:
            integration_samples = 3
            print('Integration samples number is set default 3')
            
        self.integr_deque = collections.deque([(0,0)]* integration_samples, maxlen = integration_samples)
        
        if diff_filter_samples < 2:
            diff_filter_samples = 2
            print('Diff filter samples number is set to default 2')

        self.diff_deque = collections.deque([(0,0)]* diff_filter_samples, maxlen = diff_filter_samples)
        
        self.setpoint_value = setpoint_value
        
        self.min_value, self.max_value = range_tup
        if self.min_value >= self.max_value:
            self.min_value = 0
            self.max_value = 1
            print('Values range is set to default (0,1)')
        
        
        
    ###########################################################################    
    def setpoint(self, setpoint_value):
        """ Changes setpoint value """
        
        self.setpoint_value = setpoint_value
    
   
    ##########################################################################
    def get_filtered_derivative(self):
        """ Calculates derivatives and applies the median filter to them"""
        
        # unpack deque of tuples (time, value) into 2 lists [time], [values]
        time_val_list = list(self.diff_deque)
        unzip = list(zip(*time_val_list))
        actual_time_list = unzip[0]
        actual_val_list = unzip[1]
        
        deriv_list = list()
        for i in range(len(actual_time_list)-1):
            
            #current errors
            val0e = actual_val_list[i] - self.setpoint_value
            val1e = actual_val_list[i+1] - self.setpoint_value
            
            #current derivative
            dt_i = actual_time_list[i+1]-actual_time_list[i]
            if dt_i > 0:
                deriv_i = (val1e - val0e)/(dt_i)
            else:
                deriv_i = 0
            
            deriv_list.append(deriv_i)
        
        deriv_list.sort()
        
        if len(deriv_list) == 1:
            deriv_filtered = deriv_list[0]
        else:
            k = (len(deriv_list)-1)//2
            deriv_filtered = deriv_list[k]
        
        return deriv_filtered
        
    
    #########################################################################
    def get_integral(self):
        """ Calculates integral of errors"""
        
        # unpack deque of tuples (time, value) into 2 lists [time], [values]
        time_val_list = list(self.integr_deque)
        unzip = list(zip(*time_val_list))
        actual_time_list = unzip[0]
        actual_val_list = unzip[1]
        
        # calculate integral error
        integral = 0
        for i in range(len(actual_time_list)-1):
            
            #current errors
            val0e = actual_val_list[i] - self.setpoint_value
            val1e = actual_val_list[i+1] - self.setpoint_value
            
            integral = integral + ((val0e + val1e )/ 2)*(actual_time_list[i+1]-actual_time_list[i])
        
        return integral
        
    ####################################################################3
    def control(self, actual_time, actual_value):
        """main method which returns the control signal
            actual_time - time when the measurement was done (preferably in ms)
            actula_value - value of the measured parameter """
        
        time_val_tup = (actual_time, actual_value)
        self.integr_deque.append(time_val_tup)
        self.diff_deque.append(time_val_tup)
        
        derivative = 0
        integral = 0
        
        if self.Kd != 0:
            derivative = self.get_filtered_derivative()
            
        
        if self.Ki != 0:
            integral = self.get_integral()
                                             
        proportional_e = actual_value - self.setpoint_value
                                             
        
        pid_result = self.Kp  *proportional_e + self.Ki * integral + self.Kd * derivative
        if pid_result < self.min_value:
            pid_result = self.min_value
        elif pid_result > self.max_value:
            pid_result = self.max_value
        
        return pid_result
        
        
    ##############################################################3
    def set_coefs(self, coefs_tup):
        """Set new PID coefficients
            coefs_tup - tuple of (Kp, Ki, Kd)"""
        
        self.Kp, self.Ki, self.Kd = coefs_tup
