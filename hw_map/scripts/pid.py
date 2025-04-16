from collections import deque
import numpy as np

################################# class PID ########################

class PID:
    '''Implement a generic PID controller for the motion control of a mobile robot.
       Given current error measure, it provide two separate methods for returning 
       the output values of linear and angular velocities.
       The initialization method sets all the parameters of the controller. 
       If some components of the P,I,D controller aren't used, the corresponding gains
       can be left set to their default value, which is 0.    
    '''

    def __init__(self,
                 _p_gain_distance=0.0, _p_gain_angle=0.0,
                 _i_gain_distance=0.0, _i_gain_angle=0.0,
                 _i_err_window_len=30, _i_err_dt=0.01,
                 _d_gain_distance=0.0, _d_gain_angle=0.0,
                 _v_lin_max=0.2, _v_ang_max=0.8):
        '''
           Input: all the gains and parameters that are necessary to set up a PID controller.
           _v_max and _angle_max are bounds to the velocities that can be give as output.
           The input arguments are used to define class variables with the same meaning.

           _p_gain_distance: the P gain for the error in the distance component
           _p_gain_angle: the P gain for the error in the angle component
           _i_gain_distance: the I gain for the error in the distance component
           _i_gain_angle: the I gain for the error in the angle component
           _i_err_window_len: the length, as number of error measures, of the time window used
                              to compute the integral for the I part (both for distance and angle)
           _i_err_dt: this is dt, the numeric differential in the numeric computation of the
                       integral ( i_err = \int_t0^t1 err(t)dt )
           _d_gain_distance: the D gain for the error in the distance component
           _d_gain_angle: the D gain for the error in the angle component
        '''
        
        self.p_gain_distance = _p_gain_distance
        self.p_gain_angle = _p_gain_angle

        self.i_gain_distance = _i_gain_distance
        self.i_gain_angle = _i_gain_angle

        self.i_err_window_len = _i_err_window_len 
        self.i_err_dt = _i_err_dt

        # Create two circular buffers to hold the error data for computing integral errors
        # separately for distance and angle. Values are initialized to zero.
        self.i_err_window_data_distance = deque([0] * self.i_err_window_len,
                                                 maxlen=self.i_err_window_len)

        self.i_err_window_data_angle = deque([0] * self.i_err_window_len,
                                              maxlen=self.i_err_window_len)
                
        self.d_gain_distance = _d_gain_distance
        self.d_gain_angle = _d_gain_angle
                
        self.v_max = _v_lin_max
        self.omega_max = _v_ang_max
        
        self.previous_error_distance, self.previous_error_angle = (0, 0)

        
    def update_integral_error_distance(self, err):
        '''The error err is used to update the integral error on distance'''
        self.i_err_window_data_distance.append(err)
        self.i_err_distance = np.sum(self.i_err_window_data_distance) * self.i_err_dt

    def update_integral_error_angle(self, err):
        '''The error err is used to update the integral error on angle'''
        self.i_err_window_data_angle.append(err)
        self.i_err_angle_integral = np.sum(self.i_err_window_data_angle) * self.i_err_dt

    
    def reset_integral_errors(self):
        '''Reset the integral errors, both for distance and angle.
           This is used when a waypoint is reached and a new one shall start.'''
        self.i_err_window_data_distance = deque([0] * self.i_err_window_len,
                                                 maxlen=self.i_err_window_len)

        self.i_err_window_data_angle = deque([0] * self.i_err_window_len,
                                              maxlen=self.i_err_window_len)
        self.i_err_distance = 0
        self.i_err_angle_integral = 0
        
    def bound_v(self, value):
        sign = -1 if value < 0 else 1
        return min(abs(value), self.v_max) * sign
    
    def bound_o(self, value):
        sign = -1 if value < 0 else 1       
        return min(abs(value), self.omega_max) * sign
    
    def get_linear_velocity_P(self, err_d):
        '''Get error in distance and return P component of linear velocity'''
        ##### your code here
        return self.p_gain_distance * err_d

    
    def get_linear_velocity_I(self, err_d):
        '''Get error in distance and return I component of linear velocity'''
        ##### your code here
        self.update_integral_error_distance(err_d)
        err_dist_i = self.i_err_distance
        return self.i_gain_distance * err_dist_i


    def get_linear_velocity_D(self, err_d):
        '''Get error in distance and return D component of linear velocity'''
        ##### your code here
        derivative = (err_d - self.previous_error_distance) / self.i_err_dt
        self.previous_error_distance = err_d
        return self.d_gain_distance * derivative

    def get_linear_velocity(self, err_d, p=0, i=0, d=0):
        return self.bound_v(p * self.get_linear_velocity_P(err_d)+ 
                            i * self.get_linear_velocity_I(err_d)+ 
                            d * self.get_linear_velocity_D(err_d))
    
    def get_angular_velocity_P(self, err_angle):
        '''Get error in angle and return P component of angular velocity'''
        ##### your code here
        return self.p_gain_angle * err_angle


    def get_angular_velocity_I(self, err_angle):
        '''Get error in angle and return I component of angular velocity'''
        ##### your code here
        self.update_integral_error_angle(err_angle)
        err_angle_i = self.i_err_angle_integral
        return self.i_gain_angle * err_angle_i

    def get_angular_velocity_D(self, err_angle):
        '''Get error in angle and return D component of angular velocity'''
        ##### your code here
        derivative = (err_angle - self.previous_error_angle) / self.i_err_dt
        self.previous_error_angle = err_angle
        return self.d_gain_angle * derivative
    
    def get_angular_velocity(self, err_angle, p=0, i=0, d=0):
        return self.bound_o(p * self.get_angular_velocity_P(err_angle)+
                            i * self.get_angular_velocity_I(err_angle)+
                            d * self.get_angular_velocity_D(err_angle))
