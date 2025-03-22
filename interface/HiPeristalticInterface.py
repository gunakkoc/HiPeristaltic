# Copyright 2025 Gun Deniz Akkoc
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#    http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# https://github.com/gunakkoc/HiPeristaltic

from threading import Event, Thread, Lock
from datetime import timedelta
from time import sleep
import numpy as np
import inspect
import serial
import logging
import toml
import sys
import os

class Pump():

    ### Public variables

    uL_per_rev: float = 60.0 #calibration factor
    direction_default: str = 'CW'

    ### Constants
    
    _max_rpm: float = 100
    _motor_min_step_interval_us: np.uint32 = 24 #in microseconds
    _motor_max_step_interval: np.uint32 = np.uint32(np.iinfo(np.uint32).max - 1024) #in ticks, absolute max 2^32 -1
    _min_to_us: int = 60000000

    ### Private variables

    _gear_ratio: float = 1.0 #revolutions of the motor to produce one revolution of the pump
    _motor_running: bool = False
    _motor_ind: int = 0
    _motor_dir: bool = True
    _motor_dir_inverse: bool = False
    _motor_base_spr: int = 200 #base steps per revolution, typically 200 for stepper motors
    _motor_enabled: bool = False
    _motor_finite_mode: bool = False
    _motor_step_interval: np.uint32 = 2000
    _motor_usteps: int = 1
    _motor_min_step_interval: np.uint32 = 24 #in ticks
    _motor_max_steps: np.uint32 = np.iinfo(np.uint32).max - 2
    _motor_var_ustep_support: bool = False #variable microstepping support
    _motor_max_ustep_exp: int = 8 #max microstepping, exponent of 2
    _motor_min_ustep_exp: int = 0 #min microstepping, exponent of 2
    _min_to_mcu_ticks: np.float64 = 0 #minutes to mcu ticks conversion factor
    _sub_us_divider: np.float64 = 1
    _event_motor_stopped: Event
    _func_pump_send_cmd: callable

    def __init__(self, motor_ind: int, sub_us_divider: np.float64 = None,
                 uL_per_rev: float = None, gear_ratio: float = None, motor_usteps: int = None, max_rpm: float = None, direction_default: str = None,
                 direction_inverse: bool = None,
                 func_pump_send_cmd = None):
        
        self._motor_ind = motor_ind
        if not (sub_us_divider is None):
            self._sub_us_divider = sub_us_divider
        self._apply_sub_us_divider(self._sub_us_divider)
        if not (uL_per_rev is None):
            self.uL_per_rev = uL_per_rev
        if not (gear_ratio is None):
            self._gear_ratio = gear_ratio
        if not (motor_usteps is None):
            self._motor_usteps = motor_usteps
        if not (max_rpm is None):
            self._max_rpm = max_rpm
        if not (direction_default is None):
            self.direction_default = direction_default
        if not (direction_inverse is None):
            self._motor_dir_inverse = direction_inverse
        self._func_pump_send_cmd = func_pump_send_cmd

        self._event_motor_stopped = Event()
        # self._read_initial_variables() #this is done in the HiPeristalticInterface class

    ### Public functions
    
    def get_max_rpm(self)->float:
        if self._motor_var_ustep_support: #assume min microstepping for the fastest speed
            max_rpm_2 = self._step_interval_to_rpm_precise(self._motor_min_step_interval,
                                                           self._motor_base_spr * self._gear_ratio * np.power(2,self._motor_min_ustep_exp))
        else:
            max_rpm_2 = self._step_interval_to_rpm(self._motor_min_step_interval)
        return min(self._max_rpm, max_rpm_2)
    
    def get_min_rpm(self)->float:
        if self._motor_var_ustep_support: #assume max microstepping for the slowest speed
            min_rpm = self._step_interval_to_rpm_precise(self._motor_max_step_interval,
                                                         self._motor_base_spr * self._gear_ratio * np.power(2,self._motor_max_ustep_exp))
        else:
            min_rpm = self._step_interval_to_rpm(self._motor_max_step_interval)
        return min_rpm
    
    ### Public pump functions

    def get_running(self)->bool:
        return self._motor_running
    
    def get_flow_rate_uLpersec(self)->float:
        rpm = self._step_interval_to_rpm(self._motor_step_interval)
        flow_rate_uLpersec = self.rpm_to_flow_rate_uLpersec(rpm)
        return flow_rate_uLpersec
    
    def set_flow_rate_uLpersec(self, flow_rate_uLpersec: float)->bool: #works even while running
        rpm = self.flow_rate_uLpersec_to_rpm(flow_rate_uLpersec)
        return self._motor_change_rpm(rpm)
    
    def get_remaining_volume_uL(self)->float:
        return (self._get_m_steps() / self._calc_spr()) * self.uL_per_rev
    
    def get_remaining_time(self)->timedelta:
        return timedelta(seconds=(self.get_remaining_volume_uL()) / self.get_flow_rate_uLpersec())
    
    def get_target_volume_uL(self)->float:
        return (self._get_m_target_steps() / self._calc_spr()) * self.uL_per_rev
    
    def rpm_to_flow_rate_uLpersec(self, rpm: float)->float:
        return (rpm / 60) * self.uL_per_rev #one can set calibration uLperRev as 60 uL/rev then rpm = flow_rate(uL/s)
    
    def flow_rate_uLpersec_to_rpm(self, flow_rate_uLpersec: float)->float:
        return (flow_rate_uLpersec / self.uL_per_rev) * 60
    
    def get_max_volume_uL(self)->float:
        if self._motor_var_ustep_support:
            usteps = np.power(2,self._motor_min_ustep_exp) #assume min microstepping for max volume
        else:
            usteps = self._motor_usteps
        calc_spr = self._motor_base_spr * usteps * self._gear_ratio
        return (self._motor_max_steps / calc_spr) * self.uL_per_rev
    
    def get_min_volume_uL(self)->float:
        if self._motor_var_ustep_support:
            usteps = np.power(2,self._motor_max_ustep_exp) #assume max microstepping for min volume
        else:
            usteps = self._motor_usteps
        calc_spr = self._motor_base_spr * usteps * self._gear_ratio
        return (1 / calc_spr) * self.uL_per_rev
    
    def get_max_flow_rate_uLpersec(self)->float:
        return self.rpm_to_flow_rate_uLpersec(self.get_max_rpm())
    
    def get_min_flow_rate_uLpersec(self)->float:
        return self.rpm_to_flow_rate_uLpersec(self.get_min_rpm())
    
    def pump_volume_rpm(self, target_volume_uL: float, rpm: float, direction: str = None, blocking: bool = False)->bool:
        if target_volume_uL <= 0:
            return False
        revs = target_volume_uL / self.uL_per_rev #number of revolutions
        max_revs = self._motor_max_steps / self._calc_spr()
        if revs >= max_revs:
            return False
        if rpm > self._max_rpm:
            return False
        if rpm > self.get_max_rpm():
            return False
        if rpm < self.get_min_rpm():
            return False
        dir = self._dir_str2bool(direction)
        result = self._motor_start_finite(rpm=rpm,dir=dir,revs=revs,blocking=blocking)
        return result

    def pump_volume(self, target_volume_uL: float, flow_rate_uLpersec: float, direction: str = None, blocking: bool = False)->bool:
        rpm = (flow_rate_uLpersec / self.uL_per_rev) * 60 #revolutions per minute
        result = self.pump_volume_rpm(target_volume_uL=target_volume_uL,rpm=rpm,direction=direction,blocking=blocking)
        return result

    def pump_timedelta(self, duration: timedelta, flow_rate_uLpersec: float, direction: str = None, blocking: bool = False)->bool:
        if duration.total_seconds() <= 0:
            return False
        vol_uL = flow_rate_uLpersec * duration.total_seconds()
        return self.pump_volume(target_volume_uL=vol_uL,flow_rate_uLpersec=flow_rate_uLpersec,direction=direction,blocking=blocking)
    
    def pump_timedelta_rpm(self, duration: timedelta, rpm: float, direction: str = None, blocking: bool = False)->bool:
        if duration.total_seconds() <= 0:
            return False
        vol_uL = (rpm / 60) * self.uL_per_rev * duration.total_seconds()
        return self.pump_volume_rpm(target_volume_uL=vol_uL,rpm=rpm,direction=direction,blocking=blocking)
    
    def pump_duration(self, duration_sec: float, flow_rate_uLpersec: float, direction: str = None, blocking: bool = False)->bool:
        if duration_sec <= 0:
            return False
        vol_uL = flow_rate_uLpersec * duration_sec
        return self.pump_volume(target_volume_uL=vol_uL,flow_rate_uLpersec=flow_rate_uLpersec,direction=direction,blocking=blocking)
    
    def pump_duration_rpm(self, duration_sec: float, rpm: float, direction: str = None, blocking: bool = False)->bool:
        if duration_sec <= 0:
            return False
        vol_uL = (rpm / 60) * self.uL_per_rev * duration_sec
        return self.pump_volume_rpm(target_volume_uL=vol_uL,rpm=rpm,direction=direction,blocking=blocking)
    
    def pump_continuous_rpm(self, rpm: float, direction: str)->bool:
        if rpm > self._max_rpm:
            return False
        if rpm > self.get_max_rpm():
            return False
        if rpm < self.get_min_rpm():
            return False
        dir = self._dir_str2bool(direction)
        result = self._motor_start_continuous(rpm=rpm,dir=dir)
        return result

    def pump_continuous(self, flow_rate_uLpersec: float, direction: str)->bool:
        rpm = (flow_rate_uLpersec / self.uL_per_rev) * 60 #revolutions per minute
        result = self.pump_continuous_rpm(rpm=rpm,direction=direction)
        return result
    
    def pump_stop(self)->float: #return remaining volume in uL
        self._motor_stop()
        return self.get_remaining_volume_uL()

    def pump_resume(self,blocking:bool = False)->float : #return remaining volume in uL
        remaining_volume_uL = self.get_remaining_volume_uL()
        self._motor_resume(blocking=blocking)
        return remaining_volume_uL
    
    ### Private functions

    def _apply_sub_us_divider(self,sub_us_divider: np.float64)->bool:
        self._min_to_mcu_ticks = sub_us_divider * self._min_to_us #minutes to mcu ticks conversion factor
        self._motor_min_step_interval = self._motor_min_step_interval_us * sub_us_divider
        return True

    def _step_interval_to_rpm(self, step_interval: int)->float:
        return self._min_to_mcu_ticks / (step_interval * self._calc_spr())
    
    def _rpm_to_step_interval(self, rpm: float)->np.uint32:
        return np.uint32(np.round(self._min_to_mcu_ticks / (rpm * self._calc_spr())))
    
    def _motor_change_rpm(self, rpm: float)->bool:
        if rpm < 0:
            return False
        if rpm >= self._max_rpm:
            return False
        if rpm == 0:
            self._motor_stop()
            return True
        initially_running = self._motor_running
        if self._motor_running:
            self._motor_stop()
        if self._motor_finite_mode: #for finite mode
            revs = self._get_m_steps() / self._calc_spr() #remaining steps
            target_revs = self._get_m_target_steps() / self._calc_spr() #target steps
            if self._motor_var_ustep_support:
                optimal_ustep_exp = self._calc_finite_optimal_usteps_exp(base_spr=self._motor_base_spr,gear_ratio=self._gear_ratio,rpm=rpm)
                if optimal_ustep_exp < 0: #not possible to achieve the rpm with any microstepping
                    return False
                self._set_m_usteps_exp(optimal_ustep_exp)
            spr = self._calc_spr() #get new spr
            step_interval = self._rpm_to_step_interval_precise(rpm,spr) #get new step interval
            if step_interval < self._motor_min_step_interval:
                return False
            if step_interval > self._motor_max_step_interval: #also implies step_interval fits into uint32
                return False
            self._set_m_step_interval(step_interval) #set new step interval
            self._set_m_steps(self._revs_to_steps_precise(revs,spr)) #set new number of steps
            self._set_m_target_steps(self._revs_to_steps_precise(target_revs,spr)) #set new target number of steps
            if initially_running:
                self._motor_resume() #start again if was initially running
        if not self._motor_finite_mode: #for continous mode
            if self._motor_var_ustep_support:
                optimal_ustep_exp = self._calc_cont_optimal_usteps_exp(base_spr=self._motor_base_spr,gear_ratio=self._gear_ratio,rpm=rpm,revs=revs)
                if optimal_ustep_exp < 0: #not possible to achieve the rpm with any microstepping
                    return False
                self._set_m_usteps_exp(optimal_ustep_exp)
            spr = self._calc_spr() #get new spr
            step_interval = self._rpm_to_step_interval_precise(rpm,spr)
            if step_interval < self._motor_min_step_interval:
                return False
            if step_interval > self._motor_max_step_interval: #also implies step_interval fits into uint32
                return False
            self._set_m_step_interval(step_interval)
            if initially_running:
                self._motor_resume()
        return True

    def _motor_start_continuous(self,rpm,dir=True):
        if self._motor_running:
            return False
        if rpm <= 0:
            return False
        if rpm >= self._max_rpm:
            return False
        if self._motor_var_ustep_support:
            optimal_ustep_exp = self._calc_cont_optimal_usteps_exp(base_spr=self._motor_base_spr,gear_ratio=self._gear_ratio,rpm=rpm)
            if optimal_ustep_exp < 0: #not possible to achieve the rpm with any microstepping
                return False
            self._set_m_usteps_exp(optimal_ustep_exp)
        spr = self._calc_spr()
        step_interval = self._rpm_to_step_interval_precise(rpm,spr)
        if step_interval < self._motor_min_step_interval:
            return False
        if step_interval > self._motor_max_step_interval: #also implies step_interval fits into uint32
            return False
        self._set_m_running(False)
        self._set_m_enabled(True)
        self._set_m_dir(dir)
        self._set_m_finite_mode(0) #0 for continuous mode, 1 for finite steps
        self._set_m_step_interval(step_interval)
        self._set_m_steps(1) #any value > 0
        self._set_m_running(True)
        return True
    
    def _motor_start_finite(self,rpm,dir,revs,blocking=True):
        if self._motor_running:
            return False
        if rpm <= 0:
            return False
        if rpm >= self._max_rpm:
            return False
        if self._motor_var_ustep_support:
            optimal_ustep_exp = self._calc_finite_optimal_usteps_exp(base_spr=self._motor_base_spr,gear_ratio=self._gear_ratio,rpm=rpm,revs=revs)
            if optimal_ustep_exp < 0: #even with min ustep, max number of steps is exceeded or step delay out of range
                return False
            self._set_m_usteps_exp(optimal_ustep_exp)
        spr = self._calc_spr()
        step_interval = self._rpm_to_step_interval_precise(rpm,spr)
        if step_interval < self._motor_min_step_interval:
            return False
        if step_interval > self._motor_max_step_interval: #also implies step_interval fits into uint32
            return False
        step_count = self._revs_to_steps_precise(revs,spr)
        if step_count > self._motor_max_steps: #also implies step_count fits into uint32
            return False
        self._set_m_running(False)
        self._set_m_enabled(True)
        self._set_m_dir(dir)
        self._set_m_finite_mode(1) #0 for continuous mode, 1 for finite steps
        self._set_m_step_interval(step_interval)
        self._set_m_steps(step_count)
        self._set_m_running(True)

        if blocking:
            self._event_motor_stopped.wait()
            self._event_motor_stopped.clear()
        return True
    
    def _motor_stop(self):
        self._set_m_running(False)
        return True
    
    def _motor_resume(self, blocking=False):
        self._set_m_running(True)
        if blocking and (self._motor_finite_mode == 1):
            self._event_motor_stopped.wait()
            self._event_motor_stopped.clear()
        return True
    
    def _rpm_to_step_interval_precise(self,rpm,spr)->np.uint64:
        return np.uint64(np.round(np.float64(self._min_to_mcu_ticks) / (np.float64(rpm) * np.float64(spr))))
    
    def _step_interval_to_rpm_precise(self,step_interval,spr)->np.float64:
        return np.float64(self._min_to_mcu_ticks) / (np.float64(step_interval) * np.float64(spr))

    def _revs_to_steps_precise(self,revs,spr)->np.uint64:
        return np.uint64(np.round(np.float64(revs) * np.float64(spr)))

    def _calc_cont_optimal_usteps_exp(self,base_spr,gear_ratio,rpm):
        min_err = np.inf #or np.finfo(np.float64).max
        optimal_ustep_exp = -1
        for ustep_exp in range(self._motor_min_ustep_exp, self._motor_max_ustep_exp+1):
            spr = base_spr * np.power(2,ustep_exp) * gear_ratio
            calc_step_interval = self._rpm_to_step_interval_precise(rpm,spr)
            if calc_step_interval > self._motor_max_step_interval:
                continue
            elif calc_step_interval < self._motor_min_step_interval:
                break #calc_step_interval will get smaller with increased ustep_exp
            calc_rpm = self._step_interval_to_rpm_precise(calc_step_interval,spr)
            err = np.abs(rpm - calc_rpm)
            if err < min_err:
                min_err = err
                optimal_ustep_exp = ustep_exp
        return optimal_ustep_exp

    def _calc_finite_optimal_usteps_exp(self,base_spr,gear_ratio,rpm,revs):
        min_err = np.inf #or np.finfo(np.float64).max
        optimal_ustep_exp = -1
        for ustep_exp in range(self._motor_min_ustep_exp,self._motor_max_ustep_exp+1):
            spr = base_spr * np.power(2,ustep_exp) * gear_ratio
            calc_step_interval = self._rpm_to_step_interval_precise(rpm,spr)
            if calc_step_interval > self._motor_max_step_interval:
                continue
            elif calc_step_interval < self._motor_min_step_interval:
                # continue
                break #calc_step_interval will get smaller with increased ustep_exp
            calc_nstep = self._revs_to_steps_precise(revs,spr)
            if calc_nstep > np.uint64(self._motor_max_steps):
                break #calc_nstep will get larger with increased ustep_exp
            calc_rpm = self._step_interval_to_rpm_precise(calc_step_interval,spr)
            err = np.abs(rpm - calc_rpm)
            if err < min_err:
                min_err = err
                optimal_ustep_exp = ustep_exp
        return optimal_ustep_exp
    
    def _calc_spr(self)->float:
        return self._motor_base_spr * self._motor_usteps * self._gear_ratio
    
    def _read_initial_variables(self):
        self._get_m_var_ustep_support()
        self._get_m_running()
        self._get_m_enabled()
        self._get_m_dir()
        self._get_m_step_interval()
        self._get_m_finite_mode()
        self._get_m_usteps_exp()
    
    ### Signals from the MCU (i.e., end of motor task)
    
    def _signal_m_stopped(self)->bool:
        #called by the HiPeristalticInterface class, pointed per Pump class after initalization
        self._motor_running = False
        self._event_motor_stopped.set()
        return True
    
    ### Parameters below are to be in sync with MCU

    def _get_m_running(self)->bool:
        result = self._pump_send_cmd(fnc_name=inspect.stack()[0][3])
        self._motor_running = bool(result)
        return result

    def _set_m_running(self, val)->bool:
        if bool(val) == bool(self._motor_running):
            return True
        if bool(val):
            self._event_motor_stopped.clear()
        else:
            self._event_motor_stopped.set()
        result = self._pump_send_cmd(fnc_name=inspect.stack()[0][3], val=val)
        if result:
            self._motor_running = bool(val)
        return result
    
    def _get_m_enabled(self)->bool:
        result = self._pump_send_cmd(fnc_name=inspect.stack()[0][3])
        self._motor_enabled = bool(result)
        return result

    def _set_m_enabled(self, val)->bool:
        if bool(val) == bool(self._motor_enabled):
            return True
        result = self._pump_send_cmd(fnc_name=inspect.stack()[0][3], val=val)
        if result:
            self._motor_enabled = bool(val)
        if self._motor_enabled == False:
            self._motor_stop()
        return result
    
    def _get_m_dir(self)->bool:
        result = self._pump_send_cmd(fnc_name=inspect.stack()[0][3])
        if self._motor_dir_inverse:
            result = not bool(result)
        self._motor_dir = bool(result)
        return result

    def _set_m_dir(self, val)->bool:
        if bool(val) == bool(self._motor_dir):
            return True
        if self._motor_dir_inverse:
            val = not bool(val)
        result = self._pump_send_cmd(fnc_name=inspect.stack()[0][3], val=val)
        if result:
            self._motor_dir = val
        return result
    
    def _get_m_step_interval(self)->np.uint32:
        result = self._pump_send_cmd(fnc_name=inspect.stack()[0][3])
        self._motor_step_interval = result
        return result
    
    def _set_m_step_interval(self, val)->bool:
        if np.uint32(val) == np.uint32(self._motor_step_interval):
            return True
        result = self._pump_send_cmd(fnc_name=inspect.stack()[0][3], val=val)
        if result:
            self._motor_step_interval = val
        return result
    
    def _get_m_finite_mode(self)->np.uint8:
        result = self._pump_send_cmd(fnc_name=inspect.stack()[0][3])
        self._motor_finite_mode = result
        return result

    def _set_m_finite_mode(self, val)->bool:
        #0 for continuous mode, 1 for finite steps
        if np.uint8(val) == np.uint8(self._motor_finite_mode):
            return True
        result = self._pump_send_cmd(fnc_name=inspect.stack()[0][3], val=val)
        if result:
            self._motor_finite_mode = val
        return result
    
    def _get_m_var_ustep_support(self)->bool: #variable microstepping support
        result = self._pump_send_cmd(fnc_name=inspect.stack()[0][3])
        self._motor_var_ustep_support = bool(result)
        return result
    
    def _get_m_usteps_exp(self)->np.uint8: #microstepping exponent of 2
        result = self._pump_send_cmd(fnc_name=inspect.stack()[0][3])
        self._motor_usteps = np.power(2,np.uint32(result))
        return result
    
    def _set_m_usteps_exp(self,val)->bool: #microstepping exponent of 2
        if not self._motor_var_ustep_support:
            return False
        if self._motor_usteps == np.power(2,np.uint32(val)):
            return True
        result = self._pump_send_cmd(fnc_name=inspect.stack()[0][3], val=val)
        if result:
            self._motor_usteps = np.power(2,np.uint32(val))
        return result
    
    ### Parameters below are not to be in sync with MCU

    def _get_m_steps(self)->np.uint32:
        result = self._pump_send_cmd(fnc_name=inspect.stack()[0][3])
        return result
    
    def _set_m_steps(self, val)->bool:
        result = self._pump_send_cmd(fnc_name=inspect.stack()[0][3], val=val)
        return result

    def _get_m_target_steps(self)->np.uint32:
        result = self._pump_send_cmd(fnc_name=inspect.stack()[0][3])
        return result
    
    def _set_m_target_steps(self, val)->bool:
        result = self._pump_send_cmd(fnc_name=inspect.stack()[0][3], val=val)
        return result
    
    ### Private pump functions

    def _dir_str2bool(self,direction:str = None)->bool:
        try:
            if direction is None:
                direction = self.direction_default
            elif len(direction) == 0:
                direction = self.direction_default
            
            if direction.lower() == 'cw':
                return True
            elif direction.lower() == 'clockwise':
                return True
            elif direction.lower() == 'ccw':
                return False
            elif direction.lower() == 'counterclockwise':
                return False
            elif direction.lower() == 'counter-clockwise':
                return False
            elif direction.lower() == 'anticlockwise':
                return False
            elif direction.lower() == 'anti-clockwise':
                return False
            elif direction.lower() == 'acw':
                return False
            else:
                return True
        except:
            return True

    def _volume_uL_to_revs(self, volume_uL: float)->float:
        return volume_uL / self.uL_per_rev

    def _revs_to_volume_uL(self, revs: float)->float:
        return revs * self.uL_per_rev
    
    def _pump_send_cmd(self, fnc_name:str, val=None)->bool:
        #_m_ serves as a wildcard for motor index, replaced with _m0_, _m1_, _m2_ etc.
        #_set_m_running becomes set_m0_running, set_m1_running, set_m2_running etc. depending on motor index
        #then looked up in the command map (_cmd_map of HiPeristalticInterface) to get the command index and the variable type
        #_func_pump_send_cmd is a function pointer to the self._send_cmd_from_table of HiPeristalticInterface class
        #get functions ignore the val argument
        cmd_str = fnc_name.lstrip("_").replace("_m_",f"_m{self._motor_ind}_")
        return self._func_pump_send_cmd(cmd_str, val)
    
class HiPeristalticInterface():
    status: str = "Disconnected"
    pump_count: int = 4
    pumps: list[Pump] = []
    config: dict = None

    ### Private variables
    _serial_com: serial.Serial = None
    _serial_port: str = "COM13"
    _serial_baudrate: int = 115200
    _serial_inter_byte_timeout_s: float = 0.5 #seconds
    _rx_buffer: bytearray
    _tx_buffer: bytearray
    _lock_send: Lock
    _lock_config: Lock
    _event_ack_rcv: Event
    _event_signal_booted_rcv: Event
    _event_msg_rcv: Event
    _event_msg_processed: Event
    _thread_msg_rcv: Thread = None
    _rx_error_cnt: int = 0
    _rx_total_error_cnt: int = 0
    _last_config_fpath: str = None

    _sub_us_divider: np.float64 = 1

    _MSG_LEN: int = 6 #number of bytes in a message
    _ARG_LEN: int = 4 #number of bytes of an argument in a message

    _rcv_msg_table:dict[np.uint8,callable] = {}


    class CommandStructure():
        cmd_ind: np.uint8 = 0
        var_type: type = np.uint8
        def __init__(self, cmd_ind: np.uint8, var_type:type):
            self.cmd_ind = cmd_ind
            self.var_type = var_type
            
    _cmd_map:dict[str,CommandStructure] = {}
    _cmd_map['get_m0_running'] = CommandStructure(cmd_ind=0, var_type=np.uint8)
    _cmd_map['set_m0_running'] = CommandStructure(cmd_ind=1, var_type=np.uint8)
    _cmd_map['get_m0_steps'] = CommandStructure(cmd_ind=2, var_type=np.uint32)
    _cmd_map['set_m0_steps'] = CommandStructure(cmd_ind=3, var_type=np.uint32)
    _cmd_map['get_m0_target_steps'] = CommandStructure(cmd_ind=4, var_type=np.uint32)
    _cmd_map['set_m0_target_steps'] = CommandStructure(cmd_ind=5, var_type=np.uint32)
    _cmd_map['get_m0_step_interval'] = CommandStructure(cmd_ind=6, var_type=np.uint32)
    _cmd_map['set_m0_step_interval'] = CommandStructure(cmd_ind=7, var_type=np.uint32)
    _cmd_map['get_m0_finite_mode'] = CommandStructure(cmd_ind=8, var_type=np.uint8)
    _cmd_map['set_m0_finite_mode'] = CommandStructure(cmd_ind=9, var_type=np.uint8)
    _cmd_map['get_m0_dir'] = CommandStructure(cmd_ind=10, var_type=np.uint8)
    _cmd_map['set_m0_dir'] = CommandStructure(cmd_ind=11, var_type=np.uint8)
    _cmd_map['get_m0_enabled'] = CommandStructure(cmd_ind=12, var_type=np.uint8)
    _cmd_map['set_m0_enabled'] = CommandStructure(cmd_ind=13, var_type=np.uint8)
    _cmd_map['get_m1_running'] = CommandStructure(cmd_ind=14, var_type=np.uint8)
    _cmd_map['set_m1_running'] = CommandStructure(cmd_ind=15, var_type=np.uint8)
    _cmd_map['get_m1_steps'] = CommandStructure(cmd_ind=16, var_type=np.uint32)
    _cmd_map['set_m1_steps'] = CommandStructure(cmd_ind=17, var_type=np.uint32)
    _cmd_map['get_m1_target_steps'] = CommandStructure(cmd_ind=18, var_type=np.uint32)
    _cmd_map['set_m1_target_steps'] = CommandStructure(cmd_ind=19, var_type=np.uint32)
    _cmd_map['get_m1_step_interval'] = CommandStructure(cmd_ind=20, var_type=np.uint32)
    _cmd_map['set_m1_step_interval'] = CommandStructure(cmd_ind=21, var_type=np.uint32)
    _cmd_map['get_m1_finite_mode'] = CommandStructure(cmd_ind=22, var_type=np.uint8)
    _cmd_map['set_m1_finite_mode'] = CommandStructure(cmd_ind=23, var_type=np.uint8)
    _cmd_map['get_m1_dir'] = CommandStructure(cmd_ind=24, var_type=np.uint8)
    _cmd_map['set_m1_dir'] = CommandStructure(cmd_ind=25, var_type=np.uint8)
    _cmd_map['get_m1_enabled'] = CommandStructure(cmd_ind=26, var_type=np.uint8)
    _cmd_map['set_m1_enabled'] = CommandStructure(cmd_ind=27, var_type=np.uint8)
    _cmd_map['get_m2_running'] = CommandStructure(cmd_ind=28, var_type=np.uint8)
    _cmd_map['set_m2_running'] = CommandStructure(cmd_ind=29, var_type=np.uint8)
    _cmd_map['get_m2_steps'] = CommandStructure(cmd_ind=30, var_type=np.uint32)
    _cmd_map['set_m2_steps'] = CommandStructure(cmd_ind=31, var_type=np.uint32)
    _cmd_map['get_m2_target_steps'] = CommandStructure(cmd_ind=32, var_type=np.uint32)
    _cmd_map['set_m2_target_steps'] = CommandStructure(cmd_ind=33, var_type=np.uint32)
    _cmd_map['get_m2_step_interval'] = CommandStructure(cmd_ind=34, var_type=np.uint32)
    _cmd_map['set_m2_step_interval'] = CommandStructure(cmd_ind=35, var_type=np.uint32)
    _cmd_map['get_m2_finite_mode'] = CommandStructure(cmd_ind=36, var_type=np.uint8)
    _cmd_map['set_m2_finite_mode'] = CommandStructure(cmd_ind=37, var_type=np.uint8)
    _cmd_map['get_m2_dir'] = CommandStructure(cmd_ind=38, var_type=np.uint8)
    _cmd_map['set_m2_dir'] = CommandStructure(cmd_ind=39, var_type=np.uint8)
    _cmd_map['get_m2_enabled'] = CommandStructure(cmd_ind=40, var_type=np.uint8)
    _cmd_map['set_m2_enabled'] = CommandStructure(cmd_ind=41, var_type=np.uint8)
    _cmd_map['get_m3_running'] = CommandStructure(cmd_ind=42, var_type=np.uint8)
    _cmd_map['set_m3_running'] = CommandStructure(cmd_ind=43, var_type=np.uint8)
    _cmd_map['get_m3_steps'] = CommandStructure(cmd_ind=44, var_type=np.uint32)
    _cmd_map['set_m3_steps'] = CommandStructure(cmd_ind=45, var_type=np.uint32)
    _cmd_map['get_m3_target_steps'] = CommandStructure(cmd_ind=46, var_type=np.uint32)
    _cmd_map['set_m3_target_steps'] = CommandStructure(cmd_ind=47, var_type=np.uint32)
    _cmd_map['get_m3_step_interval'] = CommandStructure(cmd_ind=48, var_type=np.uint32)
    _cmd_map['set_m3_step_interval'] = CommandStructure(cmd_ind=49, var_type=np.uint32)
    _cmd_map['get_m3_finite_mode'] = CommandStructure(cmd_ind=50, var_type=np.uint8)
    _cmd_map['set_m3_finite_mode'] = CommandStructure(cmd_ind=51, var_type=np.uint8)
    _cmd_map['get_m3_dir'] = CommandStructure(cmd_ind=52, var_type=np.uint8)
    _cmd_map['set_m3_dir'] = CommandStructure(cmd_ind=53, var_type=np.uint8)
    _cmd_map['get_m3_enabled'] = CommandStructure(cmd_ind=54, var_type=np.uint8)
    _cmd_map['set_m3_enabled'] = CommandStructure(cmd_ind=55, var_type=np.uint8)
    _cmd_map['get_m0_var_ustep_support'] = CommandStructure(cmd_ind=56, var_type=np.uint8)
    _cmd_map['get_m1_var_ustep_support'] = CommandStructure(cmd_ind=57, var_type=np.uint8)
    _cmd_map['get_m2_var_ustep_support'] = CommandStructure(cmd_ind=58, var_type=np.uint8)
    _cmd_map['get_m3_var_ustep_support'] = CommandStructure(cmd_ind=59, var_type=np.uint8)
    _cmd_map['get_m0_usteps_exp'] = CommandStructure(cmd_ind=60, var_type=np.uint8)
    _cmd_map['set_m0_usteps_exp'] = CommandStructure(cmd_ind=61, var_type=np.uint8)
    _cmd_map['get_m1_usteps_exp'] = CommandStructure(cmd_ind=62, var_type=np.uint8)
    _cmd_map['set_m1_usteps_exp'] = CommandStructure(cmd_ind=63, var_type=np.uint8)
    _cmd_map['get_m2_usteps_exp'] = CommandStructure(cmd_ind=64, var_type=np.uint8)
    _cmd_map['set_m2_usteps_exp'] = CommandStructure(cmd_ind=65, var_type=np.uint8)
    _cmd_map['get_m3_usteps_exp'] = CommandStructure(cmd_ind=66, var_type=np.uint8)
    _cmd_map['set_m3_usteps_exp'] = CommandStructure(cmd_ind=67, var_type=np.uint8)
    _cmd_map['get_sub_us_divider'] = CommandStructure(cmd_ind=68, var_type=np.uint32)

    def __init__(self, serial_port=None, serial_baudrate=None, pump_count:int = None):
        if not (serial_port is None):
                self._serial_port = serial_port
        if not (serial_baudrate is None):
                self._serial_baudrate = serial_baudrate
        if not (pump_count is None):
            self.pump_count = pump_count
        self._lock_config = Lock()
        self._lock_send = Lock()
        self._rx_buffer = bytearray(self._MSG_LEN)
        self._tx_buffer = bytearray(self._MSG_LEN)
        self._event_signal_booted_rcv = Event()
        self._event_ack_rcv = Event()
        self._event_msg_rcv = Event()
        self._event_msg_processed = Event()
        self._rcv_msg_table = { #first byte (uint8) of rx_buffer
            255: self._msg_checksum_err,
            254: self._msg_cmd_err,
            253: self._msg_ack,
            252: self._msg_signal_booted,
            #from 200 to 252 are for motor signals such as completion of a task
            #these must be appended when the pump classes are initialized
        }

    def connect(self,serial_port=None,serial_baudrate=None,conn_delay_s:float=3):
        """
        Initiate serial connection to the pump.
        Creates a new connection regardless of the current status.
        """
        try:
            if not (serial_port is None):
                self._serial_port = serial_port
            if not (serial_baudrate is None):
                self._serial_baudrate = serial_baudrate
            self._serial_com = serial.Serial(port=self._serial_port,baudrate=self._serial_baudrate,inter_byte_timeout=self._serial_inter_byte_timeout_s)
            sleep(conn_delay_s) #might be necessary for Arduino to boot up, may not be required for others
            self._serial_com.read_all() #clear the buffer in case it contains junk
            self._thread_msg_rcv = Thread(target=self._read_data_thread_func)
            self._thread_msg_rcv.daemon = True
            self._thread_msg_rcv.start()
            # one can also check the start signal here but it needs to be enabled in the firmware
            # self.event_start_signal_received.wait()
            # self.event_start_signal_received.clear()

            self.status = "Connected"
            logging.info('Connected to the microcontroller.')
            self._get_sub_us_divider()

            self.pumps = []
            for i in range(self.pump_count):
                self.pumps.append(
                    Pump(
                    motor_ind = i,
                    sub_us_divider=self._sub_us_divider,
                    func_pump_send_cmd = self._send_cmd_from_table, 
                    )
                    )
                self._rcv_msg_table[200+i] = self.pumps[i]._signal_m_stopped #assign the finished signal function to the corresponding motor index

            #MUST BE CALLED AFTER PUMP OBJECTS ARE CREATED
            self._apply_pump_config_pre(self.config)
            for i in range(self.pump_count):
                self.pumps[i]._read_initial_variables()
            self._apply_pump_config_post(self.config)
            logging.info(f"{self.pump_count} pumps have been initalized.")
            return True
        except serial.SerialException as e:
            self.status = "Disconnected"
            logging.critical(f"Could not connect to the microcontroller of the pump. No start signal received. {e}")
            raise Exception(f"Could not connect to the microcontroller of the pump. No start signal received.{e}")
            return False
        except Exception as e:
            self.status = "Disconnected"
            logging.critical(f"Could not connect to the microcontroller of the pump. {e}")
            raise Exception(f"Could not connect to the microcontroller of the pump. {e}")
            return False
        
    def _calc_rx_checksum8(self):
        data = np.frombuffer(self._rx_buffer,np.uint8)
        checksum = data[0]
        for i in range(1,self._MSG_LEN - 1): #-1 for excluding the last byte
            checksum = checksum ^ data[i]
        return checksum #return single byte of checksum
    
    def _calc_tx_checksum8(self):
        data = np.frombuffer(self._tx_buffer,np.uint8)
        checksum = data[0]
        for i in range(1,self._MSG_LEN - 1): #-1 for excluding the last byte
            checksum = checksum ^ data[i]
        return checksum #return single byte of checksum
    
    def _check_rx_checksum8(self):
        checksum_rx = np.uint8(self._rx_buffer[-1]) #last byte is the checksum
        checksum_calc = self._calc_rx_checksum8()
        if checksum_calc == checksum_rx:
            self._rx_error_cnt = 0
            return True
        self._rx_error_cnt += 1
        self._rx_total_error_cnt += 1
        logging.critical(f"MCU sent a message with wrong cheksum. Consecutive error count: {self._rx_error_cnt} | Total error count: {self._rx_total_error_cnt}")
        # raise Exception("MCU sent a message with wrong cheksum.")
        return False
        
    def _write_data(self):
        #writes tx_buffer after setting the last byte to calculated checksum8
        self._tx_buffer[-1] = self._calc_tx_checksum8()
        self._serial_com.write(self._tx_buffer)
    
    def _read_data(self):
        try:
            self._rx_buffer = self._serial_com.read(self._MSG_LEN) #operates with inter_byte_timeout
            if len(self._rx_buffer) < self._MSG_LEN: #probably junk during UART initalization
                sleep(0.01)
                return False #ignore the junk
        except:
            sleep(0.01)
            return False
        if self._check_rx_checksum8():
            return True
        return False
        
    def _read_data_thread_func(self):
        while (True):
            if self._read_data():
                msg_ind = self._rx_buffer[0]
                if msg_ind in self._rcv_msg_table: #if the message is an ack, err, end of motor task signal, or start signal
                    func = self._rcv_msg_table.get(msg_ind)
                    result = func()
                elif msg_ind >= 200: #if not one of the reserved 200-252 signals, then unknown message
                    self._msg_unknown()
                else: #if the message is a response of a get command
                    self._event_msg_rcv.set()
                    self._event_msg_processed.wait()
                    self._event_msg_processed.clear()
            sleep(0.001) #read data should block, but just in case for frequent errors

    def _send_cmd_from_table(self,fnc_name:str,val = None):
        #lookup the command from function name
        cmd = self._cmd_map.get(fnc_name)
        if cmd is None:
            return False
        if fnc_name.startswith("get_"):
            result = self._send_get_cmd(cmd_index=cmd.cmd_ind,var_type=cmd.var_type)
        elif fnc_name.startswith("set_"):
            if val is None:
                return False
            result = self._send_set_cmd(cmd_index=cmd.cmd_ind,var_type=cmd.var_type,val=val)
        else:
            return False
        return result

    def _send_set_cmd(self,cmd_index: np.uint8,var_type: type, val):
        #send the set message
        #byte 0 is the command index, byte 1 to 4 are the value bytes, byte 5 is the checksum (dealt by write func)
        self._lock_send.acquire()
        self._tx_buffer[0] = np.uint8(cmd_index).tobytes()[0]
        arg_bytes = var_type(val).tobytes()
        len_bytes = len(arg_bytes)
        if len_bytes <= self._ARG_LEN:
            self._tx_buffer[1:len_bytes+1] = arg_bytes
            self._write_data() #send the tx_buffer with the checksum 
            #wait for the acknowledgement message
            self._event_ack_rcv.wait()
            self._event_ack_rcv.clear()
        self._lock_send.release()
        return True
    
    def _send_get_cmd(self,cmd_index:np.uint8,var_type:type):
        #send the get message
        self._lock_send.acquire()
        self._tx_buffer[0] = np.uint8(cmd_index).tobytes()[0]
        self._write_data()
        #wait for the response
        self._event_msg_rcv.wait()
        #process the response
        response = np.frombuffer(buffer=self._rx_buffer[1:-1],dtype=var_type)
        response = response[0]
        #reset the event flags
        self._event_msg_rcv.clear()
        self._event_msg_processed.set()
        self._lock_send.release()
        return response
    
    def _get_sub_us_divider(self):
        result = self._send_cmd_from_table(inspect.stack()[0][3].lstrip("_"))
        self._sub_us_divider = np.float64(result)
        #min2us = 60000000.0 = 6e7
        return result
    
    def _msg_checksum_err(self):
        logging.critical("MCU received a message with a wrong checksum.")
        # raise Exception("MCU received a message with a wrong checksum.")
        print(f"Waiting {self._serial_inter_byte_timeout_s * 2} seconds for buffer reset.")
        sleep(self._serial_inter_byte_timeout_s * 2)
        return False

    def _msg_cmd_err(self):
        logging.critical("MCU received a message with wrong or unsupported command.")
        # raise Exception("MCU received a message with wrong or unsupported command.")
        # print("Waiting 1.5seconds for buffer reset.")
        # sleep(1.5)
        return False
    
    def _msg_ack(self):
        self._event_ack_rcv.set()
        return True

    def _msg_signal_booted(self):
        self._event_signal_booted_rcv.set()
        return True

    def _msg_unknown(self):
        logging.critical("MCU sent an unknown message.")
        # raise Exception("Received an unknown message.")
        return False
    
    def save_config(self, fpath:str=None):
        self._lock_config.acquire()
        config = {
            "device": {
                "serial_port": self._serial_port,
                "serial_baudrate": self._serial_baudrate,
            },
            "pump_count": self.pump_count,
        }

        config["pumps"] = {}
        for i in range(self.pump_count):
            config["pumps"]["pump"+str(i)] = {
                "calibration_uL_per_Rev": self.pumps[i].uL_per_rev,
                "gear_ratio": self.pumps[i]._gear_ratio,
                "motor_base_spr": self.pumps[i]._motor_base_spr,
                "motor_dir_inverse": self.pumps[i]._motor_dir_inverse,
                "motor_usteps": self.pumps[i]._motor_usteps,
                "direction_default": self.pumps[i].direction_default,
                "direction_inverse": self.pumps[i]._motor_dir_inverse,
                "max_rpm": self.pumps[i]._max_rpm,
                "motor_var_ustep_support": self.pumps[i]._motor_var_ustep_support,
                "motor_max_ustep_exp": self.pumps[i]._motor_max_ustep_exp,
                "motor_min_ustep_exp": self.pumps[i]._motor_min_ustep_exp,
            }
        if fpath is None:
            if self._last_config_fpath is None:
                self._last_config_fpath = os.path.dirname(__file__) + "/HiPeristaltic.toml"
        else:
            self._last_config_fpath = fpath
        try:
            with open(fpath, 'w') as f:
                toml.dump(config, f)
            fpath = self._last_config_fpath
        except Exception as e:
            logging.critical(f"Error writing settings: {e}")
            print(f"Error writing settings: {e}")
        self._lock_config.release()

    def load_config(self, fpath:str=None):
        if fpath is None:
            fpath = os.path.dirname(__file__) + "/HiPeristaltic.toml"
        self._last_config_fpath = fpath
        self._lock_config.acquire()
        if not os.path.exists(fpath):
            logging.critical(f"Config file not found at {fpath}.")
            print(f"Config file not found at {fpath}.")
            self._lock_config.release()
            raise Exception(f"Config file not found at {fpath}.")
        with open(fpath, 'r') as f:
            config = toml.load(f)
            self._serial_port = config["device"]["serial_port"]
            self._serial_baudrate = config["device"]["serial_baudrate"]
            self.pump_count = config["pump_count"]
            self.config = config
        self._lock_config.release()
        return config
    
    def _apply_pump_config_pre(self, config: dict):
        for i in range(self.pump_count):
            self.pumps[i].uL_per_rev = config["pumps"]["pump"+str(i)]["calibration_uL_per_Rev"]
            self.pumps[i]._gear_ratio = float(config["pumps"]["pump"+str(i)]["gear_ratio"])
            self.pumps[i]._motor_base_spr = config["pumps"]["pump"+str(i)]["motor_base_spr"]
            self.pumps[i]._motor_max_ustep_exp = config["pumps"]["pump"+str(i)]["motor_max_ustep_exp"]
            self.pumps[i]._motor_min_ustep_exp = config["pumps"]["pump"+str(i)]["motor_min_ustep_exp"]
            self.pumps[i].direction_default = config["pumps"]["pump"+str(i)]["direction_default"]
            self.pumps[i]._max_rpm = config["pumps"]["pump"+str(i)]["max_rpm"]
            self.pumps[i]._motor_dir_inverse = config["pumps"]["pump"+str(i)]["direction_inverse"]
        return True
    
    def _apply_pump_config_post(self, config: dict):
        for i in range(self.pump_count):
            self.pumps[i]._motor_var_ustep_support = (self.pumps[i]._motor_var_ustep_support and config["pumps"]["pump"+str(i)]["motor_var_ustep_support"])
            if not self.pumps[i]._motor_var_ustep_support:
                self.pumps[i]._motor_usteps = config["pumps"]["pump"+str(i)]["motor_usteps"]
        return True
    
    def emergency_stop(self):
        result = True
        for i in range(self.pump_count):
            try:
                result = (result and self.pumps[i]._set_m_enabled(False))
            except:
                logging.critical(f"Could not emergency stop the pump {i}.")
                print(f"Could not emergency stop the pump {i}.")
            if result == False:
                logging.critical(f"Could not emergency stop the pump {i}.")
                print(f"Could not emergency stop the pump {i}.")
            else:
                logging.info(f"Pump {i} has been disabled due to emergency stop command.")
                print(f"Pump {i} has been disabled due to emergency stop command.")
        return result
    
    def stop_all_pumps(self):
        result = True
        for i in range(self.pump_count):
            try:
                self.pumps[i].pump_stop()
            except:
                result = False
                logging.critical(f"Could not stop the pump {i}.")
                print(f"Could not stop the pump {i}.")
        return result

#test code
if __name__ == "__main__":
    test = HiPeristalticInterface()
    test.load_config()
    test.connect()
    print("Max RPM of Pump 1:",test.pumps[0].get_max_rpm())
    print("Max volume(L) of Pump 1:",(test.pumps[0].get_max_volume_uL() / 1e6))
    print("Min volume(uL) of Pump 1:",test.pumps[0].get_min_volume_uL())
    print("Max flow rate(uL/s) of Pump 1:",test.pumps[0].get_max_flow_rate_uLpersec())
    print("Min flow rate(uL/s) of Pump 1:",test.pumps[0].get_min_flow_rate_uLpersec())
    test.pumps[3].pump_volume(target_volume_uL=60,flow_rate_uLpersec=12,direction="cw",blocking=True)
    test.pumps[1].pump_continuous(flow_rate_uLpersec=10,direction="cw")
    test.pumps[2].pump_continuous(flow_rate_uLpersec=10,direction="ccw")
    test.pumps[0].pump_volume(target_volume_uL=60,flow_rate_uLpersec=12,direction="ccw",blocking=False)
    print("Flow rate (uL/s): ", test.pumps[0].get_flow_rate_uLpersec())
    print("Step interval (ticks): ", test.pumps[0]._motor_step_interval)
    print("Step interval (us): ", test.pumps[0]._motor_step_interval * test.pumps[0]._sub_us_divider)
    while test.pumps[0].get_running():
        print("Remaining time (s):" , test.pumps[0].get_remaining_time().total_seconds())
        print("Remaining volume (uL):" , test.pumps[0].get_remaining_volume_uL())
        sleep(1)
    print("----------------")
    test.pumps[2].pump_stop()
    test.pumps[2].pump_volume(target_volume_uL=60,flow_rate_uLpersec=12,direction="cw",blocking=False)
    sleep(1)
    test.pumps[2].pump_stop()
    sleep(1)
    test.pumps[2].pump_resume()
    while test.pumps[2].get_running():
        print("Remaining time (s):" , test.pumps[2].get_remaining_time().total_seconds())
        print("Remaining volume (uL):" , test.pumps[2].get_remaining_volume_uL())
        sleep(0.5)
    # pump.save_config()