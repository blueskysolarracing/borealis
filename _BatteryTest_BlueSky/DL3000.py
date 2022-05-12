#!/usr/bin/env python3
import time

__all__ = ["DL3000"]

class DL3000(object):
    """
    Rigol DL3000 command wrapper.
    """
    def __init__(self, inst):
        """
        Initialize the DL3000 wrapper with a specific PyVISA resource.
        This class does NOT open the resource, you have to open it for yourself!
        """
        self.keyAPPState = 0
        self.inst = inst

    def voltage(self):
        # My DL3021 returns a string like '0.000067\n0'
        return float(self.inst.query(":MEAS:VOLT?").partition("\n")[0])

    def current(self):
        # My DL3021 returns a string like '0.000067\n0'
        return float(self.inst.query(":MEAS:CURR?").partition("\n")[0])

    def power(self):
        # My DL3021 returns a string like '0.000067\n0'
        return float(self.inst.query(":MEAS:POW?").partition("\n")[0])

    def resistance(self):
        # My DL3021 returns a string like '0.000067\n0'
        return float(self.inst.query(":MEAS:RES?").partition("\n")[0])

    def set_cc_slew_rate(self, slew):
        # My DL3021 returns a string like '0.000067\n0'
        self.inst.write(f":SOURCE:CURRENT:SLEW {slew}")

    def is_enabled(self):
        """
        Enable the electronic load
        Equivalent to pressing "ON/OFF" when the load is ON
        """
        return self.inst.query(":SOURCE:INPUT:STAT?").strip() == "1"

    def enable(self):
        """
        Enable the electronic load
        Equivalent to pressing "ON/OFF" when the load is ON
        """
        self.inst.write(":SOURCE:INPUT:STAT ON")

    def disable(self):
        """
        Disable the electronic load
        Equivalent to pressing "ON/OFF" when the load is ON
        """
        self.inst.write(":SOURCE:INPUT:STAT OFF")

    def set_mode(self, mode="CC"):
        """
        Set the load mode to "CURRENT", "VOLTAGE", "RESISTANCE", "POWER"
        """
        self.inst.write(":SOURCE:FUNCTION {}".format(mode))

    def query_mode(self):
        """
        Get the mode:
        "CC", "CV", "CR", "CP"
        """
        return self.inst.query(":SOURCE:FUNCTION?").strip()

    def set_cc_current(self, current):
        """
        Set CC current limit
        """
        return self.inst.write(":SOURCE:CURRENT:LEV:IMM {}".format(current))

    def set_cp_power(self, power):
        """
        Set CP power limit
        """
        return self.inst.write(":SOURCE:POWER:LEV:IMM {}".format(power))

    def set_cp_ilim(self, ilim):
        """
        Set CP current limit
        """
        return self.inst.query(":SOURCE:POWER:ILIM {}".format(ilim))

    def cc(self, current, activate=True):
        """
        One-line constant-current configuration.
        if activate == True, also turns on the power supply
        """
        self.set_mode("CC")
        self.set_cc_current(current)
        self.enable()

    def cp(self, power, activate=True):
        """
        One-line constant-current configuration.
        if activate == True, also turns on the power supply
        """
        self.set_mode("CP")
        self.set_cp_power(power)
        self.enable()

    def reset(self):
        return self.inst.write("*RST")

    def set_voltage(self, voltage):
        """
        Sets load voltage
        """
        return self.inst.write(":SOURCE:VOLTAGE:LEV:IMM {}".format(voltage))
    
    def simAPP_Key(self):

        self.keyAPPState = (self.keyAPPState + 1) % 3

        return self.inst.write(":SYST:KEY 13")

    def simINT_KeyPress(self, value):
        return self.inst.write(":SYST:KEY {}".format(20 + value))
    
    def setBATT_Curr(self, inCurrent):

        if inCurrent >= 40:
            return False

        # toggle the CURRENT menu button
        self.inst.write(":SYST:KEY 14")
        look4DecimalPtn = False
        maxEntries = 5

        # enter the value accordingly
        if type(inCurrent) != 'int':
            look4DecimalPtn = True
            if inCurrent > 9.999:
                maxEntries = 6
        
        inCurrentString = str(inCurrent)
        keyPresses = min(maxEntries, len(inCurrentString))
        currInteger = 0

        for i in range(0, keyPresses):

            # check for decimal point
            if look4DecimalPtn == True:
                if inCurrentString[i] == '.':
                    look4DecimalPtn = False
                    self.inst.write(":SYST:KEY 30")
                    time.sleep(1)
                    continue
            
            currInteger = int(inCurrentString[i])

            self.simINT_KeyPress(currInteger)
            time.sleep(1)

        # press OK to confirm input
        self.inst.write(":SYST:KEY 41")

        return True

    def setBATT_VStop(self, inVoltage, cutOFF):

        # check if the input voltage is a valid value
        if inVoltage < cutOFF:
            return False

        # toggle the VOLTAGE menu button (needs to be pressed twice to enable)
        self.inst.write(":SYST:KEY 16")
        time.sleep(2)
        self.inst.write(":SYST:KEY 16")

        look4DecimalPtn = False
        maxEntries = 5

        # enter the value accordingly
        if type(cutOFF) != 'int':
            look4DecimalPtn = True
            if cutOFF > 9.999:
                maxEntries = 6
        
        inVoltageString = str(cutOFF)
        keyPresses = min(maxEntries, len(inVoltageString))
        currInteger = 0

        for i in range(0, keyPresses):

            # check for decimal point
            if look4DecimalPtn == True:
                if inVoltageString[i] == '.':
                    look4DecimalPtn = False
                    self.inst.write(":SYST:KEY 30")
                    time.sleep(1)
                    continue
            
            currInteger = int(inVoltageString[i])

            self.simINT_KeyPress(currInteger)
            time.sleep(1)

        # press OK to confirm input
        self.inst.write(":SYST:KEY 41")

        return True