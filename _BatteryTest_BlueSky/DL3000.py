#!/usr/bin/env python3

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
