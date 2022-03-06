#!/usr/bin/env python3
import time

__all__ = ["DP800"]

class DP800(object):

    """
    Rigol DP800 command wrapper.
    """

    def __init__(self, inst):
            
        """
        Initialize the DP800 wrapper with a specific PyVISA resource.
        This class does NOT open the resource, you have to open it when passed as 
        an instance.
        """

        self.m_instance = inst

        self.m_measureLUT = {"Current": "CURR",
                             "Voltage": "VOLT",
                             "Power": "POWE"}
    
    def queryChannel(self) -> int:

        result = int(self.m_instance.query(":INST?").partition("\n"))

        return result
    
    def selectChannel(self, ChannelID) -> bool:

        """
        ChannelID: 1, 2 and 3 as integers.
        """

        if type(ChannelID) != int:
            print("Enter the right type for Channel ID ... \n")
            return False
        else:
            if ChannelID < 1 or ChannelID > 3:
                print("ChannelID is not valid ... \n")
                return False

        # query channel to check if it is already selected
        currChannel = self.queryChannel()

        result = True

        if currChannel != ChannelID:

            command = ":INST:NSEL " + ChannelID
            result = (self.m_instance.write(command).strip() == "1")
        
        return result

    def measureValue(self, ChannelID, Value) -> float:
        
        """
        ChannelID: 1, 2 and 3 as integers.
        Value: Current, Voltage, Power as strings.
        """

        if type(ChannelID) != int:
            print("Enter the right type for Channel ID ... \n")
            return -1000.0
        else:
            if ChannelID < 1 or ChannelID > 3:
                print("ChannelID is not valid ... \n")
                return -1000.0

        if Value not in self.m_measureLUT:
            print("Invalid value query ... \n")
            return -1000.0
        
        key = self.m_measureLUT[Value]
        command = "MEAS:" + key + "? CH" + str(ChannelID)

        result = float(self.m_instance.query(command).partition("\n"))

        return result

    def reset(self) -> bool:

        """
        Reset the settings to factory settings and clear the error
        queue.
        """
        return self.m_instance.write("*RST")

    def simKeyPress(self, KeyID):

        """
        Simulate key presses based on ID. 
        """

        commandON = ":SYST:KLOC " + KeyID + ",ON"
        commandOFF = ":SYST:KLOC " + KeyID + ",OFF"
        commandQuery = ":SYST:KLOC? " + KeyID

        self.m_instance.write(commandON)

        time.sleep(1)

        status = int(self.m_instance.query(commandQuery).partition("\n"))

        if status != 1:
            print("Couldn't lock the key {} ...".format(KeyID))
            return False

        self.m_instance.write(commandOFF)

        time.sleep(1)

        status = int(self.m_instance.query(commandQuery).partition("\n"))
        
        if status != 0:
            print("Couldn't unlock the key {} ...".format(KeyID))
            return False

        return True

        
