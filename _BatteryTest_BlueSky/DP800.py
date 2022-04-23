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
        
        self.m_limitsCHCurr = {"CH1": 5,
                               "CH2": 2,
                               "CH3": 2}
        
        self.m_limitsCHCurr = {"CH1": 8,
                               "CH2": 30,
                               "CH3": -30}
    
    def queryChannel(self) -> int:

        queryResult = self.m_instance.query(":INST?").partition("\n")
        
        result = int(queryResult[0][2])

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

            command = ":INST:NSEL " + str(ChannelID)
            result = self.m_instance.write(command)
            time.sleep(0.5)
        
        return result
    
    def channelON(self):
        
        """
        Enable the selected Channel.
        """
        
        ID = self.queryChannel()
        
        command = ":OUTP CH" + str(ID) + ",ON"
        self.m_instance.write(command)
        
        time.sleep(0.5)
        
        return
    
    def channelOFF(self):
            
        """
        Disable the selected Channel.
        """
        
        ID = self.queryChannel()
        
        command = ":OUTP CH" + str(ID) + ",OFF"
        self.m_instance.write(command)
        
        time.sleep(0.5)
        
        return
        

    def measureValue(self, Value) -> float:
        
        """
        ChannelID: 1, 2 and 3 as integers.
        Value: Current, Voltage, Power as strings.
        """

        ID = self.queryChannel()
        
        key = self.m_measureLUT[Value]
        command = "MEAS:" + key + "? CH" + str(ID)

        resultSTR = self.m_instance.query(command).partition("\n")
        result = float(resultSTR[0])

        return result

    def reset(self) -> bool:

        """
        Reset the settings to factory settings and clear the error
        queue.
        """
    
        value = self.m_instance.write("*RST")
        
        return True 
    
    def setCURR(self, inCurr) -> bool:
        
        """
        Set current based on the selected channel.
        """
        
        ID = self.queryChannel()
        LUTId = "CH" + str(ID)
        
        if inCurr > self.m_limitsCHCurr[LUTId]:
            
            command = ":CURR " + str(5)
            self.m_instance.write(command)
            
            return False
        
        command = ":CURR " + str(inCurr)
        self.m_instance.write(command)
        
        time.sleep(0.5)
        
        return True
    
    def setVOLT(self, inVolt) -> bool:
        
        """
        Set voltage on the selected channel.
        """
        
        ID = self.queryChannel()
        LUTId = "CH" + str(ID)
        
        if ID <= 2:
            if inVolt > self.m_limitsCHCurr[LUTId]:
                return False
        else:
            if inVolt < self.m_limitsCHCurr[LUTId]:
                return False
                
        command = ":VOLT " + str(inVolt)
        self.m_instance.write(command)

        time.sleep(0.5)
        
        return True
    
    def setOVP(self, voltOVP):
        
        """
        Set over-voltage protetction.
        """
        
        commandVal = "VOLT:PROT " + str(voltOVP)
        commandON = "VOLT:PROT:STAT ON"
        
        self.m_instance.write(commandVal)
        
        time.sleep(0.5)
        
        self.m_instance.write(commandON)
        
        time.sleep(0.5)
        
        return
    
    def setOCP(self, currOCP):
            
        """
        Set over-current protetction.
        """
        
        commandVal = "CURR:PROT " + str(currOCP)
        commandON = "CURR:PROT:STAT ON"
        
        self.m_instance.write(commandVal)
        
        time.sleep(0.5)
        
        self.m_instance.write(commandON)
        
        time.sleep(0.5)
        
        return
    
    def disableOP(self):
        
        """
        Disable over protection options.
        """
        
        commandOFF = "CURR:PROT:STAT OFF"
        
        self.m_instance.write(commandOFF)
        
        time.sleep(0.5)
        
        commandOFF = "VOLT:PROT:STAT OFF"
        
        self.m_instance.write(commandOFF)
        return