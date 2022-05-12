import time
import datetime

class testHCCP:

    def __init__(self, inBatteryObj, eLoad, pSupply, cycles = 0):

        # initialize member variables
        self.m_battery = inBatteryObj
        self.m_eLoad = eLoad
        self.m_pSupply = pSupply
        self.m_numCycles = cycles
        self.m_fileName = ''
        self.m_numCycles = cycles
        self.m_lowVolt = 0
        self.m_highVolt = 0
        self.m_totSimTime = 0

    def initELoad(self) -> bool:

        """
        Set setting for the eLoad.
        """
        
        self.m_lowVolt = self.m_battery.m_voltageBounds[0]
        inCurrent = self.m_battery.genInCurrent()[1]
        result = True

        if inCurrent == 0:
            print("eLoad :: Input Current cannot be 0 ... \n")
            return False

        # setup device to run constant battery discharge
        self.m_eLoad.reset()
        self.m_eLoad.simAPP_Key()

        result = self.m_eLoad.setBATT_Curr(inCurrent)

        if not result:
            print("eLoad :: Discharge current is outside of the allowable range (40 A)! \n")
            return False

        currVoltage = self.m_eLoad.voltage()
        result =  self.m_eLoad.setBATT_VStop(currVoltage, self.m_lowVolt)

        if not result:
            print("eLoad :: Target voltage is lower than the cutoff voltage ({} V)! \n".format(self.m_lowVolt))
            return False

        if (abs(currVoltage - self.m_lowVolt) <= 0.05):
            print("eLoad :: Cannot discharge battery, near end of capacity ... \n")
            return False

        return True
    
    def initDCSupply(self) -> bool:

        """
        Set settings of the pSupply.
        """

        self.m_highVolt = self.m_battery.m_voltageBounds[1]
        inCurrent = self.m_battery.genInCurrent()[0]

        if inCurrent == 0:
            print("pSupply :: Input Current cannot be 0 ... \n")
            return False

        result = self.m_pSupply.reset()

        if not result:
            print("pSupply :: Could not reset DC Power Supply ... \n")
            return False

        # select Channel 1 on the power supply since it has the 
        # range of interest
        result = self.m_pSupply.selectChannel(1)

        if self.m_pSupply.queryChannel() != 1:
            print("pSupply :: Could not select CH1 for battery testing ... \n")
            return False

        self.m_pSupply.channelON()
        currVoltage = self.m_pSupply.measureValue("Voltage") 
        self.m_pSupply.channelOFF()

        # set voltage a bit higher to allow enough current to be drawn
        result = self.m_pSupply.setVOLT(self.m_highVolt + 0.5)
    
        if not result:
            print("pSupply :: Voltage is out of bounds, please review your battery characteristics ... \n")
            return False

        # set OVP
        self.m_pSupply.setOVP(self.m_highVolt)

        result = self.chargeBatteryInit(currVoltage)  

        # set input current
        result = self.m_pSupply.setCURR(inCurrent)

        if not result:
            print("pSupply :: Input current is too large - defaulted to maximum ... \n")
            result = True

        return result

    def chargeBatteryInit(self, currVoltage) -> bool:

        """
        Check if the battery needs to be charged before starting the test.
        """

        if currVoltage > self.m_highVolt:
            print("pSupply :: Battery voltage over maximum operating voltage ... \n")
            return False

        if (abs(currVoltage - self.m_highVolt) <= 0.05):
            print("Cannot charge battery, near full capacity ... \n")
            return True
        
        self.m_pSupply.setCURR(5)
        self.m_pSupply.channelON()

        while (True):

            currVoltage = self.m_pSupply.measureValue("Voltage")

            time.sleep(0.5)

            if (abs(currVoltage - self.m_highVolt) <= 0.05):
                break

        self.m_pSupply.channelOFF()

        return True

    def discharge(self, modeTimeBased, timeIn) -> int:

        tempTime = 0
        result = 0

        self.m_eLoad.enable()

        while (True):
            
            prevTime = time.time()

            time.sleep(0.001)

            currVoltage = self.m_eLoad.voltage()
            currCurrent = self.m_eLoad.current()

            timeDelta = time.time() - prevTime
            tempTime += timeDelta

            self.logValues(currVoltage, currCurrent, timeDelta, tempTime + self.m_totSimTime)

            if (abs(currVoltage - self.m_lowVolt) <= 0.05):
                print("eLoad :: Battery is fully discharged ... \n")
                result = 1
                break

            if (tempTime > timeIn) and modeTimeBased:
                break

        self.m_eLoad.disable()
        self.m_totSimTime += tempTime

        return result

    def charge(self, modeTimeBased, timeIn) -> int:

        tempTime = 0
        result = 0

        self.m_pSupply.channelON()

        while (True):
            
            prevTime = time.time()

            time.sleep(0.001)

            currVoltage = self.m_pSupply.measureValue("Voltage")
            currCurrent = self.m_pSupply.measureValue("Current")

            timeDelta = time.time() - prevTime
            tempTime += timeDelta

            self.logValues(currVoltage, currCurrent, timeDelta, tempTime + self.m_totSimTime)

            if (abs(currVoltage - self.m_highVolt) < 0.05):
                print("Battery is fully charged ... \n")
                result = 1
                break

            if (tempTime > timeIn) and modeTimeBased:
                break

        self.m_pSupply.channelOFF()
        self.m_totSimTime += tempTime

        return result

    def rest(self, inTimeRest) -> bool:

        tempTime = 0

        while (tempTime < inTimeRest):
            
            prevTime = time.time()

            time.sleep(0.001)

            currVoltage = self.m_eLoad.voltage()
            currCurrent = self.m_eLoad.current()

            timeDelta = time.time() - prevTime
            tempTime += timeDelta

            self.logValues(currVoltage, currCurrent, timeDelta, tempTime + self.m_totSimTime)

        self.m_totSimTime += tempTime

        return True

    def swapCurrents(self, step):

        currents = self.m_battery.genInCurrent()

        if step == "DISCHARGE":

            self.m_eLoad.setBATT_Curr(currents[1])
            self.m_pSupply.setCURR(currents[0])

        elif step == "CHARGE":

            self.m_eLoad.setBATT_Curr(currents[0])
            self.m_pSupply.setCURR(currents[1])

        return 

    def runHCCP(self) -> bool: 

        result = 0

        if self.m_numCycles == 0:
        
            # start with a complete discharging of the battery
            result = self.discharge(False, 0)
            # rest for 1hr
            self.rest(3600)
            # charge the battery to 100% SOC
            result = self.charge(False, 0)
            # rest for 1hr
            self.rest(3600)

        # 10% SoC time - approximate in seconds
        timeStepSoC = ((0.001*(self.m_battery.m_totalCapacity))/self.m_battery.genInCurrent()[1])*60

        while (True):

            # discharge pulse for 10s
            result = self.discharge(True, 10)

            if result == 1:
                break

            # rest for 40s
            self.rest(40)
            # charge pulse for 10s
            self.charge(True, 10)

            # discharge 10% SoC
            result = self.discharge(True, timeStepSoC)
            
            if result == 1:
                break
            
            #rest for 1h
            self.rest(180)

            self.m_numCycles += 1
            print("Cycle {} completed. \n".format(self.m_numCycles))

        return True

    def initLogging(self):

        """Create logging file."""

        now = datetime.datetime.now()
        nowStr = now.strftime("%Y%m%d_%H%M%S")
        self.m_fileName = "Results_HCCP_" + nowStr + ".csv"

        with open(self.m_fileName, 'w', encoding = "UTF8") as file:

            file.write("TimeStamp, Time Delta, Input Current, Output Voltage \n")

        file.close()

        return
    
    def logValues(self, inVolt, inCurr, timeDelta, timeStamp):
        
        """Log measurements."""

        inStr = str(timeStamp) + ',' + str(timeDelta) + ',' + str(inCurr) + ',' + str(inVolt) + '\n'

        with open(self.m_fileName, 'a', encoding = "UTF8") as file:

            file.write(inStr)

        file.close()

        return 