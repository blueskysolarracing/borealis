import csv

class BatteryObj:

    def __init__(self, cellCapacity, numCells, testSetting, voltageBounds, CRate = 0, HCCPRates = []):

        # initialize member variables
        self.m_totalCapacity = numCells*cellCapacity
        self.m_setting = testSetting
        self.m_voltageBounds = voltageBounds
        self.m_Crate = CRate
        self.m_HCCPRates = HCCPRates

        self.m_measuredVoltage = []
        self.m_measuredCurrent = []
        self.m_timeRec = []


    def genInCurrent(self, HCCPRates):

        if abs(self.m_setting) == 1:
            
            return self.m_totalCapacity*self.m_CRate/1000

        elif self.m_setting == 0:

            return [10]

        return 0

    def dumpData(self, filename):

        dataCnt = len(self.m_measuredVoltage)
        row = ""

        with open(filename, 'w', encoding = "UTF8") as file:

            file.write("Time Delta, Input Current, Output Voltage \n")

            for i in range(0, dataCnt, 1):

                row = str(self.m_timeRec[i]) + ", " + str(self.m_measuredCurrent[i]) + ", " + str(self.m_measuredVoltage[i]) + "\n"
                file.write(row)

        file.close()

        return

    def logMeasurement(self, measurement_V, measurement_C, timestamp):
        
        self.m_timeRec.append(timestamp)
        self.m_measuredVoltage.append(measurement_V)
        self.m_measuredCurrent.append(measurement_C)

        return
