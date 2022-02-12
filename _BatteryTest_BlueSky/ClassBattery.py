import csv

class BatteryObj:

    def __init__(self, cellCapacity, numCells, testSetting, voltageBounds):

        # initialize member variables
        self.m_totalCapacity = numCells*cellCapacity
        self.m_setting = testSetting
        self.m_voltageBounds = voltageBounds

        self.m_measuredVoltage = []
        self.m_timeRec = [] 
        self.m_inputCurrent = []


    def setupCurrentProfile(self, CRate):

        if abs(self.m_setting) == 1:
            
            # set inputCurrent
            self.m_inputCurrent = [self.m_totalCapacity*CRate/1000]

        else:

            self.createDynamic()

        return

    def createDynamic(self):

        return

    def dumpData(self, filename):

        dataCnt = len(self.m_measuredVoltage)
        row = ""

        with open(filename, 'w', encoding = "UTF8") as file:

            file.write("Time Delta, Input Current, Output Voltage \n")

            for i in range(0, dataCnt, 1):

                if self.m_setting != 0:

                    row = str(self.m_timeRec[i]) + ", " + str(self.m_inputCurrent[0]) + ", " + str(self.m_measuredVoltage[i]) + "\n"

                else:
                    
                    row = str(self.m_timeRec[i]) + ", " + str(self.m_inputCurrent[i]) + ", " + str(self.m_measuredVoltage[i]) + "\n"
                
                file.write(row)

        file.close()

        return

    def logMeasurement(self, measurement, timestamp):
        
        self.m_timeRec.append(timestamp)
        self.m_measuredVoltage.append(measurement)

        return
