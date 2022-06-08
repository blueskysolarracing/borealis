#Script to sweep input voltage and load current using DP800 and DL3000 to characterize the PSM

import DP800
import DL3000
import pyvisa as pv
import time
import serial

#Serial communication setup
serial = serial.Serial('/dev/cu.usbserial-1420', 115200, timeout=0.002)

#Parameters
voltage_range = [0, 12] #Voltage range to sweep
current_range = [0, 6] #Current range to sweep
delay_bt_measurements = 1000 #Delay between change of voltage or current, in ms
num_data = 20 #Number of data point to take per value (there will be num_data^2 data poimts)
targetID_eLoad = "USB0::6833::3601::DL3A192600119::0::INSTR"
targetID_pSupply = "USB0::6833::3601::DP8A234200572::0::INSTR"

#Initialize e-Load and DC Power Supply
resourceManager = pv.ResourceManager()
eLoad = DL3000(resourceManager.open_resource(targetID_eLoad))
pSupply = DP800(resourceManager.open_resource(targetID_pSupply))

#Reset devices
eLoad.reset()
pSupply.reset()

#Initial settings
pSupply.setVOLT(0)
eLoad.setCurrent(0)
pSupply.channelON()
eLoad.enable()
eLoad.set_mode("CC")
pSupply.selectChannel(1)
    #1: 8V/5A
    #2: 30V/2A
    #3: -30V/2A

#Create file to save results
now = time.datetime.now()
nowStr = now.strftime("%Y%m%d_%H%M%S")
fileName = "PSM_Calibration_" + nowStr + ".csv"
f = open(fileName, "w")

#Run sweep

for i in range(0, num_data):
    eLoad.set_cc_current(i * (current_range[1] - current_range[0])/num_data + current_range[0]) #Increment current
    f.write("Voltage sweep at " + str(eLoad.current()) + "A")
    
    for j in range(0, num_data): #Sweep voltage at given current
        pSupply.setVOLT(j * (voltage_range[1] - voltage_range[0])/num_data + voltage_range[0])
        
        string_received = ""
        while (string_received == ""): #Wait for measurements from Nuke on serial comms
            string_received = serial.readline()
            time.sleep(0.001)

        f.write(string_received + ", " + str(pSupply.voltage()) + ", " + str(eLoad.current())) #Log to CSV file

        time.sleep(delay_bt_measurements / 1000)
