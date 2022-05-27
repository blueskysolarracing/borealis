import serial
from matplotlib import pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from time import sleep
from tkinter import *
from tkinter import ttk
from PIL import ImageTk, Image
from threading import Thread
import time as time_mod

#Serial communication setup
serial = serial.Serial('/dev/cu.usbserial-1420', 115200, timeout=0.002)

#Commands
#Generic command encoding is command_name::umppt_id,,value
#Each packet is 16 bytes

STM_cmd = {
    "disable": "DSBL::",
    "new_frequency": "NFRE::",
    "read_frequency": "RFRE::",
    "new_duty_cycle": "NDUT::",
    "read_duty_cycle": "RDUT::",
    "new_phase": "NPHA::",
    "input_voltage": "VINP::",
    "output_voltage": "VOUT::",
    "output_current": "IOUT::",
    "input_current": "IINP::",
    "mpp_voltage": "MPPV::",
    "mpp_current": "MPPI::",
    "output_current_avg": "IAVG",
}

default_duty_cycle = 0.45
default_frequency = 9.50
default_phase = 0.00

length_message = 16
start_time = time_mod.time()
time = []
y_plot1 = []
y_plot2_current = []
y_plot2_power = []
voltage_to_plot = 1 #Indicates which uMPPT input voltage to plot
reset_plot_flag = 0 #Indicates whether plot needs to be reset. Set when pressing "Reset" button and reset at the next animate plot call
counter = 0

log_avg_current_vs_frequency = 0

if log_avg_current_vs_frequency == 1:
    #Textfile to avg current vs frequency
    log_file = open('avg_current_vs_freq_uMPPT1.txt', 'w')
    log_file.write('Average output current vs. switching frequency of uMPPT #1\n')
    log_file.write('Used to determine LUT of correction factor for INA260 gain\n')
    log_file.write('Frequency (kHz); current (A)\n\n')
    log_file.write('Multimeter average (DC): 2.244')


#Set up GUI frame
root = Tk()
root.title("µMPPT Control GUI")
mainframe = ttk.Frame(root, padding="10 10 10 10")
mainframe.grid(column=0, row=0, sticky=(N, W, E, S))
root.columnconfigure(0, weight=1)
root.rowconfigure(0, weight=1)
root.geometry("1450x900")

output_voltage = StringVar(value="0.000V, 0.000A")
output_current = StringVar(value="0.000A")
output_power = StringVar(value="0.000W")

class uMPPT():    
    def __init__(self, uMPPT_id, row_offset):
        self.id = uMPPT_id
        self.row_offset = row_offset
        self.voltage_series = []
        self.current_series = []
        self.live_freq = 0
        self.live_duty_cycle = 0
        self.duty_cycle_reading = StringVar(value="Live: 0.000")
        self.freq_reading = StringVar(value="Live: 0.000")
        self.MPP_reading = StringVar(value="MPP: 0.000V, 0.000A")

        ttk.Label(mainframe, text="uMPPT #" + str(self.id), font=("Arial", 18, "bold")).grid(column=0, row=0+row_offset, sticky=W)

        #Duty cycle input box
        ttk.Label(mainframe, text="Duty cycle").grid(column=0, row=1+row_offset, sticky=W, padx=2)
        self.duty_cycle = StringVar(value=str(default_duty_cycle))
        self.duty_cycle_entry = ttk.Entry(mainframe, width=8, textvariable=self.duty_cycle).grid(column=1, row=1+row_offset, sticky=W)

        #Duty cycle label
        ttk.Label(mainframe, textvariable=self.duty_cycle_reading).grid(column=1, row=1+row_offset)

        #Frequency input box
        ttk.Label(mainframe, text="Frequency (kHz)").grid(column=0, row=2+row_offset, sticky=W, padx=2)
        self.frequency = StringVar(value=str(default_frequency))
        self.frequency_entry = ttk.Entry(mainframe, width=8, textvariable=self.frequency).grid(column=1, row=2+row_offset, sticky=W)

        #Frequency label
        ttk.Label(mainframe, textvariable=self.freq_reading).grid(column=1, row=2+row_offset)
        
        #Phase box
        self.phase = StringVar(value=str(default_phase))
        if self.id == 1:
            ttk.Label(mainframe, text="Phase (°): ").grid(column=0, row=3+row_offset, sticky=W, padx=2)  
            ttk.Label(mainframe, textvariable=self.phase).grid(column=1, row=3+row_offset, sticky=W)
        else:
            ttk.Label(mainframe, text="Phase (°) ").grid(column=0, row=3+row_offset, sticky=W, padx=2)  
            ttk.Entry(mainframe, width=4, textvariable=self.phase).grid(column=1, row=3+row_offset, sticky=W)

        #MPP label
        ttk.Label(mainframe, textvariable=self.MPP_reading).grid(column=0, row=4+row_offset)

        #Input voltage/current string
        self.input_voltage_current = StringVar(value="0.00")
        self.output_voltage = float(self.duty_cycle.get()) * float(self.input_voltage_current.get())

        #Buttons
        #ttk.Button(mainframe, text="Disable", command=self.disable).grid(column=0, row=5+row_offset, sticky=(W, E))
        ttk.Button(mainframe, text="Send", command=self.send_data).grid(column=0, row=5+row_offset, sticky=(W))
        ttk.Button(mainframe, text="Start MPPT", command=self.send_data).grid(column=1, row=5+row_offset, sticky=(W))
        ttk.Button(mainframe, text="Plot", command=self.change_plot).grid(column=1, row=0+row_offset, sticky=(W))

    def disable(self):
        #Send command to disable to uMPPT
        string = STM_cmd["disable"] + str(self.id)
        serial.write(string.encode('utf8'))
        print("Disabled uMPPT #" + str(self.id))

    def send_data(self):
        send_and_receive(STM_cmd["new_duty_cycle"], 0, self.id, self.duty_cycle.get())
        send_and_receive(STM_cmd["new_frequency"], 0, self.id, self.frequency.get())
        send_and_receive(STM_cmd["new_phase"], 0, self.id, self.phase.get())

    def change_plot(self):
        global voltage_to_plot
        global ax
        voltage_to_plot = self.id
        #Set appropriate title
        ax.set_title('µMPPT' + str(voltage_to_plot) + ' Input Voltage')
    
def GUI_loop(arg):
    global root
    global output_voltage
    global output_current
    global output_power

    ###### GUI ######
    #Plot variables
    global ax, ax2, ax2_2
    global line, line2, line3
    global time

    ttk.Label(mainframe, text="µMPPT Dev Dashboard", font=("Comic Sans MS", 23, "bold")).grid(column=1, row=0, sticky=(N, E))

    #Voltages
    ttk.Label(mainframe, text="Input", font=("Arial", 18, "bold")).grid(column=1, row=1, sticky=E)

    #uMPPT input voltages
    for i in range(0,5):
        ttk.Label(mainframe, text="µMPPT #" + str(i+1) + ": ").grid(column=1, row=2+i, sticky=E)
        ttk.Label(mainframe, textvariable=uMPPT_list[i].input_voltage_current).grid(column=2, row=2+i, sticky=(W, E))

    #Output voltage
    ttk.Label(mainframe, text="Output: ").grid(column=1, row=7, sticky=E)
    ttk.Label(mainframe, textvariable=output_voltage).grid(column=2, row=7, sticky=(W, E))

    #Current
    ttk.Label(mainframe, text="Current:", font=("Arial", 18, "bold")).grid(column=1, row=9, sticky=E)
    ttk.Label(mainframe, textvariable=output_current).grid(column=2, row=9, sticky = (W, E))

    #Power
    ttk.Label(mainframe, text="Power:", font=("Arial", 18, "bold")).grid(column=1, row=10, sticky=E)
    ttk.Label(mainframe, textvariable=output_power).grid(column=2, row=10, sticky = (W, E))

    #Plots
    plots = plt.figure(figsize=(7.75, 8.5), dpi=75, facecolor='#ececec')
    
    #plt.minorticks_on()

    #Input voltage
    ax = plots.add_subplot(211)
    ax.set_ylim(0, 5)
    ax.set_ylabel('Voltage (V)')
    ax.set_xlabel('Time (s)')
    ax.grid(visible=True, which='major', color='#111111', linestyle='-')
    ax.grid(visible=True, which='minor', color='#8C8C8C', linestyle='-')
    ax.set_title('µMPPT1 Input Voltage') #Default to uMPPT #1 voltage plot
    line, = ax.plot(time, y_plot1)

    #Output
    # ax2 = plots.add_subplot(212)
    # ax2.set_ylabel('Current (A; blue)')
    # ax2.set_xlabel('Time (s)')
    # ax2.grid(visible=True, which='major', color='#111111', linestyle='-')
    # ax2.grid(visible=True, which='minor', color='#8C8C8C', linestyle='-')
    # ax2.set_title('Output power and current', fontweight = 20)
    # line2, = ax2.plot(time, y_plot2_current)

    # ax2_2 = ax2.twinx()
    # ax2_2.set_ylabel('Power (W; red)')
    # ax2_2.grid(visible=True, which='major', color='#111111', linestyle='-')
    # ax2_2.grid(visible=True, which='minor', color='#8C8C8C', linestyle='-')
    # line3, = ax2_2.plot(time, y_plot2_power, color = 'red')

    plt.subplots_adjust(left=0.1,
                    bottom=0.1, 
                    right=0.9, 
                    top=0.9, 
                    wspace=0.4, 
                    hspace=0.4)
    canvas = FigureCanvasTkAgg(plots, root)
    canvas.get_tk_widget().place(x=520, y=0)

    ttk.Button(mainframe, text="Reset plots", command=reset_plot).grid(column=2, row=16, sticky=(E))

    #Add BSSR logo
    # BSSR_logo = Image.open("bssr_bitmap_black.bmp")
    # BSSR_logo = BSSR_logo.resize((182,60), Image.NEAREST)
    # BSSR_logo = ImageTk.PhotoImage(BSSR_logo)
    # BSSR_logo_label = ttk.Label(mainframe, image=BSSR_logo)
    # BSSR_logo_label.place(x=275, y=600)

    #Add padding to everything
    for child in mainframe.winfo_children(): 
        child.grid_configure(padx = 2, pady = 2)

    #Send default PWM parameters
    for i in range(0, 5):
        uMPPT_list[i].send_data()

    sleep(0.015) #Delay added for stability
    ani1 = animation.FuncAnimation(plots, animate_plot, interval=5, blit=False)
    canvas.draw()

uMPPT1 = uMPPT(1, 1)
uMPPT2 = uMPPT(2, 7)
uMPPT3 = uMPPT(3, 15)
uMPPT4 = uMPPT(4, 22)
uMPPT5 = uMPPT(5, 29)
uMPPT_list = [uMPPT1, uMPPT2, uMPPT3, uMPPT4, uMPPT5]

def meas_loop(arg):
    while(1):
        sleep(0.001)

def send_and_receive(cmd, receive, uMPPT_num = 1, newVal = 1.0):
    global serial
    #Prepare and send data
    string = cmd + str(uMPPT_num) + ',,{:4f}'.format(float(newVal))
    string = pad_message(string, length_message)
    serial.write(string.encode('utf-8'))
    
    time1 = time_mod.time()
    if receive:
        data_received = ''
        while data_received == '':
            data_received = serial.readline() #Read 1 line from serial port
            data_received = data_received.decode('utf-8')

        #Return the data
        return data_received[9:len(data_received)]

def pad_message(message, num_char):
    if len(message) > num_char:
        return message[0:num_char-1]
    elif len(message) == num_char:
        return message
    else:
        return (message + (num_char-len(message))*"0")
    
def animate_plot(i):
    global y_plot1, y_plot2_current, y_plot2_power
    global time
    global line, line2, line3
    global ax, ax2, ax2_2
    global reset_plot_flag
    global start_time
    global counter

    if log_avg_current_vs_frequency == 1:
        counter  = counter + 1

        if (counter == 15):
            counter = 0
            current_freq = float(uMPPT1.frequency.get())
            log_file.write(str(current_freq) + "; " + str(y_plot2_current[-1]) + "\n")

            uMPPT1.frequency.set(str( current_freq + 0.5 ))
            uMPPT1.send_data()

#   Update duty cycle, frequency and voltage reading
    for i in range(0, 5):
        uMPPT_list[i].live_duty_cycle = float(send_and_receive(STM_cmd["read_duty_cycle"], 1, i+1))
        uMPPT_list[i].duty_cycle_reading.set('Live: {:.3f}'.format(uMPPT_list[i].live_duty_cycle))
        uMPPT_list[i].live_freq = float(send_and_receive(STM_cmd["read_frequency"], 1, i+1))
        uMPPT_list[i].freq_reading.set('Live: {:.3f}'.format(uMPPT_list[i].live_freq))
        voltage = float(send_and_receive(STM_cmd["input_voltage"], 1, i+1))
        current = float(send_and_receive(STM_cmd["input_current"], 1, i+1))
        uMPPT_list[i].voltage_series.append(voltage)
        uMPPT_list[i].input_voltage_current.set('{:.3f}V, '.format(float(voltage)) + '{:.3f}A'.format(float(current)))

    if reset_plot_flag == 1:
        start_time = time_mod.time()

        time = []
        uMPPT1.voltage_series = []
        uMPPT2.voltage_series = []
        uMPPT3.voltage_series = []
        uMPPT4.voltage_series = []
        uMPPT5.voltage_series = []
        y_plot2_power = []
        y_plot2_current = []
        reset_plot_flag = 0

    time.append(time_mod.time() - start_time)

    #Change voltage trace to display
    y_plot1 = uMPPT_list[voltage_to_plot-1].voltage_series

    #Update output voltage
    line.set_data(time, y_plot1)
    # line2.set_data(time, y_plot2_current)
    # line3.set_data(time, y_plot2_power)

    if len(time) < 2:
        ax.set_xlim(0, 1)
        # ax2.set_ylim(0, 1)
        # ax2_2.set_ylim(0, 1)
        # ax2.set_xlim(0, 1)

    else:
        ax.set_xlim(time[0], time[-1])
        # ax2.set_ylim(0, max(y_plot2_current)*1.25+0.001)
        # ax2_2.set_ylim(0, max(y_plot2_power)*1.25+0.001)
        # ax2.set_xlim(time[0], time[-1])

    output_voltage_value = float(send_and_receive(STM_cmd["output_voltage"], 1))
    output_voltage.set('{:.3f}V'.format(output_voltage_value))
    
#   Update output current
    y_plot2_current.append(float(send_and_receive(STM_cmd["output_current"], 1)))
    output_current.set('{:.3f}A'.format(y_plot2_current[-1]))

#    Update output power
    #y_plot2_power.append( float(y_plot2_current[-1] * output_voltage_value))
    y_plot2_power.append( 0.5)
    output_power.set('{:.3f}W'.format(y_plot2_power[-1]))

#   Update y-limits
    # ax2.set_ylim(0, max(y_plot2_current)*1.25+0.001)
    # ax2_2.set_ylim(0, max(y_plot2_power)*1.25+0.001)

    # line2.set_data(time, y_plot2_current)
    # line3.set_data(time, y_plot2_power)
    # ax2.set_xlim(time[0], time[-1])

    #return line, line2, line3, ax, ax2, ax2_2

def reset_plot():
    global reset_plot_flag
    reset_plot_flag = 1

if __name__ == "__main__":
    # thread_GUI = Thread(target = GUI_loop, args = (12,))
    # thread_meas = Thread(target = meas_loop, args = (12,))
    # thread_GUI.start()
    GUI_loop(12)
    # thread_meas.start()

    root.mainloop()

serial.close()