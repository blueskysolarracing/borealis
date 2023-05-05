function [R_in, R_CT, C_CT] = getBatteryParameters(file,CRate)

% Load data from file (.CSV) formatted as below
% {'TimeStamp', 'TimeDelta', 'InputCurrent', 'OutputVoltage'}
data = readtable(file);

idx_Time = 1;
idx_DeltaT = 2;
idx_InCurr = 3;
idx_VoltOut = 4;

% Time for the 1st pulse to start (seconds)
T_pulse_start = 50;
% duration of pulse (seconds) for 10% SOC
T_pulse_end = 360/CRate + T_pulse_start;

I_Rin_0 = 0;
V_Rin_0 = 0;
V_R_CT_0 = 0;
I_R_CT = 0;
T_Rin = 0;
T_RCT = 0;
computeStage = 0;

n = height(data);
prev_Volt = 0;

for i = 2:n
    
    if data{i, idx_Time} > T_pulse_start
        
        deltaV = data{i, idx_VoltOut} - prev_Volt;
        
        if abs(deltaV) > 0.05 && computeStage == 0
            
            I_Rin_0 = data{i, idx_InCurr};
            V_Rin_0 = data{i-1, idx_VoltOut};
            T_Rin = data{i-1, idx_Time};
            computeStage = 1;
            continue
            
        end
        
        if computeStage == 1
            
            if data{i, idx_Time} - T_Rin > 0.015
                
                R_in = (V_Rin_0 - data{i, idx_VoltOut})/I_Rin_0;
                T_RCT = data{i, idx_Time} + 1 ;
                computeStage = 2;
                continue
            
            end
        end
        
        if computeStage == 2 && data{i, idx_Time} > T_RCT
            
             V_R_CT_0 = data{i, idx_VoltOut};
             I_R_CT = data{i, idx_InCurr};
             computeStage = 3;
             continue;
            
        end
        
        if computeStage == 3 && deltaV > 0 && data{i, idx_Time} > T_pulse_end
            
            R_CT = (V_R_CT_0 - data{i-1, idx_VoltOut})/I_R_CT - R_in;
            T_RCT = data{i, idx_Time};
            computeStage = 4;
            continue
            
        end

        if computeStage == 4 && deltaV ~= 0 && deltaV < 0.00009
    
            C_CT = (data{i, idx_Time} - T_RCT)/R_CT;
            break

        end
        
    end
    
    prev_Volt = data{i, idx_VoltOut};
    
end

end