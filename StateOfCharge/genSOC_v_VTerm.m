function result = genSOC_v_VTerm(file, Q, mode)

% Data format inside file (.CSV)
% {'TimeStamp','TimeDelta','InputCurrent','OutputVoltage'}
% Q - nominal capacity in mAh
% mode - (1) or (-1) for dischargin or charging
data = readtable(file);
dt_idx = 2;
inCurr_idx = 3;
outVolt_idx = 4;
n = height(data);

Cte = 1000/3600;

% Assume we start from a fully charged battery.
outSOC = [1];
outVTerm = [data{2, outVolt_idx}];

% Do Coulomb counting.
for i = 3:n
    
    tempSOC = outSOC(i - 2) - (Cte*mode*data{i, dt_idx}*data{i, inCurr_idx})/Q;
    
    if tempSOC < 0 
        break
    else
        outSOC = [outSOC; tempSOC];
        outVTerm = [outVTerm; data{i, outVolt_idx}];
    end
end

% Create output file.

figure
plot(outVTerm, outSOC);
title('Terminal Voltage vs SOC');
legend('SOC', 'Location', 'northwest');
xlim([outVTerm(i-1), outVTerm(1)]);
ylim([0,1]);
xlabel('Voltage (V)');
ylabel('SOC');
hold on

time = clock;
outFile = 'VTerm_v_SOC_' + string(time(4)) + string(time(5)) + '.csv';

outTable = table(outVTerm, outSOC);
writetable(outTable, outFile);



result = true;

end