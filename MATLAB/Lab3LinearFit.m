%Linear fitter for lab 3
%Written by Sonia Berliner
scopeData = readtable('"filename".csv', 'PreserveVariableNames', true) % filename is a placeholder, fill it in with the name of the file being analyzed
sampleNumber = scopeData(:,1); % x-axis for the linear plot, name may change depending on what we are measuring
output = scopeData(:,2); %y-axis for linear plot

format long %for more precision

padded = [ones(length(scopeData),1) scopeData]; %this adds a column of ones to facilitate calculating the intercept

regression = padded\output %regression with y-intercept value

yvalues = padded*regression; %this is the vector of y-values of the regression
plot(sampleNumber,yvalues,'--')
legend('Data','Slope & Intercept');


scatter(sampleNumber,output)
hold on
plot(sampleNumber,yvalues)
xlabel('Oscilloscope sample number')
ylabel('90 Degree/Transmission Vpp measurement')
title('Linear Regression of Sample Number vs 90 degree/transmission Vpp')
grid on