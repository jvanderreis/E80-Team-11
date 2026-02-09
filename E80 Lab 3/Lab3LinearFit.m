%Linear fitter for lab 3
%Written by Sonia Berliner
Data = readtable('"filename".csv', 'PreserveVariableNames', true) % filename is a placeholder, fill it in with the name of the file being analyzed.
% Download the spreadsheet with data in it as a .csv file with a unique and
% descriptive name.
turbidity = Data(:,1); % x-axis for the linear plot, name may change depending on what happens in office hours
voltage = Data(:,2); %y-axis for linear plot, measures voltage ratios


format long %for more precision

padded = [ones(length(turbidity),1) turbidity]; %this adds a column of ones to facilitate calculating the intercept

regression = padded\voltage; %regression with y-intercept value

yvalues = padded*regression; %this is the vector of y-values of the regression
plot(turbidity,yvalues,'--')
legend('Data','Slope & Intercept');


scatter(turbidity,voltage)
hold on
plot(sampleNumber,yvalues)
xlabel('Turbidity')
ylabel('90 Degree/Transmission Vpp measurement')
title('Linear Regression of Turbidity vs 90 degree/transmission Vpp')
grid on