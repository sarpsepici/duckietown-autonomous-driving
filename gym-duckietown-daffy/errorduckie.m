
clc
clear all
close all

% 14.745 s for original (Ti = 1000, Td = 10)
% 13.89 s for tuned (Ti = 3000, Td = 15)

Error = load('errorvaluesorig.txt');
Error2 = load('errorvaluestuned.txt');

plot(Error)
hold on
plot(Error2)
xlabel('step count (1/30 s)')
ylabel('distance error')
legend({'Hand tuned parameters','Brute Force '},'Location','southeast')

mse = trapz(Error.*Error)
mse2 = trapz(Error2.*Error2)