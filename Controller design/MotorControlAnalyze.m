close all; clear all; clc;
% 
%% Read samples from raw data file and organize in vectors
% fileName='/home/maayan4/Reps/Targuino/log2015-02-28 23:48:53.dat';
%
% f=fopen(fileName,'r');
% 
% line=fgetl(f);
% 
% T=zeros(0,1);
% PwmH=zeros(0,1);
% LV=zeros(0,1);
% AV=zeros(0,1);
% CU=zeros(0,1);
% FA=zeros(0,1);
% FB=zeros(0,1);
% PS=zeros(0,1);
% 
% i=1;
% 
% while(ischar(line))
%     
%     [token,rem]=strtok(line);
%     
%     T=[T; str2double(token(2:length(token)))];
%     [token,rem]=strtok(rem);
%     
%     PwmH=[PwmH; str2double(token(3:length(token)))];  
%     
%     [token,rem]=strtok(rem);
%     LV=[LV; str2double(token(3:length(token)))];  
%     
%     [token,rem]=strtok(rem);
%     AV=[AV; str2double(token(3:length(token)))];
%     
%     [token,rem]=strtok(rem);
%     CU=[CU; str2double(token(3:length(token)))];
%       [token,rem]=strtok(rem);
%     
%     FA=[FA; str2double(token(3:length(token)))];
%       [token,rem]=strtok(rem);
%     
%     FB=[FB; str2double(token(3:length(token)))];
%      
%     [token,rem]=strtok(rem);
%     PS=[PS; str2double(token(3:length(token)))];
% 
%     line=fgetl(f);
%     
%     if(i>1000)
%         disp(line);
%         i=1;
%     else
%         i=i+1;
%     end
% end
% 
% clear i fileName line token rem;
% 
% save motor.mat

%fclose(f);

%% model the motor

load motor.mat

delayV=diff(T);
UdelayV=unique(delayV);
i=1;
while(i<length(UdelayV))
    count(i)=sum(delayV==UdelayV(i));
    i=i+1;
end
    
Ts=max(count);
data = iddata(LV,PwmH,[],'SamplingInstants',time);
%Ts=10e-3;
data = iddata(LV,PwmH,Ts); %assume evenly spaced samples
set(data,'InputName','PWM DutyCycle','OutputName','Velocity');
delay=delayest(data);
sys=tfest(data,1,1,Ts*delay)