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
mat_path='\home\Reps\';
mat_fname='switchVNoisy.mat';
mat_fname_unix='/home/maayan4/Reps/Targuino/';

load([mat_fname_unix mat_fname]);

% UdelayV=unique(delayV);
% i=1;
% while(i<length(UdelayV))
%     count(i)=sum(delayV==UdelayV(i));
%     i=i+1;
% end
xhat=xhat'; PH=PH'; time=time';
Ts=max(diff(time));
data = iddata(xhat',PH',[],'SamplingInstants',time);
%Ts=10e-3;
data = iddata(xhat',PH',Ts); %assume evenly spaced samples
set(data,'InputName','PWM DutyCycle','OutputName','Velocity');
delay=delayest(data);
m=[1 1; 2 1; 2 2; 3 1; 3 2; 3 3; 4 1; 4 2; 4 3; 4 4];
matrix=zeros(15,6);
numOfSystems=3; %for each figure
for i=1:size(m,2)
    disp(m(i,:));
    sys=tfest(data,m(i,1),m(i,2),Ts*delay);
    Controller=pidtune(sys,'pid',6);
    matrix(i,:)=[m(i,:) sys.Report.Fit.FitPercent Controller.Kp Controller.Ki Controller.Kd];
    eval(['sys' num2str(i) '=sys;']);
    eval(['Controller' num2str(i) '=Controller;']);
    figure(i)
    subplot(1,3,1)
    try
    step(feedback(Controller*rsample(sys,10),1));
    end
    hold on;
    try
    step(feedback(Controller*sys,1),'r');
    end
    title('step response');
    subplot(1,3,2)
    nyquist(sys); xlim([-0.2 0.2]);
    title('nyquist plot')
    subplot(1,3,3)
    nyquist(feedback(Controller*sys,1))
    title('with PID')
    saveas(gcf,['system' num2str(i)],'png');
end


% save('R2_MA2_lastval_matlab.mat');



