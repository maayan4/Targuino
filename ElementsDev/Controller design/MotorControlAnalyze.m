close all; clear all; clc;
% 
%% Read samples from raw data file and organize in labvectors

fname = 'current';
fileName=['\\tsclient\home\Reps\Targuino\ElementsDev\Controller design\' fname '.txt'];
  
f=fopen(fileName,'r');

line=fgetl(f);

T=zeros(0,1);
vAng=zeros(0,1);
In=zeros(0,1);
vLin=zeros(0,1);
delta=zeros(0,1);
measuredIn=zeros(0,1);
sensor=zeros(0,1);
current=zeros(0,1);

i=1;

while(ischar(line))
    
    [token,rem]=strtok(line);
    
    T=[T; str2double(token(1:length(token)))];
    [token,rem]=strtok(rem);
    
    In=[In; str2double(token(1:length(token)))];  
    
    [token,rem]=strtok(rem);
    vAng=[vAng; str2double(token(1:length(token)))];  
    
    [token,rem]=strtok(rem);
    vLin=[vLin; str2double(token(1:length(token)))];
    
    [token,rem]=strtok(rem);
    delta=[delta; str2double(token(1:length(token)))];
      
    [token,rem]=strtok(rem);
    
    measuredIn=[measuredIn; str2double(token(1:length(token)))];
      [token,rem]=strtok(rem);
    
    sensor=[sensor; str2double(token(1:length(token)))];
     
    [token,rem]=strtok(rem);
    current=[current; str2double(token(1:length(token)))];

    line=fgetl(f);
    
    if(i>50)
        disp(line);
        i=1;
    else
        i=i+1;
    end
end

clear i fileName line token rem;

save motor.mat

fclose(f);

%% model the motor
% mat_path='\home\Reps\';
% mat_fname='switchVNoisy.mat';
% mat_fname_unix='/home/maayan4/Reps/Targuino/';
% mat_osx_fname = '\\tsclient\home\Reps\Targuino\ElementDev\Controller design\MotorControl\motor.mat';
% 
% % load([mat_fname_unix mat_fname]);
% load(mat_osx_fname);

% UdelayV=unique(delayV);
% i=1;
% while(i<length(UdelayV))
%     count(i)=sum(delayV==UdelayV(i));
%     i=i+1;
% end
%xhat=xhat'; PH=PH'; time=time';
Ts=max(diff(T));
data = iddata(measuredIn,In,[],'SamplingInstants',T);
Ts=500e-3;
data = iddata(measuredIn,In,Ts); %assume evenly spaced samples
% datae = misdata(data);
datae = data;
set(datae,'InputName','PWMIn','OutputName','PWMOut');
delay=delayest(datae);
m=[1 1; 2 1; 2 2; 3 1; 3 2; 3 3; 4 1; 4 2; 4 3; 4 4];
matrix=zeros(10,6);
numOfSystems=3; %for each figure
for i=1:size(m,1)
    disp(m(i,:));
    sys=tfest(datae,m(i,1),m(i,2),Ts*delay);
    Controller=pidtune(sys,'pi');
    matrix(i,:)=[m(i,:) sys.Report.Fit.FitPercent Controller.Kp Controller.Ki Controller.Kd];
    eval(['sys' num2str(i) '=sys;']);
    eval(['Controller' num2str(i) '=Controller;']);
    figure(i)
    subplot(2,2,1)
    try
    step(feedback(Controller*sys,1));
    end
    title('step response');
    subplot(2,2,2)
    nyquist(sys); xlim([-0.2 0.2]);
    title('nyquist plot')
    subplot(2,2,3)
    nyquist(feedback(Controller*sys,1))
    title('with PID')
    saveas(gcf,[fname '-' num2str(i)],'png');
    subplot(2,2,4)
    figure(i+10)
    compare(datae,sys);
    title(['system number ' num2str(i)]);
    saveas(gcf,[fname '-' num2str(i+10)],'png');
end

figure();
plot(T, In, T, sensor);
title(['current comparing to input, ' fname]);
saveas(gcf,'current','png');

csvwrite([fname '-matrix'],matrix);


close all;
% save('R2_MA2_lastval_matlab.mat');




