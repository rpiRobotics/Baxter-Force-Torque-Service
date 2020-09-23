% FT Timing Test

% This script is for tracking the performance of the ATI FT sensor
% installed on Baxter
% Run the Robot Raconteur script before this script

clear variables; close all; clc

%ati_sensor = RRconnect('')
c_host=RobotRaconteur.ConnectService('tcp://localhost:5300/sensors.ati/ATImini45Host');
%Use objref's to pull out the cameras. c_host is a "WebcamHost" type
%and is used to find the webcams
ft_sensor=c_host.get_ft(0);
%Connect to the stream pipe
%filename in startRecordingData currently does nothing, but could be
%reenahled in Program.cs code to capture data
ft_sensor.startRecordingData('output2');
ft_sensor.bias();
p=ft_sensor.FTDataStream.Connect(-1); % define pipe
% im=p.ReceivePacket(); % use this to pull data from the pipe
% ft_sensor.wrench; % use this to get most up to date answer

num_samples = 10000;
t = zeros(1,num_samples);
ft = zeros(6,num_samples);
k = 1;
tic
while k<=num_samples
    ft(:,k) = ft_sensor.wrench;
    t(k) = toc; 
    k = k+1;
end