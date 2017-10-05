hold off
rosshutdown
clear;
clc;
close all;
rosinit('127.0.0.1')
IMU = rossubscriber('/sensor_msgs/imu');
figure(1);
hold on
OAnglexHd = plot(nan);
OAngleyHd = plot(nan);
OAnglezHd = plot(nan);

figure(2);
hold on
ENUAnglexHd = plot(nan);
ENUAngleyHd = plot(nan);
ENUAnglezHd = plot(nan);

figure(3);
hold on
axaxHd = plot(nan);
axayHd = plot(nan);
axazHd = plot(nan);
axawHd = plot(nan);

figure(4);
hold on
ENUaxaxHd = plot(nan);
ENUaxayHd = plot(nan);
ENUaxazHd = plot(nan);
ENUaxawHd = plot(nan);

R = [1 0 0;
    0 0 -1; 
    0 1 0];

R_t = [1 0 0;
    1 0 0; 
    0 1 0];

R_BvBi =   [1 0 0;
             0 0 1; 
            0  -1 0];

R_WenuWned =    [0 1 0;
                1 0 0; 
                0 0 -1];
X=zeros(100,1);
Y_ang=zeros(100,3);
Y_ang_enu=Y_ang;
Y_axa = zeros(100,4);
Y_axa_enu = Y_axa;
while(1)
    IMUdata = receive(IMU,10);
    seq = IMUdata.Header.Seq;
    q = [IMUdata.Orientation.W IMUdata.Orientation.X IMUdata.Orientation.Y IMUdata.Orientation.Z];
    ang_raw = [IMUdata.LinearAcceleration.X IMUdata.LinearAcceleration.Y IMUdata.LinearAcceleration.Z];
    
    dcm = quat2dcm(q);
    ang = quat2eul(q);
    axa = quat2axang(q);
    
    %axa_Bi = [axa(1) axa(3) -axa(2) axa(4)];
    %axa_Bv = [axa(1) -axa(2) -axa(3) axa(4)];
    %dcm_Bv = axang2rotm(axa_Bv);
    dcm_enu = R_WenuWned * dcm * R_BvBi';
    %dcm_enu = dcm_Bv;
    %dcm_enu =  R  * dcm;
    
%     ang_enu = quat2eul(dcm2quat(dcm_enu));
%     axa_enu = quat2axang(dcm2quat(dcm_enu));
    ang_enu = quat2eul(dcm2quat(dcm_enu));
    axa_enu = quat2axang(dcm2quat(dcm_enu));
    
    X = [X ; seq];X=X(2:101);
    Y_ang = [Y_ang ; ang];Y_ang =Y_ang(2:101,:);
    Y_ang_enu = [Y_ang_enu ; ang_enu];Y_ang_enu =Y_ang_enu(2:101,:);
    
    Y_axa = [Y_axa ; axa];Y_axa =Y_axa(2:101,:);
    Y_axa_enu = [Y_axa_enu ; axa_enu];Y_axa_enu =Y_axa_enu(2:101,:);

    set(OAnglexHd,'XData',X,'YData',Y_ang(:,1),'DisplayName','Z');
    set(OAngleyHd,'XData',X,'YData',Y_ang(:,2),'DisplayName','Y');
    set(OAnglezHd,'XData',X,'YData',Y_ang(:,3),'DisplayName','X');
    xlim([X(1) X(100)]);
    ylim([-pi pi]);
    %legend('show')

    
    set(ENUAnglexHd,'XData',X,'YData',Y_ang_enu(:,1),'DisplayName','Z');
    set(ENUAngleyHd,'XData',X,'YData',Y_ang_enu(:,2),'DisplayName','Y');
    set(ENUAnglezHd,'XData',X,'YData',Y_ang_enu(:,3),'DisplayName','X');
    xlim([X(1) X(100)]);
    ylim([-pi pi]);
    %legend('show')

    set(axaxHd,'XData',X,'YData',Y_axa(:,1),'DisplayName','X');
    set(axayHd,'XData',X,'YData',Y_axa(:,2),'DisplayName','Y');
    set(axazHd,'XData',X,'YData',Y_axa(:,3),'DisplayName','Z');
    set(axawHd,'XData',X,'YData',Y_axa(:,4),'DisplayName','w');
    xlim([X(1) X(100)]);
    ylim([-pi pi]);
    %legend('show')
 
    
    set(ENUaxaxHd,'XData',X,'YData',Y_axa_enu(:,1),'DisplayName','X');
    set(ENUaxayHd,'XData',X,'YData',Y_axa_enu(:,2),'DisplayName','Y');
    set(ENUaxazHd,'XData',X,'YData',Y_axa_enu(:,3),'DisplayName','Z');
    set(ENUaxawHd,'XData',X,'YData',Y_axa_enu(:,4),'DisplayName','w');
    xlim([X(1) X(100)]);
    ylim([-pi pi]);
    legend('show')
    drawnow
    
    
end