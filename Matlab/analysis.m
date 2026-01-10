%% loads
clear all; close all; clc;

shapes = {"custom", "square"};
select = 1;

load(shapes{select} + "_c_on_B.mat")

bp.X(:,1:3) = [x, y, theta];
bp.U(:,1:2) = [v_r, w_r];
bp.Xd(:,1:3) = [xd', yd', thetad];
bp.dXd(:,1:2) = [dxd', dyd'];
bp.ddXd(:,1:2) = [ddxd', ddyd'];

load(shapes{select} + "_c_on_CM.mat")

cm.X(:,1:3) = [x, y, theta];
cm.U(:,1:2) = [v_r, w_r];
cm.Xd(:,1:3) = [xd', yd', thetad];
cm.dXd(:,1:2) = [dxd', dyd'];
cm.ddXd(:,1:2) = [ddxd', ddyd'];

clearvars -except bp cm t shapes select

%% plots

xlimts = [5.5 7.5];

figure(Name="Errors X")
plot(t, cm.X(:,1) - cm.Xd(:,1), t, bp.X(:,1) - bp.Xd(:,1), LineWidth=1.5 )
grid on;
if strcmp(shapes{select}, "square")
    xlim(xlimts); ylim([-0.25, 0.05]);
end
legend("CM", "B"); xlabel("t [s]"); ylabel("Tracking error [m]"); title("Error X")

figure(Name="Errors Y")
plot(t, cm.X(:,2) - bp.Xd(:,2), t, bp.X(:,2) - bp.Xd(:,2), LineWidth=1.5 )
grid on;
if strcmp(shapes{select}, "square")
    xlim(xlimts); ylim([-0.25, 0.05]);
end
legend("CM", "B"); xlabel("t [s]"); ylabel("Tracking error [m]"); title("Error Y")

figure(Name="Input v")
plot(t, cm.U(:,1), t, bp.U(:,1), LineWidth=1.5 )
grid on;
if strcmp(shapes{select}, "square")
    xlim(xlimts); 
else
    ylim([0 1.5])
end
legend("CM", "B"); xlabel("t [s]"); ylabel("v [m/s]"); title("Input v")

figure(Name="Input w")
plot(t, cm.U(:,2), t, bp.U(:,2), LineWidth=1.5 )
grid on;
if strcmp(shapes{select}, "square")
    xlim(xlimts); 
else
    ylim([-2 2])
end

legend("CM", "B"); xlabel("t [s]"); ylabel("w [rad/s]"); title("Input w")

