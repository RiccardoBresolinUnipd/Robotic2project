%% loads
clear all; close all; clc;

shapes = {"custom", "square"};
select = 1;

load(".\collected_data\"+shapes{select} + "_c_on_B.mat")

bp.X(:,1:3) = [x, y, theta];
bp.U(:,1:2) = [v_r, w_r];
bp.Xd(:,1:3) = [xd', yd', thetad];
bp.dXd(:,1:2) = [dxd', dyd'];
bp.ddXd(:,1:2) = [ddxd', ddyd'];

load(".\collected_data\"+shapes{select} + "_c_on_CM.mat")

cm.X(:,1:3) = [x, y, theta];
cm.U(:,1:2) = [v_r, w_r];
cm.Xd(:,1:3) = [xd', yd', thetad];
cm.dXd(:,1:2) = [dxd', dyd'];
cm.ddXd(:,1:2) = [ddxd', ddyd'];

clearvars -except bp cm t shapes select

%% plots errors
save = false;
xlimts = [5.5 7.5];

figure(Name="Errors X")
plot(t, cm.X(:,1) - cm.Xd(:,1), t, bp.X(:,1) - bp.Xd(:,1), LineWidth=1.5 )
grid on; axis("square")
if strcmp(shapes{select}, "square")
    xlim(xlimts); ylim([-0.25, 0.05]);
end
legend("CoM", "Bp"); xlabel("t [s]"); ylabel("Tracking error [m]"); title("Error X")
if save
    f = gcf;
    exportgraphics(gcf, ".\corrected_images\" + shapes{select} + "_ErrorX.png", "Resolution",300)
end


figure(Name="Errors Y")
plot(t, cm.X(:,2) - bp.Xd(:,2), t, bp.X(:,2) - bp.Xd(:,2), LineWidth=1.5 )
grid on; axis("square")
if strcmp(shapes{select}, "square")
    xlim(xlimts); ylim([-0.25, 0.05]);
end
legend("CoM", "Bp"); xlabel("t [s]"); ylabel("Tracking error [m]"); title("Error Y")
if save
    f = gcf;
    exportgraphics(gcf, ".\corrected_images\" + shapes{select} + "_ErrorY.png", "Resolution",300)
end

%% plots Inputs
save = false;

figure(Name="Input v")
plot(t, cm.U(:,1), t, bp.U(:,1), LineWidth=1.5 )
grid on; axis("square")
if strcmp(shapes{select}, "square")
    xlim(xlimts); 
else
    ylim([0 1.5])
end
legend("CoM", "Bp"); xlabel("t [s]"); ylabel("v [m/s]"); title("Input v")
if save
    f = gcf;
    exportgraphics(gcf, ".\corrected_images\" + shapes{select} + "_InputV.png", "Resolution",300)
end

figure(Name="Input w")
plot(t, cm.U(:,2), t, bp.U(:,2), LineWidth=1.5 )
grid on; axis("square")
if strcmp(shapes{select}, "square")
    xlim(xlimts); 
else
    ylim([-2 2])
end
legend("CoM", "Bp"); xlabel("t [s]"); ylabel("w [rad/s]"); title("Input w")
if save
    f = gcf;
    exportgraphics(gcf, ".\corrected_images\" + shapes{select} + "_InputW.png", "Resolution",300)
end

%% plots trajectories
save = true;

figure(Name="Trajectory"); 
total = subplot(2,2,[1 3]);
grid(total, "on"); hold(total,"on");
plot(total, cm.X(:,1), cm.X(:,2), LineWidth=1.5 );
plot(total, bp.X(:,1), bp.X(:,2), LineWidth=1.5 ); 
plot(total, cm.Xd(:,1), cm.Xd(:,2), "--", LineWidth=3); 
legend(total,"CoM", "Bp", "Desired"); xlabel("x [m]"); ylabel("y [m]");

zoom1 = subplot(2,2,2); 
grid(zoom1, "on"); hold(zoom1,"on");
xlim(zoom1, [2.2 4.2]); ylim(zoom1, [2 2.8]);
plot(zoom1, cm.X(:,1), cm.X(:,2), LineWidth=1.5 ); 
plot(zoom1, bp.X(:,1), bp.X(:,2), LineWidth=1.5 );
plot(zoom1, cm.Xd(:,1), cm.Xd(:,2), "--", LineWidth=3); 
legend(zoom1, "CoM", "Bp", "Desired");
xlabel("x [m]"); ylabel("y [m]");

zoom2 = subplot(2,2,4); 
grid(zoom2, "on"); hold(zoom2,"on");
xlim(zoom2, [4.2 5.2]); ylim(zoom2, [-3.8 -3]);
plot(zoom2, cm.X(:,1), cm.X(:,2), LineWidth=1.5 ); 
plot(zoom2, bp.X(:,1), bp.X(:,2), LineWidth=1.5 );
plot(zoom2, cm.Xd(:,1), cm.Xd(:,2), "--", LineWidth=3); 
legend(zoom2, "CoM", "Bp", "Desired");
xlabel("x [m]"); ylabel("y [m]");


if save
    f = gcf;
    exportgraphics(gcf, ".\corrected_images\" + shapes{select} + "_trajectory.png", "Resolution",300)
end