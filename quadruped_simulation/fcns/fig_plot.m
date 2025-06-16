function [t,EA,EAd] = fig_plot(tout,Xout,Uout,Xdout,Udout,Uext,p)
%% smoothen for plotting
t = tout(1):p.simTimeStep:tout(end);
X = interp1(tout,Xout,t);
U = interp1(tout,Uout,t);
Xd = interp1(tout,Xdout,t);
Ud = interp1(tout,Udout,t);

%% Calculate Euler angles
nt = length(t);
EA = zeros(nt,3);
EAd = zeros(nt,3);
for ii = 1:nt
    EA(ii,:) = fcn_X2EA(X(ii,:));
    EAd(ii,:) = fcn_X2EA(Xd(ii,:));
end

% For ZOH force plotting
t2 = repelem(t,2);
t2(1) = []; t2(end+1) = t2(end);
U2 = repelem(U,2,1);

%% Create figure with 2x2 subplots
figure('Position',[200 100 1200 800]);
set(gcf, 'Color', 'white')

% Create subplots with more space for legends
ax1 = subplot(2,2,1);
plot(t,X(:,1),'r', t,X(:,2),'g', t,X(:,3),'b',...
     t,Xd(:,1),'r--', t,Xd(:,2),'g--', t,Xd(:,3),'b--','linewidth',1);
xlim([t(1) t(end)]);
title('Position [m]');
lgd1 = legend({'x','y','z','x_d','y_d','z_d'}, 'Location','eastoutside');

ax2 = subplot(2,2,2);
plot(t,X(:,4),'r', t,X(:,5),'g', t,X(:,6),'b',...
     t,Xd(:,4),'r--', t,Xd(:,5),'g--', t,Xd(:,6),'b--','linewidth',1);
xlim([t(1) t(end)]);
title('Velocity [m/s]');
lgd2 = legend({'vx','vy','vz','vx_d','vy_d','vz_d'}, 'Location','eastoutside');

ax3 = subplot(2,2,3);
plot(t,X(:,16),'r', t,X(:,17),'g', t,X(:,18),'b',...
     t,Xd(:,16),'r--', t,Xd(:,17),'g--', t,Xd(:,18),'b--','linewidth',1);
xlim([t(1) t(end)]);
title('Angular velocity [rad/s]');
lgd3 = legend({'wx','wy','wz','wx_d','wy_d','wz_d'}, 'Location','eastoutside');

ax4 = subplot(2,2,4);
plot(t2,U2(:,3),'r', t2,U2(:,6),'g', t2,U2(:,9),'b', t2,U2(:,12),'k',...
     t,Ud(:,3),'r--', t,Ud(:,6),'g--', t,Ud(:,9),'b--', t,Ud(:,12),'k--','linewidth',1);
xlim([t(1) t(end)]);
title('Fz [N]');
lgd4 = legend({'Fz_1','Fz_2','Fz_3','Fz_4','Fz1_d','Fz2_d','Fz3_d','Fz4_d'},...
             'Location','eastoutside');

% Adjust subplot positions to make space for legends
pos1 = get(ax1, 'Position');
set(ax1, 'Position', [pos1(1) pos1(2) pos1(3)*0.85 pos1(4)]);

pos2 = get(ax2, 'Position');
set(ax2, 'Position', [pos2(1) pos2(2) pos2(3)*0.85 pos2(4)]);

pos3 = get(ax3, 'Position');
set(ax3, 'Position', [pos3(1) pos3(2) pos3(3)*0.85 pos3(4)]);

pos4 = get(ax4, 'Position');
set(ax4, 'Position', [pos4(1) pos4(2) pos4(3)*0.85 pos4(4)]);
end
