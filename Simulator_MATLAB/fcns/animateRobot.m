function [t,COM,HIP,FOOT] = animateRobot(tin,Xin,Uin,Fin,p)

% If you want to generate a .mp4 movie file
boolAnimate = p.boolAnimate;            
if boolAnimate
    name = ['video.mp4'];
    vidfile = VideoWriter(name,'MPEG-4');
    open(vidfile);
end
ylim_scale = 1.5;

N = p.Nstep * p.N_animate;
R = p.Rboom;
L_limit = R*1.2;              % boundary of arena

t = linspace(tin(1),tin(end),N);
X = interp1(tin,Xin,t);
U = interp1(tin,Uin,t);
F = interp1(tin,Fin,t);

k  = 3;
t_ = linspace(tin(1),tin(end),k*N);
X_ = interp1(tin,Xin,t_);
U_ = interp1(tin,Uin,t_);
F_ = interp1(tin,Fin,t_);

figure
set(gcf, 'Color', 'white')
set(gcf, 'Position',  [100, 100, 1400, 600])

HIP = [];
FOOT = [];
COM = [];

h1 = subplot(3,4,3);
h2 = subplot(3,4,7);
h3 = subplot(3,4,11);
h4 = subplot(3,4,4);
h5 = subplot(3,4,8);
h6 = subplot(3,4,12);


nt = length(t);
for ii = 1:nt
    Xii = X(ii,:)';
    HIP(:,end+1)    = fcn_p2(Xii(1:4),p.params);         % Hip trajectory
    FOOT(:,end+1)   = fcn_p_toe(Xii(1:4),p.params);      % Foot trajectory
    COM(:,end+1)    = fcn_CoM(Xii(1:4),p.params);        % CoMtrajectory
    
    %%%%%%%%%%%%%%%%%% Robot %%%%%%%%%%%%%%%%%%
    h_main = subplot(3,4,[1,2,5,6]);
    hold on;grid on;box on;
    plot3(HIP(1,:),HIP(2,:),HIP(3,:),'color',[0, 0, 1],'LineWidth',1);
    plot3(FOOT(1,:),FOOT(2,:),FOOT(3,:),'c','LineWidth',1);
    % ---- plot the robot ------
        plotRobot(Xii,p);
    % --------------------------
    set( get(h_main,'Title'), 'String', ['Time =' num2str(t(ii),'%6.2f') 's']);
    axis equal;
    view(3)
    h_main.XLim = [-L_limit L_limit];
    h_main.YLim = [-L_limit L_limit];
    h_main.ZLim = [0 .5];
    
    %%%%%%%%%%%%%%%%%% Saggital plane %%%%%%%%%%%%%%%%%%
    h_sag = subplot(3,4,[9,10]);
    hold on;grid on; box on;
    plot3(HIP(1,:),HIP(2,:),HIP(3,:),'color',[0, 0, 1],'LineWidth',1);
    plot3(FOOT(1,:),FOOT(2,:),FOOT(3,:),'c','LineWidth',1);
    % ---- plot the robot ------
        plotRobot(Xii,p);
    % --------------------------
    set( get(h_sag,'Title'), 'String','(A) Saggital Plane View');
    view([Xii(1)*180/pi+90 0])
    hold off
    h_sag.XLim = [-L_limit L_limit];
    h_sag.YLim = [-L_limit L_limit];
    h_sag.ZLim = [0 .4];
    
    
    %%%%%%%%%%%%%%%%%% CoM velocity %%%%%%%%%%%%%%%%%%
    plot(h1,t_(1:k*ii),X_(1:k*ii,5)*R,'b',...
            t_(1:k*ii),X_(1:k*ii,6)*R,'r','linewidth',1)
    grid on
    set(h1,'NextPlot','add')
    
    ylim = 5;
    if any(t_(k*ii) > p.tTD(1,:))
        for jj = 1:size(p.tTD,2)
            if (t_(k*ii) > p.tTD(1,jj)) && (t_(k*ii) < p.tTD(2,jj))
                quad = [p.tTD(1,jj) -ylim;
                        p.tTD(1,jj) ylim;
                        t_(k*ii) ylim;
                        t_(k*ii) -ylim];
                fill(h1,quad(:,1),quad(:,2),'r','facealpha',0.1,'edgecolor','none')
            elseif (t_(k*ii) >= p.tTD(2,jj))
                quad = [p.tTD(1,jj) -ylim;
                        p.tTD(1,jj) ylim;
                        p.tTD(2,jj) ylim;
                        p.tTD(2,jj) -ylim];
                fill(h1,quad(:,1),quad(:,2),'r','facealpha',0.1,'edgecolor','none')
            end
            
        end
    end
    h1.YLim = ylim_scale*[min([X_(:,5); X_(:,6)])*R max([X_(:,5); X_(:,6)])*R];
    h1.XLim = [0 t(end)];
    set( get(h1,'YLabel'), 'String', 'CoM Velocity [m/s]' );
    set( get(h1,'Title'), 'String', '(B) v_y (blue); v_z (red)' );
    
    %%%%%%%%%%%%%%%%%% torque %%%%%%%%%%%%%%%%%%
    plot(h2,t_(1:k*ii),U_(1:k*ii,1),'b',...
            t_(1:k*ii),U_(1:k*ii,2),'r','linewidth',1)
    set(h2,'NextPlot','add')
    ylim = 10;
    if any(t_(k*ii) > p.tTD(1,:))
        for jj = 1:size(p.tTD,2)
            if (t_(k*ii) > p.tTD(1,jj)) && (t_(k*ii) < p.tTD(2,jj))
                quad = [p.tTD(1,jj) -ylim;
                        p.tTD(1,jj) ylim;
                        t_(k*ii) ylim;
                        t_(k*ii) -ylim];
                fill(h2,quad(:,1),quad(:,2),'r','facealpha',0.1,'edgecolor','none')
            elseif (t_(k*ii) >= p.tTD(2,jj))
                quad = [p.tTD(1,jj) -ylim;
                        p.tTD(1,jj) ylim;
                        p.tTD(2,jj) ylim;
                        p.tTD(2,jj) -ylim];
                fill(h2,quad(:,1),quad(:,2),'r','facealpha',0.1,'edgecolor','none')
            end
            
        end
    end    
    h2.YLim = ylim_scale*[min([U_(:,1); U_(:,2)]) max([U_(:,1); U_(:,2)])];
    h2.XLim = [0 t(end)];
    grid on
    set( get(h2,'YLabel'), 'String', 'Joint Torque [Nm]' );
    set( get(h2,'Title'), 'String', '(C) Hip (blue) and Knee (red)' );
    
    %%%%%%%%%%%%%%%%%% joint velocity %%%%%%%%%%%%%%%%%%
    plot(h3,t_(1:k*ii),X_(1:k*ii,7),'b',...
            t_(1:k*ii),X_(1:k*ii,8),'r','linewidth',1)
        
    set(h3,'NextPlot','add')
    ylim = 20;
    if any(t_(k*ii) > p.tTD(1,:))
        for jj = 1:size(p.tTD,2)
            if (t_(k*ii) > p.tTD(1,jj)) && (t_(k*ii) < p.tTD(2,jj))
                quad = [p.tTD(1,jj) -ylim;
                        p.tTD(1,jj) ylim;
                        t_(k*ii) ylim;
                        t_(k*ii) -ylim];
                fill(h3,quad(:,1),quad(:,2),'r','facealpha',0.1,'edgecolor','none')
            elseif (t_(k*ii) >= p.tTD(2,jj))
                quad = [p.tTD(1,jj) -ylim;
                        p.tTD(1,jj) ylim;
                        p.tTD(2,jj) ylim;
                        p.tTD(2,jj) -ylim];
                fill(h3,quad(:,1),quad(:,2),'r','facealpha',0.1,'edgecolor','none')
            end
            
        end
    end
    h3.YLim = ylim_scale*[min([X_(:,7); X_(:,8)]) max([X_(:,7); X_(:,8)])];
    h3.XLim = [0 t(end)];
    grid on
    set( get(h3,'XLabel'), 'String', 'Time [s]' );
    set( get(h3,'YLabel'), 'String', 'Joint Velocity [rad/s]' );
    set( get(h3,'Title'), 'String', '(D) Hip (blue) and Knee (red)' );
    
    %%%%%%%%%%%%%%%%%%% HIP Joint torque-omega plot %%%%%%%%%%%%%%%%%%%%%%
    vertices_HIP = [p.uHip.V;p.uHip.V(1,:)];
    plot(h4,X_(1:k*ii,7),U_(1:k*ii,1),'b',...
            X_(k*ii,7),U_(k*ii,1),'bo',...
            vertices_HIP(:,1),vertices_HIP(:,2),'b--','linewidth',1)
    h4.XLim = [-60 60];
    h4.YLim = ylim_scale*[-5 5];
    grid on
    set( get(h4,'XLabel'), 'String', 'Angular Velocity [rad/s]' );
    set( get(h4,'YLabel'), 'String', 'Joint Torque [Nm]' );
    set( get(h4,'Title'), 'String', '(E) Hip (blue)' );
    
    %%%%%%%%%%%%%%%%%%% KNEE Joint torque-omega plot %%%%%%%%%%%%%%%%%%%%%%
    vertices_KNEE = [p.uKnee.V;p.uKnee.V(1,:)];
    plot(h5,X_(1:k*ii,8),U_(1:k*ii,2),'r',...
            X_(k*ii,8),U_(k*ii,2),'ro',...
            vertices_KNEE(:,1),vertices_KNEE(:,2),'r--','linewidth',1)
    h5.XLim = [-60 60];
    h5.YLim = ylim_scale*[-5 6];
    grid on
    set( get(h5,'XLabel'), 'String', 'Angular Velocity [rad/s]' );
    set( get(h5,'YLabel'), 'String', 'Joint Torque [Nm]' );
    set( get(h5,'Title'), 'String', '(F) Knee (red)' );
                
    %%%%%%%%%%%%%%%%%%% Ground Reaction Forces %%%%%%%%%%%%%%%%%%%%%%
    plot(h6,t_(1:k*ii),F_(1:k*ii,1),'b',...
            t_(1:k*ii),F_(1:k*ii,2),'r','linewidth',1)
    set(h6,'NextPlot','add')
    ylim = 50;
    if any(t_(k*ii) > p.tTD(1,:))
        for jj = 1:size(p.tTD,2)
            if (t_(k*ii) > p.tTD(1,jj)) && (t_(k*ii) < p.tTD(2,jj))
                quad = [p.tTD(1,jj) -ylim;
                        p.tTD(1,jj) ylim;
                        t_(k*ii) ylim;
                        t_(k*ii) -ylim];
                fill(h6,quad(:,1),quad(:,2),'r','facealpha',0.1,'edgecolor','none')
            elseif (t_(k*ii) >= p.tTD(2,jj))
                quad = [p.tTD(1,jj) -ylim;
                        p.tTD(1,jj) ylim;
                        p.tTD(2,jj) ylim;
                        p.tTD(2,jj) -ylim];
                fill(h6,quad(:,1),quad(:,2),'r','facealpha',0.1,'edgecolor','none')
            end
        end
    end    
    h6.YLim = ylim_scale*[min([F_(:,1); F_(:,2)]) max([F_(:,1); F_(:,2)])];
    h6.XLim = [0 t(end)];
    grid on    
    set( get(h6,'XLabel'), 'String', 'Time [s]' );
    set( get(h6,'YLabel'), 'String', 'GRF [N]' );
    set( get(h6,'Title'), 'String', '(G) GRFx (blue) and GRFz (red)' ); 
    
    drawnow

    if boolAnimate
        for repeat = 1:3
            writeVideo(vidfile,getframe(gcf))
        end
    end

    if ii < nt
        cla(h_main);
        cla(h_sag);
        cla(h1)
        cla(h2)
        cla(h3)
        cla(h6)
    end
    
end

if boolAnimate
    close(vidfile)
end

end




