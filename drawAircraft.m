function drawAircraft(uu,V,F,patchcolors)

    % process inputs to function
    pn       = uu(1);       % inertial North position     
    pe       = uu(2);       % inertial East position
    pd       = uu(3);           
    u        = uu(4);       
    v        = uu(5);       
    w        = uu(6);       
    phi      = uu(7);       % roll angle         
    theta    = uu(8);       % pitch angle     
    psi      = uu(9);       % yaw angle     
    p        = uu(10);       % roll rate
    q        = uu(11);       % pitch rate     
    r        = uu(12);       % yaw rate    
    t        = uu(13);       % time

       % define persistent variables 
    persistent spacecraft_handle;
    persistent pos_handle; 
    persistent pnhis pehis pdhis; 
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure(1), clf
        spacecraft_handle = drawSpacecraftBody(V,F,patchcolors,...
                                               pn,pe,pd,phi,theta,psi,...
                                               [],'normal');
%         pos_handle=plot3(pe,pn,-pd,'r','linewidth',2);,'EraseMode','xor'
        title('Aircraft')
        xlabel('East')
        ylabel('North')
        zlabel('-Down')
        view(32,47)  % set the vieew angle for figure
        axis([-400,400,-400,400,-400,400]);
        hold on
        grid
         pnhis=pn; pehis=pe; pdhis=pd;
    % at every other time step, redraw base and rod
    else 
        drawSpacecraftBody(V,F,patchcolors,...
                           pn,pe,pd,phi,theta,psi,...
                           spacecraft_handle);
         set(pos_handle,'XData',pe,'YData',pn,'ZData',-pd)
            pnhis=[pnhis;pn]; pehis=[pehis;pe];pdhis=[pdhis;pd];
            psn=[pnhis,pehis,pdhis];
            plot3(psn(:,2),psn(:,1),-psn(:,3),'r','linewidth',2);
                plot3(pe,pn,-pd,'r','linewidth',2,'EraseMode','xor');
    end
end

  
%=======================================================================
% drawSpacecraft
% return handle if 3rd argument is empty, otherwise use 3rd arg as handle
%=======================================================================
%
function handle = drawSpacecraftBody(V,F,patchcolors,...
                                     pn,pe,pd,phi,theta,psi,...
                                     handle,mode)
  V = rotate(V', phi, theta, psi)';  % rotate spacecraft
  V = translate(V', pn, pe, pd)';  % translate spacecraft
  % transform vertices from NED to XYZ (for matlab rendering)
  R = [...
      0, 1, 0;...
      1, 0, 0;...
      0, 0, -1;...
      ];
  V = V*R;
  
  if isempty(handle),
  handle = patch('Vertices', V, 'Faces', F,...
                 'FaceVertexCData',patchcolors,...
                 'FaceColor','flat',...
                 'EraseMode', mode);
  else
    set(handle,'Vertices',V,'Faces',F);
    drawnow
  end
end

%%%%%%%%%%%%%%%%%%%%%%%
function XYZ=rotate(XYZ,phi,theta,psi);
  % define rotation matrix
  R_roll = [...
          1, 0, 0;...
          0, cos(phi), -sin(phi);...
          0, sin(phi), cos(phi)];
  R_pitch = [...
          cos(theta), 0, sin(theta);...
          0, 1, 0;...
          -sin(theta), 0, cos(theta)];
  R_yaw = [...
          cos(psi), -sin(psi), 0;...
          sin(psi), cos(psi), 0;...
          0, 0, 1];
  R = R_yaw*R_pitch*R_roll;
  % rotate vertices
  XYZ = R*XYZ;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% translate vertices by pn, pe, pd
function XYZ = translate(XYZ,pn,pe,pd)
  XYZ = XYZ + repmat([pn;pe;pd],1,size(XYZ,2));
end

  