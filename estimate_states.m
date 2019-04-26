% estimate_states
%   - estimate the MAV states using gyros, accels, pressure sensors, and
%   GPS.
%
% Outputs are:
%   pnhat    - estimated North position, 
%   pehat    - estimated East position, 
%   hhat     - estimated altitude, 
%   Vahat    - estimated airspeed, 
%   alphahat - estimated angle of attack
%   betahat  - estimated sideslip angle
%   phihat   - estimated roll angle, 
%   thetahat - estimated pitch angel, 
%   chihat   - estimated course, 
%   phat     - estimated roll rate, 
%   qhat     - estimated pitch rate, 
%   rhat     - estimated yaw rate,
%   Vghat    - estimated ground speed, 
%   wnhat    - estimate of North wind, 
%   wehat    - estimate of East wind
%   psihat   - estimate of heading angle
% 
% 

function xhat = estimate_states(uu, P)

   % rename inputs
   y_gyro_x      = uu(1);
   y_gyro_y      = uu(2);
   y_gyro_z      = uu(3);
   y_accel_x     = uu(4);
   y_accel_y     = uu(5);
   y_accel_z     = uu(6);
   y_static_pres = uu(7);
   y_diff_pres   = uu(8);
   y_gps_n       = uu(9);
   y_gps_e       = uu(10);
   y_gps_h       = uu(11);
   y_gps_Vg      = uu(12);
   y_gps_course  = uu(13);
   t             = uu(14);
   
    % define persistent variables
    persistent alpha  % constant for low pass filter - only compute once
    persistent lpf_gyro_x   % low pass filter of x-gyro
    persistent lpf_gyro_y   % low pass filter of y-gyro
    persistent lpf_gyro_z   % low pass filter of z-gyro
    persistent lpf_static   % low pass filter of static pressure sensor
    persistent lpf_diff     % low pass filter of diff pressure sensor
    persistent lpf_accel_x  % low pass filter of x-accelerometer
    persistent lpf_accel_y  % low pass filter of y-accelerometer
    persistent lpf_accel_z  % low pass filter of z-accelerometer
    persistent xhat_a       % estimate of roll and pitch
    persistent P_a          % error covariance for roll and pitch angles
    persistent xhat_p       % estimate of pn, pe, Vg, chi, wn, we, psi
    persistent P_p          % error covariance for pn, pe, Vg, chi, wn, we, psi
    persistent y_gps_n_old  % last measurement of gps_n - used to detect new GPS signal
    persistent y_gps_e_old  % last measurement of gps_e - used to detect new GPS signal
    persistent y_gps_Vg_old % last measurement of gps_Vg - used to detect new GPS signal
    persistent y_gps_course_old  % last measurement of gps_course - used to detect new GPS signal
    
    
    
    % initialize persistent variables
    lpf_a = 10;
    if t==0,
        alpha = exp(-lpf_a*P.Ts);
        lpf_gyro_x   =0;
        lpf_gyro_y   =0; 
        lpf_gyro_z   =0; 
        lpf_static   =0;
        lpf_diff     =0;
        lpf_accel_x  =0;
        lpf_accel_y  =0;
        lpf_accel_z  =0;       
        xhat_a       =[0.2;0.1] ;
        P_a          =eye(2)*1 ;
        xhat_p       = [0;0;0;0;0;0;0];
        P_p          =eye(7) ;
        y_gps_n_old  = -9999;
        y_gps_e_old  = -9999;
        y_gps_Vg_old = -9999;
        y_gps_course_old  = -9999;
    end
    
    %------------------------------------------------------------------
    % low pass filter gyros to estimate angular rates
    phat = alpha*lpf_gyro_x+(1-alpha)*y_gyro_x;
    qhat = alpha*lpf_gyro_y+(1-alpha)*y_gyro_y;
    rhat = alpha*lpf_gyro_z+(1-alpha)*y_gyro_z;
    
    %------------------------------------------------------------------
    % low pass filter static pressure sensor and invert to estimate
    % altitude
    lpf_static=alpha*lpf_static+(1-alpha)*y_static_pres;
    hhat = lpf_static/P.rho*P.gravity;
    
    % low pass filter diff pressure sensor and invert to estimate Va
    lpf_diff=alpha*lpf_diff+(1-alpha)*y_diff_pres;
    Vahat = sqrt(2/P.rho*lpf_diff);
    
    %------------------------------------------------------------------
%     % low pass filter accelerometers
%      
%     % invert accels to estimate phi and theta
%     phihat_accel = ;
%     thetahat_accel = ;
%         
    %-------------------------------------------------------------------
    % implement continous-discrete EKF to estimate roll and pitch angles
    Q_a =eye(2)*1 ;  % tune filter with this variable
    R_accel = P.sigma_accel^2*eye(3);
    
    N = 10;
    % prediction step
    for i=1:N,
        xhat_a =xhat_a+(P.Ts/N)*[phat+qhat*sin(xhat_a(1))*tan(xhat_a(2))+rhat*cos(xhat_a(1))*tan(xhat_a(2));...
            qhat*cos(xhat_a(1))-rhat*sin(xhat_a(1))] ;
        A_a=[qhat*cos(xhat_a(1))*tan(xhat_a(2))-rhat*sin(xhat_a(1))*tan(xhat_a(2)) (qhat*sin(xhat_a(1))+rhat*cos(xhat_a(1)))/cos(xhat_a(2))^2;...
            -qhat*sin(xhat_a(1))-rhat*cos(xhat_a(1)) 0];
        P_a = P_a+(P.Ts/N)*(A_a*P_a+P_a*A_a'+Q_a);
    end
    % measurement updates
    % x-axis accelerometer
    h_a =[qhat*Vahat*sin(xhat_a(2))+P.gravity*sin(xhat_a(2));...
        rhat*Vahat*cos(xhat_a(2))-phat*Vahat*sin(xhat_a(2))-P.gravity*cos(xhat_a(2))*sin(xhat_a(1));...
        -qhat*Vahat*cos(xhat_a(2))-P.gravity*cos(xhat_a(2))*cos(xhat_a(1))] ;
    C_a =[0 qhat*Vahat*cos(xhat_a(2))+P.gravity*cos(xhat_a(2));...
        -P.gravity*cos(xhat_a(2))*cos(xhat_a(1)) -rhat*Vahat*sin(xhat_a(2))-phat*Vahat*cos(xhat_a(2))+P.gravity*sin(xhat_a(2))*sin(xhat_a(1));...
        P.gravity*cos(xhat_a(2))*sin(xhat_a(1)) (qhat*Vahat+P.gravity*cos(xhat_a(1)))*sin(xhat_a(2))] ;
    L_a=P_a*C_a'*inv(R_accel+C_a*P_a*C_a');
    P_a = (eye(2)-L_a*C_a)*P_a;
    xhat_a = xhat_a+L_a*([y_accel_x;y_accel_y;y_accel_z]-h_a);
%     % y-axis accelerometer
%     P_a = ;
%     xhat_a = ;
%     % z-axis accelerometer
%     P_a = ;
%     xhat_a = ;
%     
     
    phihat   = xhat_a(1);
    thetahat = xhat_a(2);
    
        %-------------------------------------------------------------------
    % implement continous-discrete EKF to estimate pn, pe, chi, Vg
    Q_p = eye(7)*5
    R_p = diag([...
        P.sigma_n_gps^2,...      % y_gps_n
        P.sigma_e_gps^2,...      % y_gps_e
        P.sigma_Vg_gps^2,...     % y_gps_Vg
        P.sigma_course_gps^2,... % y_gps_course
        0.001,...              % pseudo measurement #1
        0.001,...              % pseudo measurement #2
        ]);
    
    N = 10;
    % prediction step
    for i=1:N,
        xhat_p = xhat_p+(P.Ts/N)*...
            [xhat_p(3)*cos(xhat_p(4));...
            xhat_p(3)*sin(xhat_p(4));...
            (Vahat*xhat_p(7)*(xhat_p(6)*cos(xhat_p(7))-xhat_p(5)*sin(xhat_p(7))))/xhat_p(3);...
            P.gravity/xhat_p(3)*tan(phihat);...
            0;...
            0;...
            qhat*sin(phihat)/cos(thetahat)+rhat*cos(phihat)/cos(thetahat)];
        
        Vdot_g=(Vahat*xhat_p(7)*(xhat_p(6)*cos(xhat_p(7))-xhat_p(5)*sin(xhat_p(7))))/xhat_p(3);
        psidot=qhat*sin(phihat)/cos(thetahat)+rhat*cos(phihat)/cos(thetahat);
        dVgdotDpsi=-psidot*xhat_p(3)*(xhat_p(5)*cos(xhat_p(7))+xhat_p(6)*sin(xhat_p(7)));
        A_p=[0 0 cos(xhat_p(4)) -xhat_p(3)*sin(xhat_p(4)) 0 0 0;...
             0 0 sin(xhat_p(4))  xhat_p(3)*cos(xhat_p(4)) 0 0 0;...
             0 0 -(Vdot_g)/xhat_p(3) 0 -psidot*Vahat*xhat_p(7)/xhat_p(3) 0 dVgdotDpsi;...
             0 0 P.gravity/xhat_p(3)^2*tan(phihat) 0 0 psidot*Vahat*cos(xhat_p(7)) 0;...
             0 0 0 0 0 0 0;...
             0 0 0 0 0 0 0;...
             0 0 0 0 0 0 0]
        P_p = P_p+(P.Ts/N)*(A_p*P_p+P_p*A_p'+Q_p)
    end
    % measurement updates
    if   (y_gps_n~=y_gps_n_old)...
        |(y_gps_e~=y_gps_e_old)...
        |(y_gps_Vg~=y_gps_Vg_old)...
        |(y_gps_course~=y_gps_course_old),
        % gps North position
        h_p = [ xhat_p(1); xhat_p(2); xhat_p(3); xhat_p(4);Vahat*cos( xhat_p(7))+ xhat_p(5)- xhat_p(3)*cos( xhat_p(4));...
            Vahat*sin( xhat_p(7))+ xhat_p(6)- xhat_p(3)*sin( xhat_p(4))];
        C_p = [1 0 0 0 0 0 0;...
               0 1 0 0 0 0 0;...
               0 0 1 0 0 0 0;...
               0 0 0 1 0 0 0;...
               0 0 -cos(xhat_p(4)) xhat_p(3)*sin( xhat_p(4)) 1 0 -Vahat*sin( xhat_p(7));...
               0 0 -sin(xhat_p(4)) -xhat_p(3)*cos( xhat_p(4)) 0 1  Vahat*cos( xhat_p(7))]

        L_p = P_p*C_p'*inv(R_p+C_p*P_p*C_p');
        P_p = (eye(7)-L_p*C_p)*P_p;
        yGPS=[y_gps_n;y_gps_e;y_gps_Vg;y_gps_course;0;0];
        xhat_p = xhat_p+L_p*(yGPS-h_p);
%         % gps East position
%         L_p = ;
%         P_p = ;
%         xhat_p = ;  
%         % gps ground speed
%         h_p = ;
%         C_p = ;
%         L_p = ;
%         P_p = ;
%         xhat_p = ;  
%         % gps course
%         h_p = ;
%         C_p = ;
%         L_p = ;
%         P_p = ;
%         xhat_p = ;  
%         % pseudo measurement #1 y_1 = Va*cos(psi)+wn-Vg*cos(chi)
%         h_p = ;  % pseudo measurement
%         C_p = ;
%         L_p = ;
%         P_p = ;
%         xhat_p = ;
%         % pseudo measurement #2 y_2 = Va*sin(psi) + we - Vg*sin(chi)
%         h_p = ;  % pseudo measurement
%         C_p = ;
%         L_p = ;
%         P_p = ;
%         xhat_p = ;

        % update stored GPS signals
        y_gps_n_old      = y_gps_n;
        y_gps_e_old      = y_gps_e;
        y_gps_Vg_old     = y_gps_Vg;
        y_gps_course_old = y_gps_course;
    end
     
    pnhat    = xhat_p(1);
    pehat    = xhat_p(2);
    Vghat    = xhat_p(3);
    chihat   = xhat_p(4); 
    wnhat    = xhat_p(5);
    wehat    = xhat_p(6);
    psihat   = xhat_p(7);
  
    alphahat = 0;
    betahat = 0;
    
      xhat = [...
        pnhat;...
        pehat;...
        hhat;...
        Vahat;...
        alphahat;...
        betahat;...
        phihat;...
        thetahat;...
        chihat;...
        phat;...
        qhat;...
        rhat;...
        Vghat;...
        wnhat;...
        wehat;...
        psihat;...
        ];
end
