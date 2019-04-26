% forces_moments.m
%   Computes the foreces and moments acting on the airframe. 
%
%   Output is
%       F     - forces
%       M     - moments
%       Va    - airspeed
%       alpha - angle of attack
%       beta  - sideslip angle
%       wind  - wind vector in the inertial frame
%
%  Revised:
%   2/2/2010 - RB 
%   5/14/2010 - RB
%   Completed:
%   3/14/2014 -
%   Abolfazl Shahrooei

function out = forces_moments(x, delta, wind, P)

    % relabel the inputs
    pn      = x(1);
    pe      = x(2);
    pd      = x(3);
    u       = x(4);
    v       = x(5);
    w       = x(6);
    phi     = x(7);
    theta   = x(8);
    psi     = x(9);
    p       = x(10);
    q       = x(11);
    r       = x(12);
    delta_e = delta(1);
    delta_a = delta(2);
    delta_r = delta(3);
    delta_t = delta(4);
    w_ns    = wind(1); % steady wind - North
    w_es    = wind(2); % steady wind - East
    w_ds    = wind(3); % steady wind - Down
    u_wg    = wind(4); % gust along body x-axis
    v_wg    = wind(5); % gust along body y-axis    
    w_wg    = wind(6); % gust along body z-axis
    
    
    %-------------------------------------------------------------------------------------------------------------------------------------------------
    %wid in body frame
    u_w =(cos(theta)*cos(psi))*w_ns+(cos(theta)*sin(psi))*w_es+(-sin(theta))*w_ds+u_wg ;
    v_w =(sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))*w_ns+(sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))*w_es+(sin(phi)*cos(theta))*w_ds+v_wg;
    w_w =(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))*w_ns+(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*w_es+(cos(phi)*cos(theta))*w_ds+w_wg;
    
    u_r=u-u_w;
    v_r=v-v_w;
    w_r=w-w_w;
    
    % compute wind vector in the inertial frame
    w_iner =[  cos(theta)*cos(psi)                              cos(theta)*sin(psi)                              -sin(theta);...
               sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)   sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)   sin(phi)*cos(theta);...
               cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)   cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)   cos(phi)*cos(theta)]'*[u_w;v_w;w_w];
   
           w_n=w_iner(1);    
           w_e = w_iner(2);    
           w_d =w_iner(3);
    
    
    % compute airspeed Va, angle-of-attack alpha, side-slip beta
    Va    = sqrt(u_r^2+v_r^2+w_r^2);
    alpha = atan((w_r)/(u_r));
    beta  = asin((v_r)/Va);
    %--------------------------------------------------------------------------------------------------------------------------------------------------
%     % compute wind vector in the inertial frame
%     w_n =(cos(theta)*cos(psi))*w_ns+(cos(theta)*sin(psi))*w_es+(-sin(theta))*w_ds+u_wg ;
%     w_e =(sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))*w_ns+(sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))*w_es+(sin(phi)*cos(theta))*w_ds+v_wg;
%     w_d =(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))*w_ns+(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*w_es+(cos(phi)*cos(theta))*w_ds+w_wg;
%     
%     
%     % compute airspeed Va, angle-of-attack alpha, side-slip beta
%     Va    = sqrt((u-w_n)^2+(v-w_e)^2+(w-w_d)^2);
%     alpha = atan((w-w_d)/(u-w_n));
%     beta  = atan((v-w_e)/Va);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    sigma_f_alpha=(1+exp(-P.M*(alpha-P.alpha0))+exp(P.M*(alpha+P.alpha0)))/((1+exp(-P.M*(alpha-P.alpha0)))*(1+exp(P.M*(alpha+P.alpha0))));% eq 4.10
    CL=(1-sigma_f_alpha)*(P.C_L_0+P.C_L_alpha*alpha)+sigma_f_alpha*(2*sign(alpha)*sin(alpha)^2*cos(alpha)); % eq 4.9
     % eq 4.11
    C_D_0=P.C_D_0; C_D_P=C_D_0;AR=P.b^2/P.S_wing; 
    CD=C_D_P+(P.C_L_0+P.C_L_alpha*alpha)^2/(pi*P.e*AR);  % OK
    
    CX=-CD*cos(alpha)+CL*sin(alpha);
    CXQ=-P.C_D_q*cos(alpha)+P.C_L_q*sin(alpha);
    CXDeltaE=-P.C_D_delta_e*cos(alpha)+P.C_L_delta_e*sin(alpha);
    CZ=-CD*sin(alpha)-CL*cos(alpha);
    CZQ=-P.C_D_q*sin(alpha)-P.C_L_q*cos(alpha);
    CZDeltaE=-P.C_D_delta_e*sin(alpha)-P.C_L_delta_e*cos(alpha);
    
    g=P.gravity;
    ff1=-P.mass*g*sin(theta)+0.5*P.rho*Va^2*P.S_wing*(CX+CXQ*(P.c/(2*Va))*q+CXDeltaE*delta_e)+0.5*P.rho*P.S_prop*P.C_prop*((P.k_motor*delta_t)^2-Va^2);
    ff2=P.mass*g*cos(theta)*sin(phi)+0.5*P.rho*Va^2*P.S_wing*(P.C_Y_0+P.C_Y_beta*beta+P.C_Y_p*(P.b/(2*Va))*p+P.C_Y_r*(P.b/(2*Va))*r+P.C_Y_delta_a*delta_a+P.C_Y_delta_r*delta_r);
    ff3=P.mass*g*cos(theta)*cos(phi)+0.5*P.rho*Va^2*P.S_wing*(CZ+CZQ*(P.c/(2*Va))*q+CZDeltaE*delta_e);
%     ff=[-P.mass*g*sin(theta); P.mass*g*cos(theta)*sin(phi);P.mass*g*cos(theta)*cos(phi)]+...
%         0.5*P.rho*Va^2*P.S_wing*[CX+CXQ*(P.c/(2*Va))*q+CXDeltaE*delta_e;...
%         P.C_Y_0+P.C_Y_beta*beta+P.C_Y_p*(P.b/(2*Va))*p+P.C_Y_r*(P.b/(2*Va))*r+P.C_Y_delta_a*delta_a+P.C_Y_delta_r*delta_r;...
%         CZ+CZQ*(P.c/(2*Va))*q+CZDeltaE*delta_e]+...
%         0.5*P.rho*P.S_prop*P.C_prop*[(P.k_motor*delta_t)^2-Va^2;0;0];
    Force = [ff1;ff2;ff3];
    tt1=0.5*P.rho*Va^2*P.S_wing*(P.b*(P.C_ell_0+P.C_ell_beta*beta+P.C_ell_p*(P.b/(2*Va))*p+P.C_ell_r*(P.b/(2*Va))*r+P.C_ell_delta_a*delta_a+P.C_ell_delta_r*delta_r));
    tt2=0;%0.5*P.rho*Va^2*P.S_wing*(P.c*(P.C_M_0+P.C_M_alpha*alpha+P.C_M_q*(P.c/(2*Va))*q)+P.C_M_delta_e*delta_e);
    tt3=0.5*P.rho*Va^2*P.S_wing*(P.b*(P.C_n_0+P.C_n_beta*beta+P.C_n_p*(P.b/(2*Va))*p+P.C_n_r*(P.b/(2*Va))*r+P.C_n_delta_a*delta_a+P.C_n_delta_r*delta_r));
%     Torque = 0.5*P.rho*Va^2*P.S_wing*[P.b*(P.C_ell_0+P.C_ell_beta*beta+P.C_ell_p*(P.b/(2*Va))*p+P.C_ell_r*(P.b/(2*Va))*r+P.C_ell_delta_a*delta_a+P.C_ell_delta_r*delta_r);...
%         P.c*(P.C_M_0+P.C_M_alpha*alpha+P.C_M_q*(P.c/(2*Va))*q)+P.C_M_delta_e*delta_e;...
%         P.b*(P.C_n_0+P.C_n_beta*beta+P.C_n_p*(P.b/(2*Va))*p+P.C_n_r*(P.b/(2*Va))*r+P.C_n_delta_a*delta_a+P.C_n_delta_r*delta_r)]+[0;0;0];
       Torque=[tt1;tt2;tt3];
    
    out = [Force; Torque; Va; alpha; beta; w_n; w_e; w_d];

end



