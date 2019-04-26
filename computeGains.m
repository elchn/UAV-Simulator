% select gains for roll loop
    % get transfer function data for delta_a to phi
    [num,den]=tfdata(T_phi_delta_a,'v');
    a_phi2 = num(3);
    a_phi1 = den(2);
    % maximum possible aileron command
    delta_a_max = 45*pi/180;
    % Roll command when delta_a_max is achieved
    phi_max = 15*pi/180;
    % pick natural frequency to achieve delta_a_max for step of phi_max
    zeta_roll = 0.707;
    wn_roll = sqrt(a_phi2*delta_a_max*sqrt(1-zeta_roll^2)/phi_max);
    
    % set control gains based on zeta and wn
    P.roll_kp = delta_a_max/phi_max;
    P.roll_kd = (2*zeta_roll*wn_roll-a_phi1)/a_phi2;
    P.roll_ki = 0.05;    % by trial and error, it should be small, the correct way to design this gain is using root locua
    
   
    
% select gains for course loop
   wn_course=wn_roll/10;
   zeta_course=1.907;
   % set control gains based on zeta and wn
   P.course_kp = 2*zeta_course*wn_course*P.Vat/P.gravity;
   P.course_ki = wn_course^2*P.Vat/P.gravity;
   P.course_kd = 0;
   
% select gains for sideslip hold
    P.beta_kp = 0;
    P.beta_ki = 0;
    P.beta_kd = 0;

   
% select gains for the pitch loop
   [numPAH,denPAH]=tfdata(T_theta_delta_e,'v');
   a_theta3=numPAH(3);
   a_theta2=denPAH(3);
   a_theta1=denPAH(2);
       % maximum possible elavator command
    delta_e_max = 45*pi/180;
    % Pitch command when delta_e_max is achieved
    theta_max = 10*pi/180;
   
   P.pitch_kp = delta_e_max/theta_max*sign(a_theta3);
        wn_theta=sqrt(a_theta2+delta_e_max/theta_max*abs(a_theta3));
        zeta_theta=0.707;
   P.pitch_kd = (2*zeta_theta*wn_theta-a_theta1)/a_theta3;
   P.pitch_ki = 0.0;
   P.K_theta_DC = P.pitch_kp*a_theta3/(a_theta2+P.pitch_kp*a_theta3);

% select gains for altitude loop
   wn_h=wn_theta/10;
   zeta_h=2.707;

   P.altitude_kp = 2*zeta_h*wn_h/(P.K_theta_DC*P.Vat);
   P.altitude_ki = wn_h^2/(P.K_theta_DC*P.Vat);
   P.altitude_kd = 0;
 
% airspeed hold using pitch
   wn_v2=wn_theta/7;
   zeta_v2=0.507;
   [numAHP,denAHP]=tfdata(T_Va_theta,'v');
   a_V1=denAHP(2);
   
   P.airspeed_pitch_kp = -(wn_v2^2)/(P.K_theta_DC*P.gravity);
   P.airspeed_pitch_ki = (a_V1-2*zeta_v2*wn_v2)/(P.K_theta_DC*P.gravity);
   P.airspeed_pitch_kd = 0;
 
% airspeed hold using throttle
   wn_v=wn_v2*4;
   zeta_v=4.707;
   [numAHT,denAHT]=tfdata(T_Va_delta_t,'v');
   a_V2=numAHT(2);
   
   P.airspeed_throttle_kp = (2*zeta_v*wn_v-a_V1)/(a_V2);
   P.airspeed_throttle_ki = wn_v^2/a_V2;
   P.airspeed_throttle_kd = 0;
 


