P.gravity = 9.8;
   
%physical parameters of airframe
P.mass = 1.56;
P.Jx   = 0.1147;
P.Jy   = 0.0576;
P.Jz   = 0.1712;
P.Jxz  = 0.0015;

% aerodynamic coefficients
P.M             = 50;
P.epsilon       = 0.1592;
P.alpha0        = 0.4712;
P.rho           = 1.2682;
P.c             = 0.3302;
P.b             = 1.4224;
P.S_wing        = 0.2589;
P.S_prop        = 0.0314;
P.k_motor       = 20;
P.e             =0.9; % Oswald Efficiency Factor
P.C_L_0         = 0.28;
P.C_L_alpha     = 3.45;
P.C_L_q         = 0.0;
P.C_L_delta_e   = -0.36;
P.C_D_0         = 0.03;
P.C_D_alpha     = 0.2108;
P.C_D_q         = 0.0;
P.C_D_delta_e   = 0.0;
P.C_M_0         = 0.0;
P.C_M_alpha     = -0.38;
P.C_M_q         = -3.6;
P.C_M_delta_e   = -0.5;
P.C_Y_0         = 0.0;
P.C_Y_beta      = -0.98;
P.C_Y_p         = -0.26;
P.C_Y_r         = 0.0;
P.C_Y_delta_a   = 0.0;
P.C_Y_delta_r   = -0.17;
P.C_ell_0       = 0.0;
P.C_ell_beta    = -0.12;
P.C_ell_p       = -0.26;
P.C_ell_r       = 0.14;
P.C_ell_delta_a = 0.08;
P.C_ell_delta_r = 0.105;
P.C_n_0         = 0.0;
P.C_n_beta      = 0.25;
P.C_n_p         = 0.022;
P.C_n_r         = -0.35;
P.C_n_delta_a   = 0.06;
P.C_n_delta_r   = -0.032;
P.C_prop        = 1;


% wind parameters
P.wind_n = 0;%3;
P.wind_e = 0;%2;
P.wind_d = 0;
P.L_wx = 1250;
P.L_wy = 1750;
P.L_wz = 1750;
P.sigma_wx = 1; 
P.sigma_wy = 1;
P.sigma_wz = 1;
P.Va0 = 10;

% autopilot sample rate
P.Ts = 0.01;

% compute trim conditions using 'mavsim_chap5_trim.mdl'
P.Vat    = 10; Vas=P.Vat; % desired airspeed (m/s)
gammas = 0*pi/180;  % desired flight path angle (radians)
Rs     = 0;         % desired radius (m) - use (+) for right handed orbit, 
                    %                          (-) for left handed orbit
% first cut at initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = P.Vat; % initial velocity along body x-axis    ????????!!!!!!!!!
P.v0     = 0;  % initial velocity along body y-axis
P.w0     = 0;  % initial velocity along body z-axis
P.phi0   = 0;  % initial roll angle
P.theta0 = 0;  % initial pitch angle
P.psi0   = 0;  % initial yaw angle
P.p0     = 0;  % initial body frame roll rate
P.q0     = 0;  % initial body frame pitch rate
P.r0     = 0;  % initial body frame yaw rate

% % run trim commands
% [x_trim, u_trim]=compute_trim('mavsim_trim',P.Va,gamma,R);
% P.u_trim = u_trim;
% P.x_trim = x_trim;

%Run my trim script
%==========================================================================
h_s=100;
u_s=P.Va0;

[x_trim, u_trim]=s_trim(P,h_s,u_s);
P.u_trim = u_trim;
P.x_trim = x_trim;
% set initial conditions to trim conditions
% initial conditions
P.pn0    = 0;  % initial North position
P.pe0    = 0;  % initial East position
P.pd0    = 0;  % initial Down position (negative altitude)
P.u0     = x_trim(4);  % initial velocity along body x-axis
P.v0     = x_trim(5);  % initial velocity along body y-axis
P.w0     = x_trim(6);  % initial velocity along body z-axis
P.phi0   = x_trim(7);  % initial roll angle
P.theta0 = x_trim(8);  % initial pitch angle
P.psi0   = x_trim(9);  % initial yaw angle
P.p0     = x_trim(10);  % initial body frame roll rate
P.q0     = x_trim(11);  % initial body frame pitch rate
P.r0     = x_trim(12);  % initial body frame yaw rate

% compute different transfer functions
[T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P);

% % linearize the equations of motion around trim conditions
% [A_lon, B_lon, A_lat, B_lat] = compute_ss_model('mavsim_trim',x_trim,u_trim);

% gain on dirty derivative
P.tau = 5;

% autopilot gains
% altitude parameters and gains
P.altitude_take_off_zone = 10;
P.altitude_hold_zone = 10;
computeGains;

% sensor parameters
P.sigma_gyro = 0.0396; % standard deviation of gyros in rad/sec
P.sigma_accel = 0.00094; % standard deviation of accelerometers in g
P.sigma_static_pres = 0.3320; % standard deviation of static pressure sensor in Pascals
P.sigma_diff_pres = 2.3;  % standard deviation of diff pressure sensor in Pascals

% GPS parameters
P.Ts_gps = 1; % sample rate of GPS in s
P.beta_gps = 1/16000; % 1/s
P.sigma_n_gps = 2.1;
P.sigma_e_gps = 2.1; 
P.sigma_h_gps = 4.0;
P.sigma_Vg_gps = 1;
P.sigma_course_gps = 5*pi/180;


