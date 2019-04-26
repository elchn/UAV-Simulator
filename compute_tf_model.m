function [T_phi_delta_a,T_chi_phi,T_theta_delta_e,T_h_theta,T_h_Va,T_Va_delta_t,T_Va_theta,T_v_delta_r]...
    = compute_tf_model(x_trim,u_trim,P)
% x_trim is the trimmed state,
% u_trim is the trimmed input

% other airodinamic coefficeint(it is recommended to use these in forces_momments!!!??? it is not OK because we separated forces and dynamics)
%GAMA
    gama=P.Jx*P.Jz-P.Jxz^2;   gama3=(P.Jz)/gama; gama4=(P.Jxz)/gama; 
    
C_p_p=gama3*P.C_ell_p+gama4*P.C_n_p;
C_p_delta_a=gama3*P.C_ell_delta_r+gama4*P.C_n_delta_r;


Va_trim=P.Va0;
theta_trim=x_trim(8);
alpha_trim=0;
delta_e_trim=u_trim(1);
delta_t_trim=u_trim(4);
% add stuff here
a_phi1=-0.5*P.rho*Va_trim^2*P.S_wing*P.b*C_p_p*P.b/(2*Va_trim);
a_phi2=0.5*P.rho*Va_trim^2*P.S_wing*P.b*C_p_delta_a;
a_theta1=-P.rho*Va_trim^2*P.c*P.S_wing/(2*P.Jy)*P.C_M_q*P.c/(2*Va_trim);
a_theta2=-P.rho*Va_trim^2*P.c*P.S_wing/(2*P.Jy)*P.C_M_alpha;
a_theta3=P.rho*Va_trim^2*P.c*P.S_wing/(2*P.Jy)*P.C_M_delta_e;
a_V1=P.rho*Va_trim*P.S_wing/P.mass*(P.C_D_0+P.C_D_alpha*alpha_trim+P.C_D_delta_e*delta_e_trim)+P.rho*P.S_prop/P.mass*P.C_prop*Va_trim;
a_V2=P.rho*P.S_prop/P.mass*P.C_prop*P.k_motor^2*delta_t_trim;
a_V3=P.gravity*cos(theta_trim-alpha_trim);
a_beta1=-P.rho*Va_trim*P.S_wing/(2*P.mass)*P.C_Y_beta;
a_beta2=P.rho*Va_trim*P.S_wing/(2*P.mass)*P.C_Y_delta_r;

    
% define transfer functions
T_phi_delta_a   = tf([a_phi2],[1,a_phi1,0]);
T_chi_phi       = tf([P.gravity/Va_trim],[1,0]);
T_theta_delta_e = tf(a_theta3,[1,a_theta1,a_theta2]);
T_h_theta       = tf([Va_trim],[1,0]);
T_h_Va          = tf([theta_trim],[1,0]);
T_Va_delta_t    = tf([a_V2],[1,a_V1]);
T_Va_theta      = tf([-a_V3],[1,a_V1]);
T_v_delta_r     = tf([Va_trim*a_beta2],[1,a_beta1]);

