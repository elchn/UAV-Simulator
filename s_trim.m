function [x_trim,u_trim]=s_trim (P,h_s,u_s)

Va=u_s;
 %gama different from gamma
% gama=Jx*Jz-Jxz^2; gama1=(Jxz*(Jx-Jy+Jz))/gama; gama2=(Jz*(Jz-Jy)+Jxz^2)/gama;  gama3=(Jz)/gama; gama4=(Jxz)/gama; gama5=(Jz-Jx)/Jy; 
% gama6=(Jxz)/Jy; gama7=((Jx-Jy)*Jx+Jxz^2)/gama; gama8=(Jx)/gama;
%     

% C_p_0=gama3*P.C_ell_0+gama4*P.C_n_0;  % it si better to use C_p_*   :lowercase p
% C_p_beta=gama3*P.C_ell_beta+gama4*P.C_n_beta;
% C_p_p=gama3*P.C_ell_p+gama4*P.C_n_p;
% C_p_r=gama3*P.C_ell_r+gama4*P.C_n_r;
% C_p_delta_a=gama3*P.C_ell_delta_r+gama4*P.C_n_delta_r;
% C_p_delta_r=gama3*P.C_ell_delta_r+gama4*P.C_n_delta_r;
% C_r_0=gama4*P.C_ell_0+gama8*P.C_n_0;
% C_r_beta=gama4*P.C_ell_beta+gama8*P.C_n_beta;
% C_r_p=gama4*P.C_ell_p+gama8*P.C_n_p;
% C_r_r=gama4*P.C_ell_r+gama8*P.C_n_r;
% C_r_delta_a=gama4*P.C_ell_delta_a+gama8*P.C_n_delta_a;
% C_r_delta_r=gama4*P.C_ell_delta_r+gama8*P.C_n_delta_r;

%computing airodynamic coefficients that are function of alpha and beta

alpha_s=0;

sigma=(1+exp(-P.M*(alpha_s-P.alpha0))+exp(P.M*(alpha_s+P.alpha0)))/((1+exp(-P.M*(alpha_s-P.alpha0)))*(1+exp(P.M*(alpha_s+P.alpha0))));% eq 4.10
CL=(1-sigma)*(P.C_L_0+P.C_L_alpha*alpha_s)+sigma*(2*sign(alpha_s)*sin(alpha_s)^2*cos(alpha_s)); % eq 4.9
 % eq 4.11
C_D_0=P.C_D_0; C_D_P=C_D_0;AR=P.b^2/P.S_wing; 
CD=C_D_P+(P.C_L_0+P.C_L_alpha*alpha_s)^2/(pi*P.e*AR);  % OK
    
CX=-CD*cos(alpha_s)+CL*sin(alpha_s);
CXQ=-P.C_D_q*cos(alpha_s)+P.C_L_q*sin(alpha_s);
CXDeltaE=-P.C_D_delta_e*cos(alpha_s)+P.C_L_delta_e*sin(alpha_s);
CZ=-CD*sin(alpha_s)-CL*cos(alpha_s);
CZQ=-P.C_D_q*sin(alpha_s)-P.C_L_q*cos(alpha_s);
CZDeltaE=-P.C_D_delta_e*sin(alpha_s)-P.C_L_delta_e*cos(alpha_s);


delta_e_s=(-2*P.mass*P.gravity/(P.rho*Va^2*P.S_wing)-CZ)/CZDeltaE;
delta_t_s=sqrt(Va^2*P.S_wing/(P.S_prop*P.C_prop)*(CX+CXDeltaE*delta_e_s)+Va^2)/P.k_motor;
x_trim=[0;0;h_s;u_s;0;0;0;0;0;0;0;0];
u_trim=[delta_e_s;0;0;delta_t_s];

end