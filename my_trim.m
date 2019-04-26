function [x_trim,u_trim]=my_trim(P,Va,gamma,R)
%This function computes input and states values in trim point specified by:
%     Va is the desired airspeed (m/s)
%     gamma is the desired flight path angle (radians)
%     R is the desired radius (m)--- use:  (+) for right handed orbit, 
%                                          (-) for left handed orbit
%
%(C) Abolfazl Shahrooei 4/7/2014

%Coeficient and Parameters=================================================

%==========================================================================

[alpha_s,beta_s,phi_s]=argmin(P,Va,gaamma,R,alpha00,beta0,phi0);
x_trim=compute_states(P,Va,gaamma,R,alpha_s,beta_s,phi_s);
u_trim=compute_inputs(P,Va,gaamma,R,alpha_s,beta_s,phi_s,x_trim);

end



 function[alpha_s,beta_s,phi_s]= argmin(P,Va,gaamma,R,alpha00,beta0,phi0)
 

 end
 function trim_states=compute_stats(P,Va,gaamma,R,alpha_s,beta_s,phi_s)
 
 
 end
 function trim_inputs=compute_inputs(P,Va,gaamma,R,alpha_s,beta_s,phi_s,trim_stats)
 end