function y = autopilot(uu,P)
%
% autopilot for mavsim
% 
% Modification History:
%   2/11/2010 - RWB
%   5/14/2010 - RWB
%   Completed By Abolfazl Shahrooei
%   4/21/2014

    % process inputs
    NN = 0;
%    pn       = uu(1+NN);  % inertial North position
%    pe       = uu(2+NN);  % inertial East position
    h        = uu(3+NN);  % altitude
    Va       = uu(4+NN);  % airspeed
%    alpha    = uu(5+NN);  % angle of attack
%    beta     = uu(6+NN);  % side slip angle
    phi      = uu(7+NN);  % roll angle
    theta    = uu(8+NN);  % pitch angle
    chi      = uu(9+NN);  % course angle
    p        = uu(10+NN); % body frame roll rate
    q        = uu(11+NN); % body frame pitch rate
    r        = uu(12+NN); % body frame yaw rate
%    Vg       = uu(13+NN); % ground speed
%    wn       = uu(14+NN); % wind North
%    we       = uu(15+NN); % wind East
%    psi      = uu(16+NN); % heading
    NN = NN+16;
    Va_c     = uu(1+NN);  % commanded airspeed (m/s)
    h_c      = uu(2+NN);  % commanded altitude (m)
    chi_c    = uu(3+NN);  % commanded course (rad)
    NN = NN+3;
    t        = uu(1+NN);   % time
    

    %----------------------------------------------------------
    % lateral autopilot
    
    
    if t==0
        % assume no rudder, therefore set delta_r=0
        delta_r = 0;%coordinated_turn_hold(beta, 1, P);
        % use commanded roll angle to regulate heading
        phi_c   = course_hold(chi_c, chi, r, 1, P);
        % use aileron to regulate roll angle
        delta_a = roll_hold(phi_c, phi, p, 1, P);     

    else
        delta_r = 0;%coordinated_turn_hold(beta, 0, P);
        phi_c   = course_hold(chi_c, chi, r, 0, P);
        delta_a = roll_hold(phi_c, phi, p, 0, P);
    end
  
    
    %----------------------------------------------------------
    % longitudinal autopilot
    
    % define persistent variable for state of altitude state machine
    persistent altitude_state;
    persistent initialize_integrator;
    % initialize persistent variable
    if t==0,
        if h<=P.altitude_take_off_zone,     
            altitude_state = 1;
        elseif h<=h_c-P.altitude_hold_zone, 
            altitude_state = 2;
        elseif h>=h_c+P.altitude_hold_zone, 
            altitude_state = 3;
        else
            altitude_state = 4;
        end
        initialize_integrator = 1;
    end
    
    % implement state machine
    switch altitude_state,
        case 1,  % in take-off zone
            delta_t = pi/4;   %??
            theta_c = pi/20;   %??
            
        case 2,  % climb zone
            delta_t = pi/4;   %??
            theta_c = airspeed_with_pitch_hold(Va_c, Va, 0, P);
            
        case 3, % descend zone
            delta_t = 0;
            theta_c = airspeed_with_pitch_hold(Va_c, Va, 0, P);
            
        case 4, % altitude hold zone
            delta_t =  airspeed_with_throttle_hold(Va_c, Va, 0, P);
            theta_c = altitude_hold(h_c, h, 0, P);
            
    end
    
    if t==0,
        delta_e = pitch_hold(theta_c, theta, q, 1, P);
    else
        delta_e = pitch_hold(theta_c, theta, q, 0, P);
    end

        
    
    
    %----------------------------------------------------------
    % create outputs
    
    % control outputs
    delta = [delta_e; delta_a; delta_r; delta_t];
    % commanded (desired) states
    x_command = [...
        0;...                    % pn
        0;...                    % pe
        h_c;...                  % h
        Va_c;...                 % Va
        0;...                    % alpha
        0;...                    % beta
        phi_c;...                % phi
        theta_c*P.K_theta_DC;... % theta
        chi_c;...                % chi
        0;...                    % p
        0;...                    % q
        0;...                    % r
        ];
            
    y = [delta; x_command];
 
end



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Autopilot functions
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% course_hold
%   - regulate heading using the roll command
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function phi_c = course_hold(chi_c, chi, r, flag, P)
  persistent integrator;
  persistent differentiator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      differentiator = 0;
      error_d1   = 0; 
  end
 
  % compute the current error
  error = chi_c - chi;
  
  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
  % update the differentiator
  differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator...
      + (2/(2*P.tau+P.Ts))*(error - error_d1);
  
  % proportional term
  up = P.course_kp * error;
  
  % integral term
  ui = P.course_ki * integrator;
  
  % derivative term
  ud = P.course_kd * differentiator;
  
  
  % implement PID control
  phi_c = sat(up + ui + ud, 25*pi/180, -25*pi/180);
  
  % implement integrator anti-wind-up
  if P.course_ki~=0,
      phi_c_unsat = up + ui + ud;
      integrator = integrator + P.Ts/P.course_ki * (phi_c - phi_c_unsat);
  end

  % update persistent variables
  error_d1 = error;
% phi_c = 0;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% roll_hold
%   - regulate roll using aileron
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_a = roll_hold(phi_c, phi, p, flag, P)
  persistent integrator;
  persistent differentiator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      differentiator = 0;
      error_d1   = 0; 
  end
 
  % compute the current error
  error = phi_c - phi;
  
  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
%   % update the differentiator
%   differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator...
%       + (2/(2*P.tau+P.Ts))*(error - error_d1);
  
  % proportional term
  up = P.roll_kp * error;
  
  % integral term
  ui = P.roll_ki * integrator;
  
  % derivative term
%   ud = P.roll_kd * differentiator;
    ud = -P.roll_kd * p;
  
  % implement PID control
  delta_a = sat(up + ui + ud, 45*pi/180, -45*pi/180);
  
  % implement integrator anti-wind-up
  if P.roll_ki~=0,
      delta_a_unsat = up + ui + ud;
      integrator = integrator + P.Ts/P.roll_ki * (delta_a - delta_a_unsat);
  end

  % update persistent variables
  error_d1 = error;
% delta_a = 0;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% pitch_hold
%   - regulate pitch using elevator
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_e = pitch_hold(theta_c, theta, q, flag, P)
  persistent integrator;
  persistent differentiator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      differentiator = 0;
      error_d1   = 0; 
  end
 
  % compute the current error
  error = theta_c - theta;
  
  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
  % update the differentiator
  differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator...
      + (2/(2*P.tau+P.Ts))*(error - error_d1);
  
  % proportional term
  up = P.pitch_kp * error;
  
  % integral term
  ui = P.pitch_ki * integrator;
  
  % derivative term
  ud = P.pitch_kd * differentiator;
  
  
  % implement PID control
  delta_e = sat(up + ui + ud, 45*pi/180, -45*pi/180);
  
  % implement integrator anti-wind-up
  if P.pitch_ki~=0,
      delta_e_c_unsat = up + ui + ud;
      integrator = integrator + P.Ts/P.pitch_ki * (delta_e_c - delta_e_c_unsat);
  end

  % update persistent variables
  error_d1 = error;
% delta_e = 0;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% airspeed_with_pitch_hold
%   - regulate airspeed using pitch angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_c = airspeed_with_pitch_hold(Va_c, Va, flag, P)
  persistent integrator;
  persistent differentiator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      differentiator = 0;
      error_d1   = 0; 
  end
 
  % compute the current error
  error = Va_c - Va;
  
  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
  % update the differentiator
  differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator...
      + (2/(2*P.tau+P.Ts))*(error - error_d1);
  
  % proportional term
  up = P.airspeed_pitch_kp * error;
  
  % integral term
  ui = P.airspeed_pitch_ki * integrator;
  
  % derivative term
  ud = P.airspeed_pitch_kd * differentiator;
  
  
  % implement PID control
  theta_c = sat(up + ui + ud, 30*pi/180, -30*pi/180);
  
  % implement integrator anti-wind-up
  if P.airspeed_pitch_ki~=0,
      theta_c_unsat = up + ui + ud;
      integrator = integrator + P.Ts/P.airspeed_pitch_ki * (theta_c - theta_c_unsat);
  end

  % update persistent variables
  error_d1 = error;

end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% airspeed_with_throttle_hold
%   - regulate airspeed using throttle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_t = airspeed_with_throttle_hold(Va_c, Va, flag, P)
  persistent integrator;
  persistent differentiator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      differentiator = 0;
      error_d1   = 0; 
  end
 
  % compute the current error
  error = Va_c - Va;
  
  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
  % update the differentiator
  differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator...
      + (2/(2*P.tau+P.Ts))*(error - error_d1);
  
  % proportional term
  up = P.airspeed_throttle_kp * error;
  
  % integral term
  ui = P.airspeed_throttle_ki * integrator;
  
  % derivative term
  ud = P.airspeed_throttle_kd * differentiator;
  
  
  % implement PID control
  delta_t = sat(up + ui + ud, 80*pi/180, -80*pi/180);
  
  % implement integrator anti-wind-up
  if P.airspeed_throttle_ki~=0,
      delta_t_c_unsat = up + ui + ud;
      integrator = integrator + P.Ts/P.airspeed_throttle_ki * (delta_t_c - delta_t_c_unsat);
  end

  % update persistent variables
  error_d1 = error;
% delta_t = 0;
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% altitude_hold
%   - regulate altitude using pitch angle
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function theta_c = altitude_hold(h_c, h, flag, P)
  persistent integrator;
  persistent differentiator;
  persistent error_d1;
  % initialize persistent variables at beginning of simulation
  if flag==1,
      integrator = 0; 
      differentiator = 0;
      error_d1   = 0; 
  end
 
  % compute the current error
  error = h_c - h;
  
  % update the integrator
  integrator = integrator + (P.Ts/2)*(error + error_d1); % trapazoidal rule
  
  % update the differentiator
  differentiator = (2*P.tau-P.Ts)/(2*P.tau+P.Ts)*differentiator...
      + (2/(2*P.tau+P.Ts))*(error - error_d1);
  
  % proportional term
  up = P.altitude_kp * error;
  
  % integral term
  ui = P.altitude_ki * integrator;
  
  % derivative term
  ud = P.altitude_kd * differentiator;
  
  
  % implement PID control
  theta_c = sat(up + ui + ud, 10*pi/180, -10*pi/180);
  
  % implement integrator anti-wind-up
  if P.altitude_ki~=0,
      theta_c_unsat = up + ui + ud;
      integrator = integrator + P.Ts/P.altitude_ki * (theta_c - theta_c_unsat);
  end

  % update persistent variables
  error_d1 = error;
% theta_c = 0;
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% coordinated_turn_hold
%   - sideslip with rudder
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function delta_r = coordinated_turn_hold(v, flag, P)
  delta_r = 0;
end

  
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% sat
%   - saturation function
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = sat(in, up_limit, low_limit)
  if in > up_limit,
      out = up_limit;
  elseif in < low_limit;
      out = low_limit;
  else
      out = in;
  end
end
  
 