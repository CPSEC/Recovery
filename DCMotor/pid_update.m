% source: https://nicisdigital.wordpress.com/2013/01/11/pid-controller-in-matlab/

function pid_update(curr_error, dt)
    global windupGuard;
    global proportional_gain;
    global integral_gain;
    global derivative_gain;
    global prev_error;
    global int_error;
    global control;
 
    % integration
    int_error = int_error + (curr_error * dt);
 
    % integration windup guarding
    if (int_error < -(windupGuard))
         int_error = -(windupGuard);
    elseif (int_error > windupGuard)
         int_error = windupGuard;
    end
 
    % differentiation
    diff = ((curr_error - prev_error) / dt);
 
    % scaling
    p_term = (proportional_gain * curr_error);
    i_term = (integral_gain     * int_error);
    d_term = (derivative_gain   * diff);
 
    % summation of terms
    control = p_term + i_term + d_term;
 
    % save current error as previous error for next iteration
    prev_error = curr_error;