function F_thrust = Engine_throttle_model(Fmax, throttle, min_throttle)
    if throttle < min_throttle
        F_thrust = 0.0;    
    else
        F_thrust = Fmax * throttle;
    end 
end