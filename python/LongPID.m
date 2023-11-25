function [throttle] = LongPID(k_p,error,k_d,de,k_i,ie)
    throttle =  (k_p * error) + (k_d * de) + (k_i * ie);
end