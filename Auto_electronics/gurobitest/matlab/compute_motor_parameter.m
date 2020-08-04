% This function is used to compute DC motor parameters 
function [kt, ke, r, stall_torq, free_vel] = compute_motor_parameter(stall_i, stall_torq, free_vel, free_i, v)

% pre-processing
stall_torq = 0.0980665*stall_torq;
free_vel = 2*pi*free_vel/60;

kt = stall_torq./stall_i;
r = v./stall_i;
ke = (-r.*free_i+v)./free_vel; 
end