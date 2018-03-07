function q1 = ik_lwa4p_closest(w1,q0)
% IK_LWA4P_CLOSEST Returns IK solution closest to initial configuration.
%  Q1 = IK_LWA4P_CLOSEST(W1,Q0) Computes Q1 as inverse kinematics solution
%  of W1. If multiple solutions are possible, returns the one that is
%  closest to Q0 in the L_inf norm.

%d = [205, 0, 0, 305, 0, 350];
%a = [0, 350, 0, 0, 0, 0];
d = [205, 0, 0, 316, 0, 318.3];
a = [0, 369.2, 0, 0, 0, 0];
d = [205, 0, 0, 365, 0, 318.3];
a = [0, 500, 0, 0, 0, 0];
% Compute inverse kinematics
q1 = ik_schunk_lwa4p(w1);
% Handle singularity in Schunk configuration 
% When q(5) = 0 the q(4) rotation is arbitrary; We will set it to q0(4)
% When doing trajectory planning, this will prevent sudden jumps in q(4)
for k = size(q1,2):-1:1
    if q1(5,k) < 2*eps % This cutoff is somewhat arbitrary, check if ok!
        q1(4,k) = q0(4);
    end
end

% Compute the closest q1
k_min = size(q1,2);
if k_min > 1
    d_min = norm(wrapToPi(q1(:,end)-q0),inf);
    for k = k_min-1:-1:1
        d = norm(wrapToPi(q1(:,k)-q0),inf);
        if d < d_min
            k_min = k;
            d_min = d;
        end
    end
    q1 = q1(:,k_min);
end
