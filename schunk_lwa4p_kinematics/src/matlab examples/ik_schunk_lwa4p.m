function q=ik_schunk_lwa4p(w)
% IK_SCHUNK_LWA4P Inverse kinematics for the Schunk LWA4P arm.
%  Q = IK_SCHUNK_LWA4P(W,A,D)

%d = [205, 0, 0, 305, 0, 350];
%a = [0, 350, 0, 0, 0, 0];
d = [205, 0, 0, 316, 0, 318.3];
a = [0, 369.2, 0, 0, 0, 0];
d = [205, 0, 0, 365, 0, 318.3];
a = [0, 500, 0, 0, 0, 0];

q0 = [0,pi/2,pi/2,0,0,0];

w1 = w(1);
w2 = w(2);
w3 = w(3);
w4 = w(4);
w5 = w(5);
w6 = w(6);

l1 = d(1);
l2 = a(2);
l3 = d(4);
l4 = d(6);

q = zeros(8,6);
% Solution layout
% [q1-1, q2-1-1, q3-1-1, q4-1-1-1, q5-1-1-1, q6;
%  q1-1, q2-1-1, q3-1-1, q4-1-1-2, q5-1-1-2, q6;
%  q1-1, q2-1-2, q3-1-2, q4-1-2-1, q5-1-2-1, q6;
%  q1-1, q2-1-2, q3-1-2, q4-1-2-2, q5-1-2-2, q6;
%  q1-2, q2-2-1, q3-2-1, q4-2-1-1, q5-2-1-1, q6;
%  q1-2, q2-2-1, q3-2-1, q4-2-1-2, q5-2-1-2, q6;
%  q1-2, q2-2-2, q3-2-2, q4-2-2-1, q5-2-2-1, q6;
%  q1-2, q2-2-2, q3-2-2, q4-2-2-2, q5-2-2-2, q6]

% Wrist roll
q(:,6) = pi*log(sqrt(w4^2+w5^2+w6^2));
r1 = w4/exp(q(1,6)/pi);
r2 = w5/exp(q(1,6)/pi);
r3 = w6/exp(q(1,6)/pi);

% Waist
q(1:4,1) = atan2(w2-l4*r2,w1-l4*r1);
q(5:8,1) = wrapToPi(q(1,1)+pi);

% Auxiliary variables
p1 = w3 -l1 - l4*r3;
p2 = w1*cos(q(:,1))+w2*sin(q(:,1)) - l4*(r1*cos(q(:,1))+r2*sin(q(:,1)));

% Elbow
q(:,3) = asin((p1^2+p2.^2-l2^2-l3^2)/(2*l2*l3));
% For q1-1
q([3:4,7:8],3) = sign(q([3:4,7:8],3))*pi - q([3:4,7:8],3);

% Shoulder
% We are discarding the imaginary part to supress warnings. Infeasible
% solutions will show up with imaginary q5.
q(:,2) = atan2(real(p1*(l2+l3*sin(q(:,3)))+p2.*l3.*cos(q(:,3))),...
               real(p2.*(l2+l3*sin(q(:,3)))-p1*l3*cos(q(:,3))));
           
% Wrist pitch
q(1:2:7,5) = acos((r1*cos(q(1:2:7,1))+r2*sin(q(1:2:7,1))).*sin(q(1:2:7,2)+q(1:2:7,3))...
                    -r3*cos(q(1:2:7,2)+q(1:2:7,3)));
q(2:2:8,5) = -q(1:2:7,5);

% Elbow roll
% !!! Undefined when q5 = 0
% We are discarding the imaginary part to supress warnings. Infeasible
% solutions will show up with imaginary q5.
q(:,4) = atan2(real(sin(q(:,5)).*(-r1*sin(q(:,1))+r2*cos(q(:,1)))),...
               real(sin(q(:,5)).*(-(r1*cos(q(:,1))+r2*sin(q(:,1)))...
               .*cos(q(:,2)+q(:,3))-r3*sin(q(:,2)+(q(:,3))))));

% When q5=0 set q4 to "default" value, which is pi in my kinematic model.
for k = 1:size(q,1)
    if q(k,5) < 2*eps
        q(k,4) = pi;
    end
end
           
% Filter infeasible points.
% Infeasible points contain imaginary joint rotations.
% TODO: Are there any other indicators of infeasibility?
% TODO: Currently, the imaginary cutoff value is arbitrary,
%       is there a better way to pick a cutoff
outliers = abs(imag(q)) > 0.001;
for k = 8:-1:1
    if find(outliers(k,:))
        q(k,:) = [];
    else
        % Compensate inital conditions for feasible points
        q(k,:) = wrapToPi(real(q(k,:))-q0);
    end
end

q = q';

% Filter out points outside of joint limits
disp("Checking")
q = cheq_lwa4p(q);
disp("Done checking")
% Filter false solutions
% These solutions are generated by fusing redundant joint solutions
% that actually don't match yield the original pose
% TODO: Be more careful when fusing redundant solutions to avoid this
% problem altogether
for k = size(q,2):-1:1
    w_k = dk_schunk_lwa4p(q(:,k));
    if (norm(w-w_k,inf) > 1e-6) % TODO: check cutoff
        q(:,k) = [];
    end
end
