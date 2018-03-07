function w=dk_schunk_lwa4p(q)
% DK_SCHUNK_LWA4P Direct kinematics for the Schunk LWA4P arm.
%  W = DK_SCHUNK_LWA4P(Q,A,D)
% 
% TODO: A and D should be removed as parameters, because this function is
%       specific for a given manipulator.

%d = [205, 0, 0, 305, 0, 350];
%a = [0, 350, 0, 0, 0, 0];
d = [205, 0, 0, 316, 0, 318.3];
a = [0, 369.2, 0, 0, 0, 0];
d = [205, 0, 0, 365, 0, 318.3];
a = [0, 500, 0, 0, 0, 0];


q = cheq_lwa4p(q);

if ~isempty(q)

    q0 = [0,pi/2,pi/2,0,0,0]';

    q1 = wrapToPi(q(1) + q0(1));
    q2 = wrapToPi(q(2) + q0(2));
    q3 = wrapToPi(q(3) + q0(3));
    q4 = wrapToPi(q(4) + q0(4));
    q5 = wrapToPi(q(5) + q0(5));
    q6 = wrapToPi(q(6) + q0(6));

    l1 = d(1);
    l2 = a(2);
    l3 = d(4);
    l4 = d(6);

    w = zeros(6,1);
    w(1) = l2*cos(q1)*cos(q2)+cos(q1)*(l3+l4*cos(q5))*sin(q2+q3)-l4*cos(q1)*cos(q2+q3)*cos(q4)*sin(q5)-l4*sin(q1)*sin(q4)*sin(q5);
    w(2) = l2*cos(q2)*sin(q1)+(l3+l4*cos(q5))*sin(q1)*sin(q2+q3)-l4*cos(q2+q3)*cos(q4)*sin(q1)*sin(q5)+l4*cos(q1)*sin(q4)*sin(q5);
    w(3) = l1-cos(q2+q3)*(l3+l4*cos(q5))+l2*sin(q2)-l4*cos(q4)*sin(q2+q3)*sin(q5);
    w(4) = exp(q6/pi)*(cos(q1)*cos(q5)*sin(q2+q3)-(cos(q1)*cos(q2+q3)*cos(q4)+sin(q1)*sin(q4))*sin(q5));
    w(5) = exp(q6/pi)*(cos(q5)*sin(q1)*sin(q2+q3)+(-cos(q2+q3)*cos(q4)*sin(q1)+cos(q1)*sin(q4))*sin(q5));
    w(6) = exp(q6/pi)*(-cos(q2+q3)*cos(q5)-cos(q4)*sin(q2+q3)*sin(q5));
end
