%AATOQUAT represent an equivalent angle axis representation (or Angle Axis) as a rotation quaterion

% for some background, see here, under "Quaternions"
% https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions

% qw = cos(angle/2)
% qx = ax * sin(angle/2)
% qy = ay * sin(angle/2)
% qz = az * sin(angle/2)

% ( QuattoAA:
% angle = 2 * acos(qw)
% x = qx / sqrt(1-qw*qw)
% y = qy / sqrt(1-qw*qw)
% z = qz / sqrt(1-qw*qw))


function qout = AAtoQuat(theta,K)
    if(size(theta) ~= 1), error('magnitude [deg] must be a scalar, 1-by-1'); end;
    Khat=normvec(K,'dim',2);
    angle=theta*pi/180;
    qw = ones(size(Khat,1),1)*cos(angle/2);
    qx = Khat(:,1) * sin(angle/2);
    qy = Khat(:,2) * sin(angle/2);
    qz = Khat(:,3) * sin(angle/2);
    qout = [qw qx qy qz];
end