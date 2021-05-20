%QUATROTATE Rotate a vector vin by quaternion qin.  
% for some background, see here, under "Quaternions"
% https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions

% In terms of the Euler axis
% 
% \hat{\mathbf{e}} = [e_x\ e_y\ e_z]^\mathrm{T}
% and angle
% 
% \theta
% this versor's components are expressed as follows:
% 

%  q_i = e_x*sin(theta/2)
%  q_j = e_y*sin(theta/2)
%  q_k = e_z*sin(theta/2)
%  q_r = cos(theta/2)


% 
% Also works on a matrix vin if it is M-by-3
function vout = quatrotate(qin,vin) 
  if(size(qin,2) ~= 4), error('qin must be 1-by-4'); end;
  if(size(vin,2) ~= 3), error('vin must be M-by-3'); end;
  
  % Normalize the quaternion 
  q = qin/norm(qin); 
  
  
    % This is what I think the formulation should be:
    % qout[0] = (as+bs-cs-ds)*r[0] + (2*b*c-2*a*d)*r[1] + (2*b*d+2*a*c)*r[2];
    % qout[1] = (2*b*c+2*a*d)*r[0] + (as-bs+cs-ds)*r[1] + (2*c*d-2*a*b)*r[2];
    % qout[2] = (2*b*d-2*a*c)*r[0] + (2*c*d+2*a*b)*r[1] + (as-bs-cs+ds)*r[2];
    %   where 'as' is a^2 where the quaternion is (a + bi +cj + dk), etc.
    % This is what MATLAB wants it to be :
  
  R = zeros(3,3);
  R(1,1) = q(:,1).^2 + q(:,2).^2 - q(:,3).^2 - q(:,4).^2;
  R(1,2) = 2.*(q(:,2).*q(:,3) + q(:,1).*q(:,4));
  R(1,3) = 2.*(q(:,2).*q(:,4) - q(:,1).*q(:,3));
  R(2,1) = 2.*(q(:,2).*q(:,3) - q(:,1).*q(:,4));
  R(2,2) = q(:,1).^2 - q(:,2).^2 + q(:,3).^2 - q(:,4).^2;
  R(2,3) = 2.*(q(:,3).*q(:,4) + q(:,1).*q(:,2));
  R(3,1) = 2.*(q(:,2).*q(:,4) + q(:,1).*q(:,3));
  R(3,2) = 2.*(q(:,3).*q(:,4) - q(:,1).*q(:,2));
  R(3,3) = q(:,1).^2 - q(:,2).^2 - q(:,3).^2 + q(:,4).^2;
  
  vout = (R*vin')';
  
end