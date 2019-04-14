function invQ = quatInv(Q)
%  quatInv Calculate the inverse of a unit quaternion.
%   invQ = quatInv(Q) calculates the quaternion inverse, invQ, of Q. 
%   Q must have the form Q = [n e] where n is the scalar and e the vector
%   part.
%   Note: Q must be a unit quaternion.

invQ = zeros(size(Q));

invQ(1) = Q(1);
invQ(2:4) = -Q(2:4);

end

