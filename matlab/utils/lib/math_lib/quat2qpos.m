function qPos = quat2qpos(Q1, Q2)
%% quat2qpos
%  Converts the unit quaternion 'Q1' to pseudo quaternion position 'qPos'
%  using as origin the unit quaternion 'Q2'. 
%  Given two unit quaternions Q1, Q2, the pseudo quaternion position of Q1 with
%  respect to Q2 is calculated as: 
%  qPos = log(Q1 * Q2^-1)
%  The original unit quaternion from a pseudo quaternion position us given by:
%  Q1 = exp(qPos) * Q2
%  @param[in] Q1: 4 X N matrix where the j-th column is the unit quaternion resulting from the j-th pseudo position and the j-th origin.
%  @param[in] Q2: 4 x 1 vector which is the unit quaternion origin (optional, default = [1 0 0 0]').
%  @param[out] qPos: 3 X N matrix where each column is a pseudo quaternion position.

    n = size(Q1,2);
    
    if (nargin < 2)
        Q2 = [1 0 0 0]';
    end
    
    qPos = zeros(3, n);
    
    for i=1:size(qPos,2)
        qPos(:,i) = quatLog(quatProd(Q1(:,i),quatInv(Q2)));
    end

end
