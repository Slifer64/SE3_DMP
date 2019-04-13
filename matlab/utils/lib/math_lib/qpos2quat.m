function Q1 = qpos2quat(qPos, Q2)
%% qpos2quat
%  Converts the pseudo quaternion position 'qPos' to unit quaternion 'Q1'
%  using as origin the unit quaternion 'Q2'. 
%  Given two unit quaternions Q1, Q2, the pseudo quaternion position of Q1 with
%  respect to Q2 is calculated as: 
%  qPos = log(Q1 * Q2^-1)
%  The original unit quaternion from a pseudo quaternion position us given by:
%  Q1 = exp(qPos) * Q2
%  @param[in] qPos: 3 X N matrix where each column is a pseudo quaternion position.
%  @param[in] Q2: 4 x 1 vector which is the unit quaternion origin (optional, default = [1 0 0 0]').
%  @param[out] Q1: 4 X N matrix where the j-th column is the unit quaternion resulting from the j-th pseudo position and the j-th origin.

n = size(qPos,2);

if (nargin < 2)
    Q2 = [1 0 0 0]';
end

Q1 = zeros(4, n);

for j=1:size(qPos,2)
    Q1(:,j) = quatProd(quatExp(qPos(:,j)), Q2);
end

end
