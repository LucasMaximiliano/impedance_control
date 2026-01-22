function [q1,q2] = CoordinateTransform(th1,th2)
%COORDINATE_TRANSFORM Converts coordinates from original simulation code
%   reference frame to our frame definition.
%   - th1: Angle of the first joint  [rad]
%   - th2: Angle of the second joint [rad]
q1 = pi/2 - th1;
q2 = th2;
end