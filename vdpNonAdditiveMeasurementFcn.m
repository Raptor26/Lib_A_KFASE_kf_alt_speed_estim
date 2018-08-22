function yk = vdpNonAdditiveMeasurementFcn(xk,vk)
% vdpNonAdditiveMeasurementFcn Example measurement function for discrete
% time nonlinear state estimators with non-additive measurement noise.
%
% yk = vdpNonAdditiveMeasurementFcn(xk,vk)
%
% Inputs:
%    xk - x[k], states at time k
%    vk - v[k], measurement noise at time k
%
% Outputs:
%    yk - y[k], measurements at time k
%
% The measurement is the first state, plus/minus the measurement noise that
% represent the percentage error. 
%
% See also extendedKalmanFilter, unscentedKalmanFilter

%   Copyright 2016-2016 The MathWorks, Inc.

%#codegen

% The tag %#codegen must be included if you wish to generate code with 
% MATLAB Coder.

yk = xk(1)*(1+vk);
end