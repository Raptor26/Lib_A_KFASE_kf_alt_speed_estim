clear all;
close all;
initialStateGuess = [2;0]; % xhat[k|k-1]

% Construct the filter
ukf = unscentedKalmanFilter(...
    @vdpStateFcn,... % State transition function
    @vdpMeasurementNonAdditiveNoiseFcn,... % Measurement function
    initialStateGuess,...
    'HasAdditiveMeasurementNoise',false);

R = 0.01; % Variance of the measurement noise v[k]
ukf.MeasurementNoise = R;

ukf.ProcessNoise = diag([1 1.1]);

ukf.Alpha = 0.01;

T = 0.01; % [s] Filter sample time
timeVector = 0:T:15;
[~,xTrue]=ode45(@vdp1,timeVector,[1;0]);

rng(1); % Fix the random number generator for reproducible results
yTrue = xTrue(:,1);
yMeas = yTrue .* (1+sqrt(R)*randn(size(yTrue))); % sqrt(R): Standard deviation of noise

Nsteps = numel(yMeas); % Number of time-steps
xCorrected = zeros(Nsteps,2); % Corrected state estimates
PCorrected = zeros(Nsteps,2,2); % Corrected state estimation error covariances
e = zeros(Nsteps,1); % Residuals (or innovations)

for k=1:Nsteps
    % Let k denote the current time
    %
    % Residuals (or innovations): Measured output - Predicted output
    e(k) = yMeas(k) - vdpMeasurementFcn(ukf.State); % ukf.State is x[k|k-1] at this point
    % Incorporate the measurements at time k into the state estimates by
    % using the "correct" command. This updates the State and StateCovariance
    % properties of the filter to contain x[k|k] and P[k|k]. These values
    % are also produced as the output of the "correct" command.
    [xCorrected(k,:), PCorrected(k,:,:)] = correct(ukf,yMeas(k));
    % Predict the states at next time step, k+1. This updates the State and
    % StateCovariance properties of the filter to contain x[k+1|k] and
    % P[k+1|k]. These will be utilized by the filter at the next time-step
    predict(ukf);
end

figure();
subplot(2,1,1);
plot(timeVector,xTrue(:,1),timeVector,xCorrected(:,1));
legend('True','Filter estimate','Measured')
ylim([-3.6 3.6]);
ylabel('x_1');
subplot(2,1,2);
plot(timeVector,xTrue(:,2),timeVector,xCorrected(:,2));
ylim([-3 3.5]);
xlabel('Time [s]');
ylabel('x_2');

figure();
plot(timeVector, PCorrected(:,1), timeVector, PCorrected(:,2));
