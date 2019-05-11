%guilherme moreira rodrigues UFPB 20160105205



function [angleKF, biasKF, rateKF, P_, K, y] = fcn(dt, Q_angle, Q_bias, R_measure, newRate, newAngle)


%matriz de covariância de erro (ruido de processo)
Q = [Q_angle 0;0 Q_bias]*dt;
A = [1  -dt;0  1];
C = [1 0];

angleKF = 0.0; %Redefine angulo
biasKF = 0.0;  %Redefine bias

%Equações discretas de atualização do tempo do filtro de kalman (projeta o estado a frente)
rateKF = newRate - biasKF;

persistent angle;
angle = angle + (dt * rateKF); 
angleKF = angle;

%atualiza a matriz de covariancia da estimativa de erro (projeta a covariancia de erro do estado a frente)
persistent P;
P = A*P*A' + Q;
P_ = P

%equações discretas de atualização de medição do filtro de kalman (Calcula o ganho de kalman)
S = C*P*C' + R_measure; %estimativa de erro
K = (P_*C')/S %ganho de kalman 

%calcular a estimativa do novo angulo
y = newAngle - angleKF; %diferença de angulo
xk = angleKF + K*y
angleKF = xk(1);
biasKF = xk(2);
angle = angleKF

%atualizar a covariancia de erro
I = [1 0;0 1];
P_ = (I - K*C)*P_;

P = P_;

return

end 
