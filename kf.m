%guilherme moreira rodrigues UFPB 20160105205%guilherme moreira rodrigues UFPB 20160105205
function [angleXKF, angleYKF,biasKF, rateKF, P_, K, y, x] = fcn(dt, Q_angle, Q_bias, R_measure, newRate, newAngleX, newAngleY)


%matriz de covariância de erro (ruido de processo)
Q = [Q_angle 0;0 Q_bias]*dt;
A = [1  -dt;0  1];
C = [1 0];


angleXKF = 0.0; %Redefine angulo
angleYKF = 0.0;
biasKF = 0.0;  %Redefine bias

%Equações discretas de atualização do tempo do filtro de kalman (projeta o estado a frente)
rateKF = newRate - biasKF;

persistent angle;
if isempty(angle)
angle = 0;
else angle = angle + (dt * rateKF);     
end
angleXKF = angle;

persistent angle2;
if isempty(angle2)
angle2 = 0;
else angle2 = angle2 + (dt * rateKF);     
end
angleYKF = angle2;


%atualiza a matriz de covariancia da estimativa de erro (projeta a covariancia de erro do estado a frente)
persistent P;
if isempty(P);P = [0 0 ; 0 0]; 
else P = A*P*A' + Q;
end
P_ = P

%equações discretas de atualização de medição do filtro de kalman (Calcula o ganho de kalman)
S = C*P*C' + R_measure; %estimativa de erro
K = (P_*C')/S %ganho de kalman 

%calcular a estimativa do novo angulo
y = newAngleX - angleXKF; %diferença de angulo
xk = angleXKF + K*y
angleXKF = xk(1);
biasKF = xk(2);
angle = angleXKF

x = newAngleY - angleYKF;
xk = angleYKF + K*x
angleYKF = xk(1);
biasKF = xk(2);
angle2 = angleYKF



%atualizar a covariancia de erro
I = [1 0;0 1];
P_ = (I - K*C)*P_;

P = P_;

return

end 
angleKF = 0.0; %Redefine angulo
biasKF = 0.0;  %Redefine bias

%Equações discretas de atualização do tempo do filtro de kalman (projeta o estado a frente)
rateKF = newRate - biasKF;

persistent angle;
if isempty(angle)
angle = 0;
else angle = angle + (dt * rateKF); 
end
angleKF = angle;

%atualiza a matriz de covariancia da estimativa de erro (projeta a covariancia de erro do estado a frente)
persistent P;
if isempty(P);P = [0 0 ; 0 0]; 
else P = A*P*A' + Q;
end
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
