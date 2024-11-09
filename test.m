% Definicion de Parametros
p = 5; % Numero de pares de polos
S = 12; % Numero de slots
I = 10; % Corriente

f = 50; % Frecuencia de operacion
w = 2*pi*f; % Velocidad angular
T = 1/f; % Periodo

fm = f/p; % Frecuencia mecanica
Tm = 1/fm; % Periodo mecanico

% Definicion de Geometria
PtoBaseBobinaA = [4, 40];
PtoBaseBobinaB = [-4, 40];

% Funcion de Matriz de Rotacion
MatrizRotacion = @(theta) [cos(theta), -sin(theta); sin(theta), cos(theta)];

Bobinas = cell(1, S);

for i = 1:S
    theta = deg2rad(360/S * (i - 1));
    p1 = MatrizRotacion(theta) * PtoBaseBobinaA';
    p2 = MatrizRotacion(theta) * PtoBaseBobinaB';
    Bobinas{i} = [p1'; p2'];
end

addpath('C:\femm42\mfiles')

% Abre FEMM y el documento
openfemm;
opendocument('GeneradorAL.FEM');

% Definicion de la funcion signo
sign = @(x) 1 * (x >= 0) + -1 * (x < 0);

Delta = [];
TorquesMedios = zeros(1, 19);
step = 10;

for delta = -90:10:90
    disp(['angulo de desfase: ', num2str(delta)]);
    Delta(end + 1) = delta;
    Torque = zeros(1, floor(360/step) + 1);

    for theta = 0:step:360
        disp(['rotacion: ', num2str(theta)]);

        Ia = I * cos(deg2rad(p * theta + delta));
        Ib = I * cos(deg2rad(p * theta - 120 + delta));
        Ic = I * cos(deg2rad(p * theta + 120 + delta));

        mi_setcurrent('A', Ia);
        mi_setcurrent('B', Ib);
        mi_setcurrent('C', Ic);
        mi_setcurrent('-A', -Ia);
        mi_setcurrent('-B', -Ib);
        mi_setcurrent('-C', -Ic);

        mi_analyze(1);
        mi_loadsolution;

        mo_groupselectblock(1);
        Torque(theta/step + 1) = mo_blockintegral(22);

        if theta ~= 360
            disp('Rotando');
            mi_clearselected;
            mo_seteditmode('group');
            mi_selectgroup(1);
            mi_moverotate(0, 0, -1 * sign(delta) * step);
        end
    end

    TorquesMedios((delta + 90)/10 + 1) = mean(Torque);
end

% Graficar los resultados
plot(Delta, TorquesMedios);
xlabel('Delta');
ylabel('Torque Medio');
title('Torque Medio vs Desfase');
