<<<<<<< HEAD
function [A,B] = con_System()
% continuous time model
% equilibrium at: x_eq = [0,0,0,0,1,0,0,0]'
% w0 is found from using geostationary orbit https://en.wikipedia.org/wiki/Orbital_speed 

%constants:
i_11 = 4.250; %inertia of spacecraft: kgm^2
i_22 = 4.337; %inertia of spacecraft: kgm^2
i_33 = 3.664; %inertia of spacecraft: kgm^2
w_0 = 3.1e3 / 42e6; %constant, mean angular velocity of F_0 calculated as v/r
I_w = 4e-5; %Axial wheel inertia: kgm^2
k_x = (i_22 - i_33) / i_11;
k_y = (i_11 - i_33) / i_22;
k_z = (i_22 - i_11) / i_33;
k = i_22 - I_w;

A = zeros(7);
B = zeros(7,4);
A(1,3) = (1-k_x)*w_0;
A(1,5) = -8*k_x*w_0^2;
A(2,6) = -6*k_y*i_22*w_0^2/k;
A(3,1) = (k_z-1)*w_0;
A(3,7) = -2*k_z*w_0^2;
A(4,6) = 6*k_y*i_22*w_0^2/k;
A(5,1) = 1/2;
A(6,2) = 1/2;
A(7,3) = 1/2;



end

