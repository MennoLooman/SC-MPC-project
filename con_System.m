function [sys] = con_System()
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

sys.A;
sys.B;

end

