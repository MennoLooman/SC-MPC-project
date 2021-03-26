function f = non_lin_model(x,u)

skew =@(x) [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];

%input: u = [tau_1 tau_2 tau_3 tau_w]'
tau = u(1:3);
tau_w = u(4);

%state: x = [w_1 w_2 w_3 w_w e_1 e_2 e_3, n]'
wb_ob = x(1:3);
w_w = x(4);
e = x(5:7);
n = x(8); %n = sqrt(1-e'*e);

%model prep
I = diag([4.250, 4.337, 3.664]);
I_w = 4e-5;
L = [0,1,0]';
J = diag([4.25 4.33696 3.664]); %J = I-L*I_w*L'; %time save
J_inv = diag([1/4.25 1/4.33696 1/3.664]); %J_inv = inv(J); %time save
w_0 = 3.1e3 / 42e6;
Rb_o = eye(3)+2*n*skew(e)+2*skew(e)*skew(e); %not sure about this line
dRb_o = -skew(wb_ob)*Rb_o;
c_2 = Rb_o(:,2);
c_3 = Rb_o(:,3);
dc_2 = dRb_o(:,2);

% dwb_ob and dw_w
f_h_inert = J_inv*(-skew(wb_ob-w_0*c_2)*(I*(wb_ob-w_0*c_2)+L*I_w*w_w));
f_d_inert = L'*J_inv*skew(wb_ob-w_0*c_2)*(I*(wb_ob-w_0*c_2)+L*I_w*w_w);
f_h_t = J_inv*tau - J_inv*L*tau_w;
f_d_t = -L'*J_inv*tau + (L'*J_inv*L+I_w^-1)*tau_w;
f_h_g = J_inv*cross(3*w_0^2*c_3,I*c_3);
f_d_g = -L'*J_inv*cross(3*w_0^2*c_3,I*c_3);
f_h_add = w_0 * dc_2;
dwb_ob = f_h_inert + f_h_t + f_h_g + f_h_add;
dw_w   = f_d_inert + f_d_t + f_d_g;

% dn and de
dn = -e'*wb_ob/2;
de = (n*eye(3)+skew(e))*wb_ob/2;

%final derivative:
f = [dwb_ob; dw_w; de; dn];
end