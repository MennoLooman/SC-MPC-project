global P

[P_LQR,K_LQR,L_LQR,info_LQR] = idare(A_dis,B_dis,Q,R,[],[]);
P = P_gain * P_LQR;

if info_LQR.Report > 0
   error("solve_DARE: DARE not solved properly"); 
end

%can check if L_LQR has values inside unit disk
if abs(L_LQR)>1
    error("solve_DARE: closed feedback eigenvalues not all inside unit disk");
end