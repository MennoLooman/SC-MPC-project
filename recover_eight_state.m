function [state_seq_8] = recover_eight_state(state_seq_7)
%RECOVER_EIGHT_STATE Returns the complete state description based on the
%states comming from the MPC
%   only works when time horizon is > 7
%   the extra state is inserted in the place as mentioned to the article.
    N = length(state_seq_7);
    state_seq_8 = zeros(8, N);
    state_seq_8(1:4, :) = state_seq_7(1:4, :);
    state_seq_8(6:8, :) = state_seq_7(5:7, :);
    
    for i = 1:N
        eta = sqrt(1 - state_seq_8(6:8, i)'*state_seq_8(6:8, i));
        state_seq_8(5, i) = eta;
    end
end

