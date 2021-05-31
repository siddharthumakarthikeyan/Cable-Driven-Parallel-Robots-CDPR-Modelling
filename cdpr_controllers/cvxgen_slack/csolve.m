% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(x, Q) + c'*x)
%   subject to
%     A*x == b
%     -4975 <= x(1)
%     -4975 <= x(2)
%     -4975 <= x(3)
%     -4975 <= x(4)
%     -4975 <= x(5)
%     -4975 <= x(6)
%     -4975 <= x(7)
%     -4975 <= x(8)
%     x(1) <= 4975
%     x(2) <= 4975
%     x(3) <= 4975
%     x(4) <= 4975
%     x(5) <= 4975
%     x(6) <= 4975
%     x(7) <= 4975
%     x(8) <= 4975
%
% with variables
%        x  14 x 1
%
% and parameters
%        A   6 x 14
%        Q  14 x 14   PSD
%        b   6 x 1
%        c  14 x 1
%
% Note:
%   - Check status.converged, which will be 1 if optimization succeeded.
%   - You don't have to specify settings if you don't want to.
%   - To hide output, use settings.verbose = 0.
%   - To change iterations, use settings.max_iters = 20.
%   - You may wish to compare with cvxsolve to check the solver is correct.
%
% Specify params.A, ..., params.c, then run
%   [vars, status] = csolve(params, settings)
% Produced by CVXGEN, 2017-07-03 04:28:33 -0400.
% CVXGEN is Copyright (C) 2006-2012 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2012 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
