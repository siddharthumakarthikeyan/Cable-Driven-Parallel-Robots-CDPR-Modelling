% csolve  Solves a custom quadratic program very rapidly.
%
% [vars, status] = csolve(params, settings)
%
% solves the convex optimization problem
%
%   minimize(quad_form(x, Q) + c'*x)
%   subject to
%     A*x == b
%     50 <= x(1)
%     50 <= x(2)
%     50 <= x(3)
%     50 <= x(4)
%     50 <= x(5)
%     50 <= x(6)
%     50 <= x(7)
%     50 <= x(8)
%     x(1) <= 10000
%     x(2) <= 10000
%     x(3) <= 10000
%     x(4) <= 10000
%     x(5) <= 10000
%     x(6) <= 10000
%     x(7) <= 10000
%     x(8) <= 10000
%     -10 <= x(9)
%     -10 <= x(10)
%     -10 <= x(11)
%     -10 <= x(12)
%     x(9) <= 10
%     x(10) <= 10
%     x(11) <= 10
%     x(12) <= 10
%
% with variables
%        x  12 x 1
%
% and parameters
%        A   6 x 12
%        Q  12 x 12   PSD
%        b   6 x 1
%        c  12 x 1
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
% Produced by CVXGEN, 2017-07-18 05:19:58 -0400.
% CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com.
% The code in this file is Copyright (C) 2006-2017 Jacob Mattingley.
% CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial
% applications without prior written permission from Jacob Mattingley.

% Filename: csolve.m.
% Description: Help file for the Matlab solver interface.
