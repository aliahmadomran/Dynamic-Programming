# Dynamic-Programming
```matlab
 dp_optimal_control Function Solves discrete-time optimal control problem using Dynamic Programming<br>
    %<br>
    % Inputs:<br>
    %   f      - System dynamics function: x(k+1) = f(x(k), u(k), k)<br>
    %   V      - Stage cost function: V(x(k), u(k), k)<br>
    %   S      - Terminal cost function: S(x(kf), kf)<br>
    %   x0     - Initial state<br>
    %   kf     - Final time step<br>
    %   x_grid - State space discretization<br>
    %   u_grid - Control space discretization<br>
    %
    % Outputs:
    %   x_opt  - Optimal state trajectory<br>
    %   u_opt  - Optimal control sequence<br>
    %   J_opt  - Optimal cost-to-go at initial state<br>
    

<br>
% Define system dynamics<br>
f = @(x, u, k) 4*x - 6*u;<br>

% Define stage cost (note: includes the 1/2 factor)<br>
V = @(x, u, k) 0.5*(2*x^2 + 4*u^2);<br>

% Define terminal cost<br>
S = @(x, kf) (x - x_target)^2;<br>

% Solve the problem<br>
[x_opt, u_opt, J_opt] = dp_optimal_control(f, V, S, x0, kf, x_grid, u_grid);<br>
```
