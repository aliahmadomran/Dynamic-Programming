function [x_opt, u_opt, J_opt] = dp_optimal_control(f, V, S, x0, kf, x_grid, u_grid)
    % DP_OPTIMAL_CONTROL Solves discrete-time optimal control problem using Dynamic Programming
    %
    % Inputs:
    %   f      - System dynamics function: x(k+1) = f(x(k), u(k), k)
    %   V      - Stage cost function: V(x(k), u(k), k)
    %   S      - Terminal cost function: S(x(kf), kf)
    %   x0     - Initial state
    %   kf     - Final time step
    %   x_grid - State space discretization
    %   u_grid - Control space discretization
    %
    % Outputs:
    %   x_opt  - Optimal state trajectory
    %   u_opt  - Optimal control sequence
    %   J_opt  - Optimal cost-to-go at initial state
    
    N = kf;  % Time horizon
    nx = length(x_grid);
    nu = length(u_grid);
    
    % Initialize cost-to-go and optimal control matrices
    J = zeros(nx, N+1);  % J(:,k) represents J_k(x)
    U_opt = zeros(nx, N);
    
    % Terminal cost
    for i = 1:nx
        J(i,N+1) = S(x_grid(i), N);
    end
    
    % Backward recursion
    for k = N:-1:1
        for i = 1:nx
            current_x = x_grid(i);
            min_cost = inf;
            optimal_u = u_grid(1);  % Initialize with first control
            
            % Find optimal control for current state
            for j = 1:nu
                current_u = u_grid(j);
                
                % Calculate next state
                next_x = f(current_x, current_u, k-1);  % k-1 because MATLAB is 1-indexed
                
                % Find closest grid point to next_x
                [~, next_idx] = min(abs(x_grid - next_x));
                
                % Calculate total cost
                stage_cost = V(current_x, current_u, k-1);
                total_cost = stage_cost + J(next_idx, k+1);
                
                % Update minimum cost
                if total_cost < min_cost
                    min_cost = total_cost;
                    optimal_u = current_u;
                end
            end
            
            % Store optimal cost and control
            J(i,k) = min_cost;
            U_opt(i,k) = optimal_u;
        end
    end
    
    % Forward simulation
    x_opt = zeros(1,N+1);
    u_opt = zeros(1,N);
    x_opt(1) = x0;
    
    for k = 1:N
        [~, idx] = min(abs(x_grid - x_opt(k)));
        u_opt(k) = U_opt(idx,k);
        x_opt(k+1) = f(x_opt(k), u_opt(k), k-1);
    end
    
    % Find optimal cost at initial state
    [~, init_idx] = min(abs(x_grid - x0));
    J_opt = J(init_idx,1);
end


% Define the specific problem parameters
x0 = 8;          % Initial state
kf = 2;          % Final time step
x_target = 20;   % Target state at final time

% State and control space discretization
x_min = -50; x_max = 50; dx = 0.02;
x_grid = x_min:dx:x_max;
u_min = -10; u_max = 10; du = 0.02;
u_grid = u_min:du:u_max;

% Define system dynamics
f = @(x, u, k) 4*x - 6*u;

% Define stage cost (note: includes the 1/2 factor)
V = @(x, u, k) 0.5*(2*x^2 + 4*u^2);

% Define terminal cost
S = @(x, kf) (x - x_target)^2;

% Solve the problem
[x_opt, u_opt, J_opt] = dp_optimal_control(f, V, S, x0, kf, x_grid, u_grid);

% Display results
disp('Optimal Control Sequence:');
disp(u_opt);
disp('Optimal State Trajectory:');
disp(x_opt);
disp('Total Optimal Cost:');
disp(J_opt);

% Plot results
figure;
subplot(2,1,1);
stem(0:kf, x_opt, 'filled', 'LineWidth', 2);
title('Optimal State Trajectory');
xlabel('Time step k');
ylabel('State x(k)');
grid on;

subplot(2,1,2);
stem(0:kf-1, u_opt, 'filled', 'LineWidth', 2);
title('Optimal Control Sequence');
xlabel('Time step k');
ylabel('Control u(k)');
grid on;