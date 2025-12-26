close all; clc;

% 1. CONFIGURATION
modelName = 'simulation';
targetSpeed = 2000;
nVar = 2;
% Create function handle capturing main workspace variables
Cost_function = @(x) cost_evaluator(x, modelName, targetSpeed);
approx = 2; % first (or second) approximation : approx = 1 (or 2)
algorithm = 'GA'; % 'PSO' or 'GA'

%% %%% FAST RESTART SETUP %%%

% 1. Load the model into memory explicitly
load_system(modelName);

% 2. Check if model is dirty (unsaved changes) to avoid popups
if strcmp(get_param(modelName, 'Dirty'), 'on')
    save_system(modelName);
end

% 3. Enable Fast Restart
set_param(modelName, 'FastRestart', 'on');

% 4. Create a safety object. 
finishup = onCleanup(@() cleanup_fast_restart(modelName));

%% %%% OPTIMIZATION EXECUTION %%%

if strcmp(algorithm, 'PSO')
    problem.nVar = nVar;
    problem.Cost_function = Cost_function;
    
    switch approx 
    case 1
        fprintf('First order approximation:...\n');
        % %% First order approximation
        center_Kp = 0.0022;
        center_Ki = 0.0057;
        problem.Var_min = [center_Kp * 0.1,  center_Ki * 0.8]; 
        problem.Var_max = [center_Kp * 1.2,  center_Ki * 5]; 
        
    case 2
        fprintf('Second order approximation:...\n');
        % %% Second order approximation
        center_Kp = 0.00122;
        center_Ki = 0.0188;
        problem.Var_min = [center_Kp * 0.1,  center_Ki * 0.1]; 
        problem.Var_max = [center_Kp * 10,   center_Ki * 10];
        
    otherwise
        error('Invalid approximation choice: approx must be 1 or 2.');
    end

    execute_PSO(problem);

elseif strcmp(algorithm, 'GA')
    switch approx 
    case 1
        fprintf('First order approximation:...\n');
        % %% First order approximation
        center_Kp = 0.0022;
        center_Ki = 0.0057;
        LB = [center_Kp * 0.1,  center_Ki * 0.8]; 
        UB = [center_Kp * 1.2,  center_Ki * 5]; 
        
    case 2
        fprintf('Second order approximation:...\n');
        % %% Second order approximation
        center_Kp = 0.00122;
        center_Ki = 0.0188;
        LB = [center_Kp * 0.1,  center_Ki * 0.1]; 
        UB = [center_Kp * 10,  center_Ki * 10];
        
    otherwise
        error('Invalid approximation choice: approx must be 1 or 2.');
    end
    execute_GA(Cost_function, nVar, LB, UB);
end

%% %%% LOCAL FUNCTIONS %%%

function Cost = cost_evaluator(x, modelName, targetSpeed)
    Kp = x(1);
    Ki = x(2);
    
    in = Simulink.SimulationInput(modelName);
    in = in.setVariable('Kp', Kp);
    in = in.setVariable('Ki', Ki);
    
    try
        % Using 'sim' in Fast Restart mode
        simOut = sim(in);
        
        if isempty(simOut.find('logsout'))
            Cost = 1e10; return;
        end
        
        logs = simOut.get('logsout');
        % Assuming the signal of interest is the first one logged
        sig = logs.get(1).Values; 
        t = sig.Time; 
        y = sig.Data;
        
        % Instability check
        if any(isnan(y)) || max(y) > targetSpeed * 1.5
            Cost = 1e10; return;
        end
        
        % 1. ITAE (Speed)
        e = abs(targetSpeed - y);
        itae = trapz(t, t .* e);
        
        % 2. Overshoot Penalty
        overshoot = max(0, max(y) - (targetSpeed * 1.02));
        
        Cost = itae + (overshoot * 500000); 
        
    catch
        Cost = 1e10; 
    end
end

function cleanup_fast_restart(modelName)
    % This function runs when the script ends or crashes
    if bdIsLoaded(modelName)
        try
            set_param(modelName, 'FastRestart', 'off');
            fprintf('Fast Restart turned off for %s.\n', modelName);
        catch
            warning('Could not turn off Fast Restart.');
        end
    end
end

%% PSO Wrapper
function execute_PSO(problem)
    % Define algorithm parameters locally
    params.MaxIt = 15;   
    params.nPop = 15;    
    params.w = 0.4;      
    params.wdamp = 1.0; 
    params.c1 = 1.5;     
    params.c2 = 1.5; 

    fprintf('Starting PSO Optimization...\n');
    
    % Run the Optimization
    out = PSO(problem, params); 

    Global_Best = out.Best_Solution;
    final_Kp = Global_Best.Position(1);
    final_Ki = Global_Best.Position(2);

    fprintf('\n========================================\n');
    fprintf('FINAL PRODUCTION VALUES (PSO)\n');
    fprintf('Kp: %.8f\n', final_Kp);
    fprintf('Ki: %.8f\n', final_Ki);
    fprintf('Minimum Cost: %.4f\n', Global_Best.Cost);
    fprintf('========================================\n');
end

%% PSO Core Logic
function out = PSO(problem, params)
    Cost_function = problem.Cost_function; 
    nVar = problem.nVar;
    Var_min = problem.Var_min;
    Var_max = problem.Var_max;
    
    MaxIt = params.MaxIt;
    nPop = params.nPop;
    w = params.w;
    wdamp = params.wdamp;
    c1 = params.c1;
    c2 = params.c2;
    
    Velocity = zeros(nPop, nVar); 
    Position = repmat(Var_min, nPop, 1) + rand(nPop, nVar) .* repmat((Var_max - Var_min), nPop, 1);
    
    Cost = zeros(nPop, 1);
    for i=1:nPop
        Cost(i) = Cost_function(Position(i,:));
    end
    
    PBest_Position = Position;
    PBest_Cost = Cost;
    
    [Global_Best_Cost, Min_Index] = min(PBest_Cost);
    Global_Best_Position = PBest_Position(Min_Index, :);
    
    for it = 1:MaxIt
        r1 = rand(nPop, nVar);
        r2 = rand(nPop, nVar);
        
        Velocity = w * Velocity + ...
                   c1 * r1 .* (PBest_Position - Position) + ...
                   c2 * r2 .* (repmat(Global_Best_Position, nPop, 1) - Position);
        
        Position = Position + Velocity;
        
        % Clamp position to bounds
        Position = max(Position, repmat(Var_min, nPop, 1));
        Position = min(Position, repmat(Var_max, nPop, 1)); 
        
        for i=1:nPop
            Cost(i) = Cost_function(Position(i,:));
            if Cost(i) < PBest_Cost(i)
                PBest_Cost(i) = Cost(i);
                PBest_Position(i,:) = Position(i,:);
                if PBest_Cost(i) < Global_Best_Cost
                    Global_Best_Cost = PBest_Cost(i);
                    Global_Best_Position = PBest_Position(i,:);
                end
            end
        end
        
        fprintf('Iter %d: Cost=%.2f (Kp=%.6f, Ki=%.6f)\n', it, Global_Best_Cost, Global_Best_Position);
        w = w * wdamp;
    end
    
    out.Best_Solution.Position = Global_Best_Position;
    out.Best_Solution.Cost = Global_Best_Cost;
end

%% GA Wrapper
function execute_GA(Cost_function, nVar, LB, UB)
    fprintf('Starting Fine Tuning with GA...\n');

    % --- 2. Set GA Options ---
    options = optimoptions('ga');
    options.PopulationSize = 30;           
    options.MaxGenerations =  10;          
    options.CrossoverFraction = 0.8;       
    options.EliteCount = 2;                
    options.PlotFcn = {@gaplotbestf};      

    % --- 3. Run the Genetic Algorithm ---
    [optimal_params, min_cost] = ga(Cost_function, nVar, ... 
                                    [], [], [], [], ... 
                                    LB, UB, ...         
                                    [], options);       
                                    
    % --- 4. Display Results ---
    optimal_Kp = optimal_params(1);
    optimal_Ki = optimal_params(2);
    fprintf('\n--- GA Optimization Complete ---\n');
    fprintf('Optimal Kp found: %f\n', optimal_Kp);
    fprintf('Optimal Ki found: %f\n', optimal_Ki);
    fprintf('Minimum Cost (ITAE): %f\n', min_cost);
end