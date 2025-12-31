function DC_motor()
    close all;

    % 1. CONFIGURATION
    modelName = 'simulation';
    targetSpeed = 2000;
    problem.nVar = 2;
    approx = 2; 

    % Define the cost function handle
    problem.Cost_function = @(x) cost_evaluator(x, modelName, targetSpeed, approx);

    center_Kp = 0.00122;
    center_Ki = 0.0188;
    problem.Var_min = [center_Kp * 0.95,  center_Ki * 0.95];
    problem.Var_max = [center_Kp * 1.15,  center_Ki * 1.15];

    algorithm = 'PSO'; 

    %% FAST RESTART SETUP 
    load_system(modelName);
    set_param(modelName, 'FastRestart', 'on');

    % This object ensures Fast Restart is turned OFF when the function ends/crashes
    finishup = onCleanup(@() cleanup_fast_restart(modelName));

    %% %%% OPTIMIZATION EXECUTION %%%
    if isempty(gcp('nocreate'))
        parpool('local'); 
    end

    if strcmp(algorithm, 'PSO')
        execute_PSO(problem);
    elseif strcmp(algorithm, 'GA')
        execute_GA(problem.Cost_function, problem.nVar, problem.Var_min, problem.Var_max);
    end
end 
%% %%% LOCAL FUNCTIONS %%%

function Cost = cost_evaluator(x, modelName, targetSpeed, approx)
    Kp = x(1);
    Ki = x(2);
    
    if ~bdIsLoaded(modelName)
        load_system(modelName);
    end
    
    in = Simulink.SimulationInput(modelName);
    in = in.setVariable('Kp', Kp);
    in = in.setVariable('Ki', Ki);
    in = in.setVariable('targetSpeed', targetSpeed); 
    in = in.setVariable('approx', approx);
    in = in.setModelParameter('FastRestart', 'on');
    
    try
        simOut = sim(in);
        
        % Check for logsout
        logs = simOut.get('logsout');
        if isempty(logs)
            Cost = 1e10; return;
        end
        
        sig = logs.get(1).Values; 
        t = sig.Time; 
        y = sig.Data;
        
        % ITAE Calculation
        e = abs(targetSpeed - y);
        itae = trapz(t, t .* e);
        
        % Overshoot Penalty
        overshoot = max(0, max(y) - (targetSpeed * 1.02));
        Cost = itae + (overshoot * 500000); 
        
    catch
        Cost = 1e10; 
    end
end

% --- THE MISSING FUNCTION THAT CAUSED YOUR ERROR ---
function cleanup_fast_restart(modelName)
    if bdIsLoaded(modelName)
        try
            set_param(modelName, 'FastRestart', 'off');
            fprintf('Cleaned up: Fast Restart turned off for %s.\n', modelName);
        catch
            % Already closed or handled
        end
    end
end

%% PSO Wrapper
function execute_PSO(problem)
    params.MaxIt = 20;   
    params.nPop = 40;    
    params.w = 0.4;      
    params.wdamp = 1.0; 
    params.c1 = 1.5;     
    params.c2 = 1.5; 

    fprintf('Starting PSO Optimization...\n');
    out = PSO(problem, params); 

    Global_Best = out.Best_Solution;
    fprintf('\n========================================\n');
    fprintf('FINAL PSO VALUES: Kp: %.6f, Ki: %.5f\n', Global_Best.Position(1), Global_Best.Position(2));
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
    
    % Initial Evaluation
    parfor i=1:nPop
        Cost(i) = Cost_function(Position(i,:));
    end
    
    PBest_Position = Position;
    PBest_Cost = Cost;
    [Global_Best_Cost, Min_Index] = min(PBest_Cost);
    Global_Best_Position = PBest_Position(Min_Index, :);
    
    for it = 1:MaxIt
        r1 = rand(nPop, nVar);
        r2 = rand(nPop, nVar);
        
        % Update Velocity & Position
        Velocity = w * Velocity + ...
                   c1 * r1 .* (PBest_Position - Position) + ...
                   c2 * r2 .* (repmat(Global_Best_Position, nPop, 1) - Position);
        
        Position = Position + Velocity;
        Position = max(min(Position, Var_max), Var_min); % Clamp bounds
        
        % Parallel evaluation of the population
        parfor i=1:nPop
            Cost(i) = Cost_function(Position(i,:));
        end
        
        % Sequential update of personal and global bests
        for i=1:nPop
            if Cost(i) < PBest_Cost(i)
                PBest_Cost(i) = Cost(i);
                PBest_Position(i,:) = Position(i,:);
                
                if PBest_Cost(i) < Global_Best_Cost
                    Global_Best_Cost = PBest_Cost(i);
                    Global_Best_Position = PBest_Position(i,:);
                end
            end
        end
        
        % --- UPDATED PRINT LINE ---
        % Global_Best_Position(1) is Kp, Global_Best_Position(2) is Ki
        fprintf('Iter %d: Cost=%.4f | Best Kp=%.7f, Best Ki=%.6f\n', ...
                it, Global_Best_Cost, Global_Best_Position(1), Global_Best_Position(2));
        
        w = w * wdamp;
    end
    
    out.Best_Solution.Position = Global_Best_Position;
    out.Best_Solution.Cost = Global_Best_Cost;
end
%% GA Wrapper
function execute_GA(Cost_function, nVar, LB, UB)
    fprintf('Starting GA Optimization...\n');

    % Fixed Syntax for PlotFcn
    options = optimoptions('ga', ...
        'UseParallel', true, ...
        'PopulationSize', 50, ...
        'MaxGenerations', 30, ...
        'CrossoverFraction', 0.8, ...
        'EliteCount', 2, ...
        'PlotFcn', @gaplotbestf);             

    [optimal_params, min_cost] = ga(Cost_function, nVar, [], [], [], [], LB, UB, [], options);       
                                    
    fprintf('\n--- GA Optimization Complete ---\n');
    fprintf('Optimal Kp: %f, Ki: %f\n', optimal_params(1), optimal_params(2));
    fprintf('Minimum Cost: %f\n', min_cost);
end
