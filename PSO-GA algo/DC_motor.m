function DC_motor()
    close all;

    % 1. CONFIGURATION
    modelName = 'simulation';
    targetSpeed = 2000;
    approx = 1; 

    problem.nVar = 2;
    center_Kp = 0.00122;
    center_Ki = 0.0188;
    problem.Var_min = [center_Kp * 0.8,  center_Ki * 0.8];
    problem.Var_max = [center_Kp * 1.2,  center_Ki * 1.2];

    algorithm = 'GA'; 

    %% FAST RESTART SETUP 
    load_system(modelName);
    set_param(modelName, 'FastRestart', 'on');
    finishup = onCleanup(@() cleanup_fast_restart(modelName));

    %% %%% OPTIMIZATION EXECUTION %%%
    if isempty(gcp('nocreate'))
        parpool('local'); 
    end

    if strcmp(algorithm, 'PSO')
       best_params = execute_PSO(problem, modelName, targetSpeed, approx);
    elseif strcmp(algorithm, 'GA')
       best_params = execute_GA(@(x) cost_evaluator(x, modelName, targetSpeed, approx) ...
            , problem.nVar, problem.Var_min, problem.Var_max);
    end

    assignin('base', 'Kp', best_params(1));
    assignin('base', 'Ki', best_params(2));
    assignin('base', 'targetSpeed', targetSpeed);
    assignin('base', 'approx', approx);
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
    in = in.setModelParameter('Open', 'off');
    
    try
        % Check for logsout
        simOut = sim(in);
        logs = simOut.get('logsout');
        if isempty(logs)
            Cost = 1e10; return;
        end
        
        sig = logs.get('logsout').Values; 
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

function cleanup_fast_restart(modelName)
    if bdIsLoaded(modelName)
        try
            set_param(modelName, 'FastRestart', 'off');
            fprintf('Cleaned up: Fast Restart turned off for %s.\n', modelName);
        catch
        end
    end
end

%% PSO Wrapper
function best_params = execute_PSO(problem, modelName, targetSpeed, approx)
    params.MaxIt = 30;   
    params.nPop = 50;    
    params.w = 0.4;      
    params.wdamp = 1.0; 
    params.c1 = 1.5;     
    params.c2 = 1.5; 

    fprintf('Starting PSO Optimization...\n');
    [best_params, best_cost] = PSO(problem, params, modelName, targetSpeed, approx); 

    fprintf('\n========================================\n');
    fprintf('FINAL PSO VALUES: Kp: %.6f, Ki: %.5f\n', best_params(1), best_params(2));
    fprintf('Minimum Cost: %.4f\n', best_cost);
    fprintf('========================================\n');
end

%% PSO Core Logic
function [Global_Best_Position, Global_Best_Cost] = PSO(problem, params, modelName, targetSpeed, approx)
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
        Cost(i) = cost_evaluator(Position(i,:), modelName, targetSpeed, approx);
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
        Position = max(min(Position, Var_max), Var_min); 
        
        % Parallel evaluation of the population
        parfor i=1:nPop
            Cost(i) = cost_evaluator(Position(i,:), modelName, targetSpeed, approx);
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
        % Global_Best_Position(1) is Kp, Global_Best_Position(2) is Ki
        fprintf('Iter %d: Cost=%.4f | Best Kp=%.7f, Best Ki=%.6f\n', ...
                it, Global_Best_Cost, Global_Best_Position(1), Global_Best_Position(2));
        w = w * wdamp;
    end
end

%% GA Wrapper
function optimal_params = execute_GA(Cost_function, nVar, LB, UB)
    fprintf('Starting GA Optimization...\n');
    options = optimoptions('ga', ...
        'UseParallel', true, ...
        'PopulationSize', 50, ...
        'MaxGenerations', 30, ...
        'PlotFcn', @gaplotbestf);             

    [optimal_params, min_cost] = ga(Cost_function, nVar, [], [], [], [], LB, UB, [], options);       
                                    
    fprintf('\n--- GA Optimization Complete ---\n');
    fprintf('Optimal Kp: %f, Ki: %f\n', optimal_params(1), optimal_params(2));
    fprintf('Minimum Cost: %f\n', min_cost);
end

