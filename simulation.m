%% simulation.m
function simulation
% Load parameters
p = load('main_params.mat', 'n_states','sim_steps','x2_dir','deactMap','states', 'parameters'); 
states      = p.states;
n_states    = p.n_states;
sim_steps   = p.sim_steps;
x2_dir      = p.x2_dir;
deactMap    = p.deactMap;
load('policy.mat','policy');
buffer = zeros(p.n_states,1);
active = true(p.n_states,1);
state = 1;
traj = zeros(1,p.sim_steps);
for t = 1:p.sim_steps
    traj(t) = state;
    action = policy(state);
    [dir,th] = decodeAction(action);
    [ns,~,buffer,active] = mdpStep(state,dir,th,p.x2_dir,buffer,active,p.deactMap,p.states);
    state = ns;
end
%fprintf('Simulated trajectory:\n'); disp(traj);
% simply index into states:
% x = zeros(length(traj), size(states,2));
% for k = 1:length(traj)
%     x(k,:) = states(traj(k), :);
% end

% Pre‑allocate and fill
x = [ repmat(states, 1, 1); states(traj, :) ];
%x = [  states(traj, :) ];

assignin('base','x',x)

%% Animation
time_step = p.parameters.time.dt;
Vc        = p.parameters.current.vc;
alphac    = p.parameters.current.alphac;
Time = (0:p.sim_steps-1)' * time_step;
animate(p.parameters, Time, x, Vc, alphac)

end
