function training

p = load('main_params.mat');
Q = zeros(p.n_states, p.n_actions);
trajectories = zeros(p.n_episodes, p.max_steps);

h = waitbar(0, 'Training progress: 0%');
tic;

for ep = 1:p.n_episodes
    state = randi(p.n_states);
    buffer = zeros(p.n_states,1);
    active = true(p.n_states,1);
    for st = 1:p.max_steps
        trajectories(ep, st) = state;
        if rand < p.epsilon
            action = randi(p.n_actions);
        else
            [~, action] = max(Q(state, :));
        end
        [dir, thrust] = decodeAction(action);
        [next_state, reward, buffer, active] = mdpStep(state, dir, thrust, p.x2_dir, buffer, active, p.deactMap, p.states);
        Q(state, action) = Q(state, action) + p.alpha * (reward + p.gamma * max(Q(next_state, :)) - Q(state, action));
        state = next_state;
    end
    if mod(ep,50)==0 || ep==p.n_episodes
        pct = ep / p.n_episodes;
        waitbar(pct, h, sprintf('Training progress: %.1f%%', pct*100));
    end
end
close(h);

policy = zeros(p.n_states,1);
for s = 1:p.n_states
    [~, policy(s)] = max(Q(s, :));
end

% Save outputs
save('policy.mat', 'policy');
save('trajectories.mat', 'trajectories');
fprintf('Training complete. Policy and trajectories saved.\n');

% Plot heatmap of Q-values
figure;
h = heatmap(1:p.n_actions, 1:p.n_states, Q);
h.Title  = 'Heatmap of Q-values';
h.XLabel = 'Action';
h.YLabel = 'State';
colormap parula;
end
