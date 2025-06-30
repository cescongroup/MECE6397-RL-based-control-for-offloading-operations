function [next_state,reward,buffer,active] = mdpStep(state,dir,th,x2_dir,buffer,active,deactMap,states)

tmp = load('main_params.mat','thrustLevels', 'parameters');
thrustLevels = tmp.thrustLevels;

cur = currentForce(states(state,:), tmp.parameters);
iner = inertiaForce(states(state,:), tmp.parameters);
haws = hawserForce(states(state,:));
Fthr = thrusterForce(states(state,:), tmp.parameters);
res_tot = iner-cur-haws;
%sig = 1-2*(dir==2)
app = Fthr(dir) * th/100;

%state
if abs(buffer(state)+app)-abs(res_tot)>=0 && sign(res_tot)==-sign(buffer(state)+app)
    over = true; buffer(state)=0;
    idxs = deactMap{state}; idxs(idxs == state) = []; active(idxs)=false;
elseif abs(buffer(state)+app) - abs(res_tot) >= 0 && sign(res_tot) == sign(buffer(state)+app)
    over = false;
    buffer(state) = 0;
    if ~active(state+1) 
        idxs = deactMap{state+1}; idxs(idxs == state) = []; active(idxs)=false;
    end
else
    over = false; buffer(state)=buffer(state)+app;
end

act = find(active);
ix = find(act>state,1);
if isempty(ix), next_state=act(1); else next_state=act(ix); end
reward = -1;
%reward = 0;
if (dir==1&&x2_dir(state)==2)||(dir==2&&x2_dir(state)==1), reward=reward+1; else reward=reward-1; end
if over, reward=reward+2; if any(next_state==[15,65]), reward=reward+5; end; end
end