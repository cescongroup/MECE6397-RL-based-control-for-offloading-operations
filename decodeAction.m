%% decodeAction.m
function [direction,thrust] = decodeAction(idx)
tmp=load('main_params.mat','thrustLevels'); TL=tmp.thrustLevels;
if idx<=5, direction=1; thrust=TL(idx);
else, direction=2; thrust=TL(idx-5); end
end