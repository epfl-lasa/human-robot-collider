% % Computing various metric results
% Created June 11, 2021
% Author: Kravtsov Denis
%% Leg metrics
% 
% Inputs:
%           Model Data  :      [REQ'D]    [m x 1]{double};   Column-wise vector of modelled force data in [N]
%           Real Data    :   [REQ'D]    [m x 1]{double};   Column-wise vector of ground truth forces in [N]
% 
%           
% Outputs:
%           NRMSE metric result : scalar {double} 
%           Custom metric result: : scalar {double}  
% Examples:
% model_data = load('result_1.mat','mydata');
% 
% [nrmse, custom] = Metric(model_data(1,1:length(model_data),7)',...
%                                  data_filtered.test_11.impact.Fx(plot_range(1:length(model_data))));
%%%
function [nrmse,custom] = Metrics(model_data,real_data)
    nrmse = goodnessOfFit(model_data, real_data, 'NRMSE');
    custom = getError(model_data, real_data);
end
%% CUSTOM ERROR FUNCTION

function Pe = getError(x,y)
if isvector(x)==0 || isvector(y)==0
    error('x and y have to be vectors!')
end
if length(x)~=length(y)
    error('x and y have to be same length!')
end
force_ratio = 0;
for i = 1:length(x)
    if (abs(x(i))<abs(y(i)))
        force_ratio = force_ratio + abs(1-(abs(x(i))/abs(y(i)))); 
    elseif (abs(y(i))<=abs(x(i)))
        force_ratio = force_ratio + abs(1-(abs(y(i))/abs(x(i)))); 
    end
end
Pe = force_ratio/(length(y));
end