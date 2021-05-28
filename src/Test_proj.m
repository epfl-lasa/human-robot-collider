%% File Load
file1 = load('result_phase_1.mat','mydata');
file2 = load('filtered_collision_struct.mat');
file3 = load('result_phase_1_2.mat','mydata');
file4 = load('result_phase_1_3.mat','mydata');
X = file1.mydata;
N = file3.mydata;
M = file4.mydata;
minTime = 0;
maxTime = 60;
minY = -10;
maxY = 5000;
AxisPlots = [minTime maxTime minY maxY];
plot_range = find(data_raw_H3.test_11.time>minTime,1):find(data_raw_H3.test_11.time>maxTime,1);
disp(length(data_filtered.test_11.impact.Fx(2001:20:3201)))
%% Metrics
d = procrustes(X(:,20:20:1000,7)',data_filtered.test_11.impact.Fx(2001:20:2981));
c_sim = getCosineSimilarity(X(:,20:20:1000,7)',data_filtered.test_11.impact.Fx(2001:20:2981));
error = immse(X(:,20:20:1000,7)',data_filtered.test_11.impact.Fx(2001:20:2981));
d1 = procrustes(N(:,20:20:700,7)',data_filtered.test_12.impact.Fx(2001:20:2681));
c_sim1 = getCosineSimilarity(N(:,20:20:700,7)',data_filtered.test_12.impact.Fx(2001:20:2681));
error2 = immse(N(:,20:20:700,7)',data_filtered.test_12.impact.Fx(2001:20:2681));
d2 = procrustes(M(:,20:20:300,7)',data_filtered.test_13.impact.Fx(2001:20:2281));
c_sim2 = getCosineSimilarity(M(:,20:20:300,7)',data_filtered.test_13.impact.Fx(2001:20:2281));
error3 = immse(M(:,20:20:300,7)',data_filtered.test_13.impact.Fx(2001:20:2281));
%c_sim1 = getCosineSimilarity(X{1,1}(1:61,7),data_filtered.test_11.impact.Fx(1:20:1201));
%% VELOCITY MODELLING
%3.11:T12
vel1 = [3.08];
acc1 = [0];
%(1->200; 201->750 T12)
%(1->32;33->66*4 for T6)
%for T15-2, only second for loop needed (why?)
ft = fittype('0.001*((-x-c*9.81*133)/(133*75/(133+75)))');
ft1 = fittype('0.00005*((-x-c*9.81*b)/(b))');
ft2 = fittype('-(x*(133*75/(133+75))+c*9.81*133)');
coeffnames(ft)

for i = 1:200
   
    acc1 = [acc1 (-data_filtered.test_13.impact.Fx(plot_range(i))-0.004*133*9.81)/(133*75/208)];
    %acc1 = [acc1 (-data_filtered.test_7.impact.Fx(plot_range(i))-0.004*133*9.81)/(133*15/148)];
    %acc1 = [acc1 (-data_filtered.test_18.impact.Fx(plot_range(i))-0.004*60*9.81)/(60*15/75)];
end
for i = 201:750
    acc1 = [acc1 (-data_filtered.test_13.impact.Fx(plot_range(i))-0.004*133*9.81)/133];
    %acc1 = [acc1 (-data_filtered.test_7.impact.Fx(plot_range(i))-0.004*133*9.81)/133];
    %acc1 = [acc1 (-abs(data_filtered.test_18.impact.Fx(plot_range(i)))-0.004*60*9.81)/60];
end
for i = 751:length(plot_range)
    acc1 = [acc1 (-abs(data_filtered.test_13.impact.Fx(plot_range(i)))-0.004*133*9.81)/(133*75/208)];
    %acc1 = [acc1 (-data_filtered.test_7.impact.Fx(plot_range(i))-0.004*133*9.81)/(133*15/148)];
    %acc1 = [acc1 (-data_filtered.test_15.impact.Fx(plot_range(i))-0.004*60*9.81)/(60*15/75)];
end
for i = 1:length(plot_range)
    vel1 = [vel1 vel1(i) + acc1(i)*0.00005];
end
%% Model Tryout
deformation = [0];
eff_spring_const = [0];
force = [0];
eff_elastic_mod_human = [0];
eff_elastic_mod_robot = [0];
delta = vel1*0.00005;
for i = 1:1200
    deformation = [deformation deformation(i)+delta(i)];
end
for i = 1:1200
    eff_elastic_mod_human = [eff_elastic_mod_human (1.55*(10^8))*(1+((((16.7*(10^6)))/(6.5*(10^9)))-1)*(2.71^(((deformation(i)/0.003)^1.5)*((((16.7*(10^6))/(6.5*(10^9))).^0.9)))))];
    eff_elastic_mod_robot = [eff_elastic_mod_robot (1.55*(10^8))*(1+((((3300*(10^6)))/(68*(10^9)))-1)*2.71^(((deformation(i)/(2.1*0.025)).^1.5)*(((3300*(10^6))/(68*(10^9))).^0.9)))];
    eff_spring_const = [eff_spring_const (4/3)*((1/(((1-(0.41^2))/abs(eff_elastic_mod_robot(i)))+((1-(0.42^2))/abs(eff_elastic_mod_human(i))))*(((1/(0.003+0.04))+(1/(0.025+0.4)))^(-1/2))))];
    force = [force eff_spring_const(i)*(deformation(i)^1.5)];%
end
%plot(deformation(1:1200),abs(force(1:1200)));
%plot([data_filtered.test_14.time(plot_range(1:250))], abs(force(1:250)))
%plot([data_filtered.test_13.time(2002:3202)], deformation)
%% FITTING
%[v,gofv] = fit(data_filtered.test_13.time(2002:20:2402), velList1(100:120), 'poly3');
[f,goff] = fit(data_filtered.test_1.time(plot_range),data_filtered.test_11.impact.Fx(plot_range), 'gauss2');
[g,gofg] = fit(data_filtered.test_1.time(plot_range),data_filtered.test_12.impact.Fx(plot_range), 'gauss1');
[h, gofh] = fit(data_filtered.test_1.time(plot_range),data_filtered.test_13.impact.Fx(plot_range), 'gauss3');
%% PLOTS
%X->M: plot sim result for vel= 1,...,3.1
plot(X(:,20:20:1400,7))
hold on
plot(N(:,20:20:800,7))
plot(M(:,20:20:320,7))
plot([data_filtered.test_1.time(plot_range)],[data_filtered.test_11.impact.Fx(plot_range)]);
%plot([data_filtered.test_1.time(plot_range)],vel1)
%hold on
plot([data_filtered.test_1.time(plot_range)],[data_filtered.test_12.impact.Fx(plot_range)]);
plot([data_filtered.test_11.time(2001:20:2281)],[data_filtered.test_14.impact.Fx(2001:20:2281)]);
%Plots for velocity profiles, work in progress...

%plot([data_filtered.test_1.time(plot_range)],vel1
%plot([data_filtered.test_13.time(plot_range(1:400))], abs(force(1:400)))
%plot(velList(96:680));
%plot(test);

%Legend and axis names are to be changed later
figName = 'Impact-H3_Legs-Robot[133Kg]';
nfig =1;
hold off
legend('Model (v=1 m/s)','Model(v=2 m/s)','Model(v=3.1 m/s)','Real data (v=1 m/s)','Real data (v=2 m/s)', 'Real data (v=3.1 m/s)')
xlabel('Time [ms]') 
ylabel('Force [N]') 

%% COSINE SIMILARITY
function Cs = getCosineSimilarity(x,y)
% 
% call:
% 
%      Cs = getCosineSimilarity(x,y)
%      
% Compute Cosine Similarity between vectors x and y.
% x and y have to be of same length. The interpretation of 
% cosine similarity is analogous to that of a Pearson Correlation
% 
% R.G. Bettinardi
% -----------------------------------------------------------------
if isvector(x)==0 || isvector(y)==0
    error('x and y have to be vectors!')
end
if length(x)~=length(y)
    error('x and y have to be same length!')
end
xy   = dot(x,y);
nx   = norm(x);
ny   = norm(y);
nxny = nx*ny;
Cs   = xy/nxny;
end