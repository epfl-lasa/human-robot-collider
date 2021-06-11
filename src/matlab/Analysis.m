%
% This file allows to get metric results, plots and do the grid search for
% head and leg collisions.
%
%% CONSTANTS DEFINITION
timestep = 0.00005;
head_mass = 2.7;
human_mass = 75;
robot_mass = 133;
eta = 0.004;
%% File Load
%
% Leg, default herzian model: result_l_def_{vel};
% Leg, modified herzian model: result_l_mod_{vel};
% Head, default herzian model: result_h_def_{vel};
% Head, modified herzian model: result_h_mod_{vel};
%
load('H3_raw_collision_struct.mat');
load('Q3_raw_collision_struct.mat');
load('filtered_collision_struct.mat');
file1 = load('result_l_mod_1.mat','mydata');
file3 = load('result_l_mod_15.mat','mydata');
file4 = load('result_l_mod_31.mat','mydata');
data1 = file1.mydata; % 1 m/s
data15 = file3.mydata; % 1.5 m/s
data31 = file4.mydata; % 3.1 m/s
minTime = -0.0;
maxTime = 60;
minY = -10;
maxY = 5000;
AxisPlots = [minTime maxTime minY maxY];
plot_range = find(data_raw_H3.test_11.time>minTime,1):find(data_raw_H3.test_11.time>maxTime,1);
%% Metrics legs
[nrmse, custom] = Metric(data1(1,1:length(data1),7)',...
                                  data_filtered.test_11.impact.Fx(plot_range(1:length(data1))));
[nrmse1, custom1] = Metric(data15(1,1:length(data15),7)',...
                                  data_filtered.test_12.impact.Fx(plot_range(1:length(data15))));  
[nrmse2, custom2] = Metric(data31(1,1:length(data31),7)',...
                                  data_filtered.test_14.impact.Fx(plot_range(1:length(data31))));
%% Metrics head
[nrmse_h, custom_h] = Metric(data1(1,1:length(data1),7)',...
                                  data_filtered.test_5.impact.Fx(plot_range(1:length(data1))));
[nrmse1_h, custom1_h] = Metric(data15(1,1:length(data15),7)',...
                                  data_filtered.test_6.impact.Fx(plot_range(1:length(data15))));  
[nrmse2_h, custom2_h] = Metric(data31(1,1:length(data31),7)',...
                                  data_filtered.test_7.impact.Fx(plot_range(1:length(data31))));
%% ACCELARATION MODELLING FOR SENSITIVITY ANALYSIS (LEGS)
%Initialise accelration and velocity
acc = [0];
acc1 = [0];
acc2 = [0];
vel = [3.1];
vel2 = [1.5];
vel3 = [1.0];
for i = 1:length(plot_range)
    acc = [acc (-data_filtered.test_14.impact.Fx(plot_range(i))-eta*robot_mass*9.81)...
                /(robot_mass*human_mass/(robot_mass+human_mass))];
    acc1 = [acc1 (-data_filtered.test_12.impact.Fx(plot_range(i))-eta*robot_mass*9.81)...
                /(robot_mass*human_mass/(robot_mass+human_mass))];
    acc2 = [acc2 (-data_filtered.test_11.impact.Fx(plot_range(i))-eta*robot_mass*9.81)...
                /(robot_mass*human_mass/(robot_mass+human_mass))];
end
for i = 1:length(plot_range)
    vel = [vel vel(i) + acc(i)*timestep];
    vel1 = [vel1 vel1(i) + acc1(i)*timestep];
    vel2 = [vel2 vel2(i) + acc2(i)*timestep];
end
%% ACCELERATION MODELLING FOR SENSITIVITY ANALYSIS (Head)
acc = [0];
acc1 = [0];
acc2 = [0];
vel = [3.1];
vel1 = [1.5];
vel2 = [1.0];
for i = 1:length(plot_range)
    acc = [acc (-data_filtered.test_7.impact.Fx(plot_range(i))-eta*robot_mass*9.81)...
                /((robot_mass*head_mass/(robot_mass+head_mass)))];
    acc1 = [acc1 (-data_filtered.test_6.impact.Fx(plot_range(i))-eta*robot_mass*9.81)...
                /((robot_mass*head_mass/(robot_mass+head_mass)))];
    acc2 = [acc2 (-data_filtered.test_5.impact.Fx(plot_range(i))-eta*robot_mass*9.81)...
                /((robot_mass*head_mass/(robot_mass+head_mass)))];
end
for i = 1:length(plot_range)
    vel = [vel vel(i) + 0.02*acc(i)*timestep];
    vel1 = [vel1 vel1(i) + 0.02*acc1(i)*timestep];
    vel2 = [vel2 vel2(i) + 0.02*acc2(i)*timestep];
end

%% SENSITIVITY ANALYSIS LEGS
%
% Damping model shall be changed manually
%
%Calculating the deformation at each time step
delta = vel*timestep;
delta1 = vel1*timestep;
delta2 = vel2*timestep;
qual = [];
coef_list = [];
%PARAMS FOR GRID SEARCH
Ea = [-1.3*(10^7)];
Eb = [1.0*(10^8)];
x = [3 2.8];
n_h = [0.3];
n_r = [2.3];
rat_cov_h = [0.65];
rat_cov_r = [0.75];
herz = [1.7];
vel_coef = [1];
%Grid search
for a=1:length(Ea)
    idx_a = a;
    for b=1:length(Eb)
        idx_b = b;
        for c=1:length(x)
            idx_c = c;
            for d=1:length(n_h)
                idx_d = d;
                for e=1:length(n_r)
                    idx_e = e;
                    for f=1:length(rat_cov_h)
                        idx_f = f;
                        for g=1:length(rat_cov_r)
                            idx_g = g;
                            for h=1:length(herz)
                                idx_h = h;
                                for z=1:length(vel_coef)
                                    coef_list = [coef_list [Ea(a); Eb(b);...
                                                   x(c); n_h(d); n_r(e);...
                                                   rat_cov_h(f); rat_cov_r(g);...
                                                   herz(h); vel_coef(z)]]; 
                                    deformation = [0];
                                    deformation1 = [0];
                                    deformation2 = [0];
                                    eff_spring_const = [0];
                                    eff_spring_const1 = [0];
                                    eff_spring_const2 = [0];
                                    force = [0];
                                    force1 = [0];
                                    force2 = [0];
                                    eff_elastic_mod_human = [0];
                                    eff_elastic_mod_robot = [0];
                                    eff_elastic_mod_human1 = [0];
                                    eff_elastic_mod_robot1 = [0];
                                    eff_elastic_mod_human2 = [0];
                                    eff_elastic_mod_robot2 = [0];
                                    for i = 1:1000
                                        deformation = [deformation deformation(i)+delta(i)];
                                        deformation1 = [deformation1 deformation1(i)+delta1(i)];
                                        deformation2 = [deformation2 deformation2(i)+delta2(i)];
                                    end
                                    %Constants used here are taken from
                                    %physical constants for human and
                                    %robot (see collision.py for values)
                                    for i = 1:1000
                                        eff_elastic_mod_human = [eff_elastic_mod_human ((Ea(a)*(vel(i)^1.25))*...
                                                                 (1+((((16.7*(10^6)))/(0.634*(10^9)))-1)...
                                                                 *(exp(((deformation2(i)/0.003)^n_h(d))*((((16.7*(10^6))/(0.634*(10^9))).^rat_cov_h(f)))))))];
                                        eff_elastic_mod_robot = [eff_elastic_mod_robot (Eb(b)*(vel(i)^1.25))...
                                                                *(1+((((3300*(10^6)))/(68*(10^9)))-1)...
                                                                *exp(((deformation2(i)/(x(c)*0.02)).^n_r(e))*(((3300*(10^6))/(68*(10^9))).^rat_cov_r(g))))];
                                        eff_spring_const = [eff_spring_const (4/3)*((1/(((1-(0.41^2))/(eff_elastic_mod_robot(i)))...
                                                            +((1-(0.42^2))/(eff_elastic_mod_human(i))))...
                                                            *(((1/(0.003+0.039))+(1/(0.02+0.4)))^(-1/2))))];
                                        force = [force (eff_spring_const(i)*(deformation(i)^herz(h)))];

                                        eff_elastic_mod_human1 = [eff_elastic_mod_human1 ((Ea(a)*(vel1(i)^1.25))...
                                                                    *(1+((((16.7*(10^6)))/(0.634*(10^9)))-1)...
                                                                    *(exp(((deformation1(i)/0.003)^n_h(d))*((((16.7*(10^6))/(0.634*(10^9))).^rat_cov_h(f)))))))];
                                        eff_elastic_mod_robot1 = [eff_elastic_mod_robot1 (Eb(b)*(vel1(i)^1.25))...
                                                                  *(1+((((3300*(10^6)))/(68*(10^9)))-1)...
                                                                  *exp(((deformation1(i)/(x(c)*0.02)).^n_r(e))*(((3300*(10^6))/(68*(10^9))).^rat_cov_r(g))))];
                                        eff_spring_const1 = [eff_spring_const1 (4/3)*((1/(((1-(0.41^2))/(eff_elastic_mod_robot1(i)))...
                                                             +((1-(0.42^2))/(eff_elastic_mod_human1(i))))...
                                                             *(((1/(0.003+0.039))+(1/(0.02+0.4)))^(-1/2))))];
                                        force1 = [force1 (eff_spring_const1(i)*(deformation1(i)^herz(h)))];

                                        eff_elastic_mod_human2 = [eff_elastic_mod_human2 ((Ea(a)*(vel2(i)^1.25))...
                                                                  *(1+((((16.7*(10^6)))/(0.634*(10^9)))-1)...
                                                                  *(exp(((deformation2(i)/0.003)^n_h(d))*((((16.7*(10^6))/(0.634*(10^9))).^rat_cov_h(f)))))))];
                                        eff_elastic_mod_robot2 = [eff_elastic_mod_robot2 (Eb(b)*(vel2(i)^1.25))...
                                                                  *(1+((((3300*(10^6)))/(68*(10^9)))-1)...
                                                                  *exp(((deformation2(i)/(x(c)*0.02)).^n_r(e))*(((3300*(10^6))/(68*(10^9))).^rat_cov_r(g))))];
                                        eff_spring_const2 = [eff_spring_const2 (4/3)*((1/(((1-(0.41^2))/(eff_elastic_mod_robot2(i)))...
                                                            +((1-(0.42^2))/(eff_elastic_mod_human2(i))))...
                                                            *(((1/(0.003+0.039))+(1/(0.02+0.4)))^(-1/2))))];
                                        force2 = [force2 (eff_spring_const2(i)*(deformation2(i)^herz(h)))];

                                    end
                                    qual = [qual (getError(abs(force(1:length(data31)))', data_filtered.test_14.impact.Fx(plot_range(1:length(data31)))))+...
                                            (getError(abs(force1(1:length(data15)))', data_filtered.test_12.impact.Fx(plot_range(1:length(data15)))))+...
                                            (getError(abs(force2(1:length(data1)))', data_filtered.test_11.impact.Fx(plot_range(1:length(data1)))))];

                                end
                            end
                        end
                    end
                end
            end
        end
    end
end
[best_l, idx_l] = min(qual);
%% SENSITIVITY ANALYSIS Head
%
% Damping model shall be changed manually
%
%Init
qual = [];
coef_list = [];
%Calculating the deformation at each time step
delta = vel*timestep;
delta1 = vel1*timestep;
delta2 = vel2*timestep;
%PARAMS FOR GRID SEARCH
Ea = [5*(10^11)];
Eb = [2.6*(10^10)];
x = [0.3];
n_h = [0.3];
n_r = [2.3];
rat_cov_h = [0.65];
rat_cov_r = [0.75];
herz = [2.5];
vel_coef = [9*(10^8)];
%Grid Search
for a=1:length(Ea)
    idx_a = a;
    for b=1:length(Eb)
        idx_b = b;
        for c=1:length(x)
            idx_c = c;
            for d=1:length(n_h)
                idx_d = d;
                for e=1:length(n_r)
                    idx_e = e;
                    for f=1:length(rat_cov_h)
                        idx_f = f;
                        for g=1:length(rat_cov_r)
                            idx_g = g;
                            for h=1:length(herz)
                                idx_h = h;
                                for z=1:length(vel_coef)
                                    coef_list = [coef_list [Ea(a); Eb(b);...
                                                 x(c); n_h(d); n_r(e);...
                                                 rat_cov_h(f); rat_cov_r(g);...
                                                 herz(h); vel_coef(z)]]; 
                                    deformation = [0];
                                    deformation1 = [0];
                                    deformation2 = [0];
                                    eff_spring_const = [0];
                                    eff_spring_const1 = [0];
                                    eff_spring_const2 = [0];
                                    force = [0];
                                    force1 = [0];
                                    force2 = [0];
                                    eff_elastic_mod_human = [0];
                                    eff_elastic_mod_robot = [0];
                                    eff_elastic_mod_human1 = [0];
                                    eff_elastic_mod_robot1 = [0];
                                    eff_elastic_mod_human2 = [0];
                                    eff_elastic_mod_robot2 = [0];
                                    for i = 1:400
                                        deformation = [deformation deformation(i)+delta(i)];
                                        deformation1 = [deformation1 deformation1(i)+delta1(i)];
                                        deformation2 = [deformation2 deformation2(i)+delta2(i)];
                                    end
                                    for i = 1:400
                                    %Constants used here are taken from
                                    %physical constants for human and
                                    %robot (see collision.py for values)
                                        eff_elastic_mod_human = [eff_elastic_mod_human ((Ea(a))...
                                                                *(1+((((16.7*(10^6)))/(4.7*(10^9)))-1)...
                                                                *(exp(((deformation2(i)/0.003)^n_h(d))*((((16.7*(10^6))/(4.7*(10^9))).^rat_cov_h(f)))))))];
                                        eff_elastic_mod_robot = [eff_elastic_mod_robot (Eb(b))...
                                                                 *(1+((((3300*(10^6)))/(68*(10^9)))-1)...
                                                                 *exp(((deformation2(i)/(x(c)*0.02)).^n_r(e))*(((3300*(10^6))/(68*(10^9))).^rat_cov_r(g))))];
                                        eff_spring_const = [eff_spring_const (4/3)*((1/(((1-(0.41^2))/(eff_elastic_mod_robot(i)))...
                                                                +((1-(0.42^2))/(eff_elastic_mod_human(i))))...
                                                                *(((1/(0.003+0.1))+(1/(0.02+0.4)))^(-1/2))))];
                                        force = [force eff_spring_const(i)*(deformation(i)^herz(h))...
                                                 +(vel_coef*vel(i)*(deformation(i)^herz(h)))];

                                        eff_elastic_mod_human1 = [eff_elastic_mod_human1 ((Ea(a))...
                                                                  *(1+((((16.7*(10^6)))/(4.7*(10^9)))-1)...
                                                                  *(exp(((deformation1(i)/0.003)^n_h(d))*((((16.7*(10^6))/(4.7*(10^9))).^rat_cov_h(f)))))))];
                                        eff_elastic_mod_robot1 = [eff_elastic_mod_robot1 (Eb(b))...
                                                                  *(1+((((3300*(10^6)))/(68*(10^9)))-1)...
                                                                  *exp(((deformation1(i)/(x(c)*0.02)).^n_r(e))*(((3300*(10^6))/(68*(10^9))).^rat_cov_r(g))))];
                                        eff_spring_const1 = [eff_spring_const1 (4/3)*((1/(((1-(0.41^2))/(eff_elastic_mod_robot1(i)))...
                                                                  +((1-(0.42^2))/(eff_elastic_mod_human1(i))))...
                                                                  *(((1/(0.003+0.1))+(1/(0.02+0.4)))^(-1/2))))];
                                        force1 = [force1 eff_spring_const1(i)*(deformation1(i)^herz(h))...
                                                    +(vel_coef*vel1(i)*(deformation1(i)^herz(h)))];

                                        eff_elastic_mod_human2 = [eff_elastic_mod_human2 ((Ea(a))...
                                                                    *(1+((((16.7*(10^6)))/(4.7*(10^9)))-1)...
                                                                    *(exp(((deformation2(i)/0.003)^n_h(d))*((((16.7*(10^6))/(4.7*(10^9))).^rat_cov_h(f)))))))];
                                        eff_elastic_mod_robot2 = [eff_elastic_mod_robot2 (Eb(b))...
                                                                    *(1+((((3300*(10^6)))/(68*(10^9)))-1)...
                                                                    *exp(((deformation2(i)/(x(c)*0.02)).^n_r(e))*(((3300*(10^6))/(68*(10^9))).^rat_cov_r(g))))];
                                        eff_spring_const2 = [eff_spring_const2 (4/3)*((1/(((1-(0.41^2))/(eff_elastic_mod_robot2(i)))...
                                                                    +((1-(0.42^2))/(eff_elastic_mod_human2(i))))...
                                                                    *(((1/(0.003+0.1))+(1/(0.02+0.4)))^(-1/2))))];
                                        force2 = [force2 eff_spring_const2(i)*(deformation2(i)^herz(h))...
                                                  +(vel_coef*vel2(i)*(deformation2(i)^herz(h)))];
                                    end
                                    qual = [qual (getError(abs(force(1:length(data31)))', data_filtered.test_7.impact.Fx(plot_range(1:length(data31))))+...
                                            getError(abs(force1(1:length(data15)))', data_filtered.test_6.impact.Fx(plot_range(1:length(data15))))+...
                                            getError(abs(force2(1:length(data1)))', data_filtered.test_5.impact.Fx(plot_range(1:length(data1)))))];
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end
[best_h, idx_h] = min(qual);

%% PLOTS
plot([data_filtered.test_1.time(plot_range(1:length(data1)))],data1(:,1:length(data1),7), 'LineWidth',2)
hold on
plot([data_filtered.test_1.time(plot_range(1:length(data15)))],data15(:,1:length(data15),7), 'LineWidth',2)
plot([data_filtered.test_1.time(plot_range(1:length(data31)))],data31(:,1:length(data31),7),'LineWidth',2)

%Change for the needed test number (5,6,7 for head, 11,12,14 for legs)
plot([data_filtered.test_1.time(plot_range)],[data_filtered.test_11.impact.Fx(plot_range)],'LineWidth',2);
plot([data_filtered.test_1.time(plot_range)],[data_filtered.test_12.impact.Fx(plot_range)],'LineWidth',2);
plot([data_filtered.test_1.time(plot_range)],[data_filtered.test_14.impact.Fx(plot_range)],'LineWidth',2);
%Legend and axis names are to be changed later
figName = 'Impact-H3_Legs-Robot[133Kg]';
nfig =1;
hold off
legend('Model (v=1 m/s)','Model(v=1.5 m/s)','Model(v=3.1 m/s)','Real data (v=1 m/s)','Real data (v=1.5 m/s)', 'Real data (v=3.1 m/s)')
xlabel('Time [ms]') 
ylabel('Force [N]') 
%% HIC CRITERION
default_crit = HIC15_criteria(data_raw_Q3.test_5.time(plot_range(1:length(data1))),data_raw_Q3.test_5.head.areas(plot_range(1:length(data1))), 0.71);
v1_crit = HIC15_criteria(data_raw_Q3.test_5.time(plot_range(1:length(data1))),((data1(:,1:length(data1),7))/(9.81*head_mass))');
default_crit2 = HIC15_criteria(data_raw_Q3.test_6.time(plot_range(1:length(data15))),data_raw_Q3.test_6.head.areas(plot_range(1:length(data15))), 0.71);
v2_crit = HIC15_criteria(data_raw_Q3.test_6.time(plot_range(1:length(data15))),((data15(:,1:length(data15),7))/(9.81*head_mass))');
default_crit3 = HIC15_criteria(data_raw_Q3.test_7.time(plot_range(1:length(data31))),data_raw_Q3.test_7.head.areas(plot_range(1:length(data31))), 0.71);
v3_crit = HIC15_criteria(data_raw_Q3.test_7.time(plot_range(1:48)),((data31(:,1:48,7))/(9.81*head_mass))');
%% FITTING OF REAL-WORLD DATA 
[f,goff] = fit(data_filtered.test_1.time(plot_range),data_filtered.test_11.impact.Fx(plot_range), 'gauss2');
[g,gofg] = fit(data_filtered.test_1.time(plot_range),data_filtered.test_12.impact.Fx(plot_range), 'gauss1');
[h, gofh] = fit(data_filtered.test_1.time(plot_range),data_filtered.test_14.impact.Fx(plot_range), 'gauss3');
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
%% Error function for grid search
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
