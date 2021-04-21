file1 = load('result_phase_1.mat','mydata');
file2 = load('filtered_collision_struct.mat');
X = file1.mydata;
minTime = 1;
maxTime = 300;
minY = -10;
maxY = 5000;
AxisPlots = [minTime maxTime minY maxY];
plot_range = find(data_raw_H3.test_11.time>minTime,1):find(data_raw_H3.test_11.time>maxTime,1);
disp(length(data_filtered.test_11.impact.Fx(2001:20:3201)))
d = procrustes(X{1,4}(5:44,7),data_filtered.test_11.impact.Fx(2001:40:3561));
c_sim = getCosineSimilarity(X{1,4}(5:44,7),data_filtered.test_11.impact.Fx(2001:40:3561));
%c_sim1 = getCosineSimilarity(X{1,1}(1:61,7),data_filtered.test_11.impact.Fx(1:20:1201));
%plot(X{1,2}(1:100,7))
%plot(X{1,4}(1:60,7))
%hold on
plot(X{1,2}(1:44,7))
hold on
%plot(X{1,2}(:,7))
%hold off

figName = 'Impact-H3_Legs-Robot[133Kg]';
nfig =1;
plot([data_filtered.test_1.time(2001:40:3561)],[data_filtered.test_11.impact.Fx(2001:40:3561)]); 
hold off
legend('Model','Real data')
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