%% import
fname_cell      = {'Noise05.txt', 'Noise10.txt', 'Noise15.txt', 'Noise20.txt', 'Noise25.txt', 'Noise30.txt', 'Noise35.txt', 'Noise40.txt'};
nNoiseLevel     = size(fname_cell,2);
Random_RMSerror_cell   = cell(1,nNoiseLevel);
MC_mean = zeros(4,nNoiseLevel);
MC_std  = zeros(4,nNoiseLevel);

for i=1:nNoiseLevel
    fid = fopen(fname_cell{i});
    data = textscan(fid, '%f %f %f %f');
    Random_RMSerror_cell{i} = [data{1} data{2} data{3} data{4}];
    MC_mean(:,i) = mean(Random_RMSerror_cell{i})';
    MC_std(:,i)  = std(Random_RMSerror_cell{i})';
end

%%
set(0,'defaultfigurecolor',[1 1 1],'defaultLineLineWidth',2)

figure(1000)
a=MC_mean';
b=MC_std';
ctrs = 5*(1:nNoiseLevel);
data = a;
hBar = bar(ctrs, data);
for k1 = 1:size(a,2)
    ctr(k1,:) = bsxfun(@plus, hBar(1).XData, [hBar(k1).XOffset]');
    ydt(k1,:) = hBar(k1).YData;
end
hold on
errorbar(ctr, ydt, b', '.black','CapSize',3)
hold off
legend('No-adaptation', 'Euclidean', 'Const. pullback', 'Natural')
title('Random initialization')
xlabel('Noise level (std)')
ylabel('RMS error (deg)')

