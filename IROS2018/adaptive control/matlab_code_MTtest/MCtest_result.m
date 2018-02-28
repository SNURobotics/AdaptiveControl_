%% Which plot
RandomInit = true;
Varied  = true;
set(0,'defaultfigurecolor',[1 1 1],'defaultLineLineWidth',2')

%% RandomInit
clear ctrs ctr ydt data hBar
if(RandomInit)
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
    
    %% plot    
    figure('Name','RandomInit','pos',[100 100 750 250])
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
    legend('No-adaptation', 'Euclidean', 'Const. pullback', 'Natural','Location','northeastoutside')
    title('')
    xlabel('Noise level (std)')
    ylabel('RMS error (deg)')
    xlim([2.5 42.5])
end

%% Varied sequence
clear ctrs ctr ydt data hBar
if(Varied)
    %% import
    expname_cell = {'NoAdapt','Euclidean','ConstPb','Natural'};
    fname_cell = cell(1,4);
    nTrajNumb     = 5;
    Varied_RMSerror_cell   = cell(1,nTrajNumb);
    MC_mean = zeros(4,nTrajNumb);
    MC_std  = zeros(4,nTrajNumb);
    for expNumb = 1:4
        fname_cell{expNumb} = ['Varied_Noise40_' expname_cell{expNumb} '.txt'];
        fid = fopen(fname_cell{expNumb});
        data = textscan(fid, '%f %f %f %f %f');
        Varied_RMSerror_cell{expNumb} = [data{1} data{2} data{3} data{4} data{5}];
        MC_mean(expNumb,:) = mean(Varied_RMSerror_cell{expNumb});
        MC_std(expNumb,:)  = std(Varied_RMSerror_cell{expNumb});
    end

    %% plot
    figure('Name','Varied','pos',[100 500 600 250])
    a=MC_mean';
    b=MC_std';
    ctrs = (1:nTrajNumb);
    data = a;
    hBar = bar(ctrs, data);
    for k1 = 1:size(a,2)
        ctr(k1,:) = bsxfun(@plus, hBar(1).XData, [hBar(k1).XOffset]');
        ydt(k1,:) = hBar(k1).YData;
    end
    hold on
    errorbar(ctr, ydt, b', '.black','CapSize',3)
    hold off
    legend('No-adaptation', 'Euclidean', 'Const. pullback', 'Natural','Location','northeastoutside')
    title('')
    xlabel('Round')
    ylabel('RMS error (deg)')
    xlim([0.5 5.5])
end


