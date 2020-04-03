load 'BenchmarkN30_40_50_60_70.mat'

sFont = 14;
hFig = figure(2);
set(hFig, 'Position', [100 100 1400 800])
set(gcf,'Color','white');
hold all
plot(Benchmark(2,3:end),Benchmark(4,3:end),'LineWidth',1);%'DisplayName','one');
grid on;
xlabel('Number of nodes','FontSize',sFont,'FontWeight','normal');
ylabel('Path length','FontSize',sFont,'FontWeight','normal');
%legend('show')
title('Number of nodes vs Path length')
set(gca,'FontSize',sFont)
grid minor;
%average path length over 10 runs vs number of nodes used

load 'Benchmarkgamma24_32_40_48_56_64_72_nr2.mat'

sFont = 14;
hFig = figure(3);
set(hFig, 'Position', [100 100 1400 800])
set(gcf,'Color','white');
hold all
plot(Benchmark(3,3:end),Benchmark(4,3:end),'LineWidth',1);%'DisplayName','one');
grid on;
xlabel('Gamma','FontSize',sFont,'FontWeight','normal');
ylabel('Path length','FontSize',sFont,'FontWeight','normal');
%legend('show')
title('Value of gamma vs Path length')
set(gca,'FontSize',sFont)
grid minor;
%average path length over 10 runs vs number of nodes used

