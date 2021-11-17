function [perf_handle] = plot_swarm_fitness(time_history, order, safety, safety_obs, ...
        min_d_obs, dist_error, speed_error, dir_error, speed_delta, p_swarm, dirname)

% Plot swarm fitness function - plot the fitness functions
fontsize = 12;

perf_handle = figure;
plot(time_history, safety, 'LineWidth', 1.5);
hold on;
plot(time_history, order, 'LineWidth', 1.5);
hold on;
plot(time_history, safety_obs, 'LineWidth', 1.5);
hold on;
xlabel('Time [s]', 'fontsize', fontsize);
ylabel('Performance', 'fontsize', fontsize);
legend('safety','order','safety obsacles');

dist_obs_handle = figure;
plot(time_history, min(min_d_obs,[],2), 'LineWidth', 1.5);
yline(0,'LineWidth', 1.5);
xlabel('Time [s]', 'fontsize', fontsize);
ylabel('Distance to obstacles', 'fontsize', fontsize);

error_handle = figure;
plot(time_history, dist_error, 'LineWidth', 1.5);
hold on;
plot(time_history, speed_error, 'LineWidth', 1.5);
hold on;
plot(time_history, dir_error, 'LineWidth', 1.5);
xlabel('Time [s]', 'fontsize', fontsize);
legend('Distance error', 'Speed error','Direction error')

accel_handle = figure;
plot(time_history(1:(end-1)), speed_delta, 'LineWidth', 1.5);
xlabel('Time [s]', 'fontsize', fontsize);
ylabel('Acceleration', 'fontsize', fontsize);

% Save only if 'dirname' is different from '[]'
if ~isempty(dirname)

    file_path = strcat(dirname,'/performance');
    savefig(perf_handle,file_path);
    print(perf_handle,file_path,'-dpng','-r300');

    file_path = strcat(dirname,'/dist_obs');
    savefig(dist_obs_handle,file_path);
    print(dist_obs_handle,file_path,'-dpng','-r300');

    file_path = strcat(dirname,'/error');
    savefig(error_handle,file_path);
    print(error_handle,file_path,'-dpng','-r300');

    file_path = strcat(dirname,'/speed_deltas');
    savefig(accel_handle,file_path);
    print(accel_handle,file_path,'-dpng','-r300');

end

close(perf_handle);
close(dist_obs_handle);
close(error_handle);
close(accel_handle);

end