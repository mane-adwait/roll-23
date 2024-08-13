clear all; close all;

p_start = -pi ; p_end = pi ;
p_sample = p_start : pi/16 : p_end ;

figure; 
% Sin terrain.
p_plot = p_start - pi/4 : 0.1 : p_end + pi/4 ;
plot_terr = plot(p_plot, sin(p_plot), 'k-') ;

hold on; grid on; axis equal;

for iter = 1:numel(p_sample)
    alpha_ts(:,iter) = alpha_terr_func(p_sample(iter)) ; % alpha_terr at the sample point.

%     Unit tangent vector points towards the concave region, and is not
%     defined at zero curvature.
    N_ut(:,iter) = N_unit_terr_func(p_sample(iter)) ; 
    N_ut_end(:,iter) = alpha_ts(:,iter) + N_ut(:,iter) ; % End point of the vector sum.
    plot([alpha_ts(1,iter), N_ut_end(1,iter)], [alpha_ts(2,iter), N_ut_end(2,iter)], 'b-' ) ;

%     Rotating the unit tangent vector pi/2 rad CCW gives us the desired
%     result.
    T_t(:,iter) = T_terr_func(p_sample(iter)) ;
    T_t_CCW(:,iter) = RM_CCW(pi/2) * T_t(:,iter) ;
    T_t_CCW_end(:,iter) = alpha_ts(:,iter) + T_t_CCW(:,iter) ; % End point of the vector sum.
    plot([alpha_ts(1,iter), T_t_CCW_end(1,iter)], [alpha_ts(2,iter), T_t_CCW_end(2,iter)], 'r--' ) ;
end