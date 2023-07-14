function dxdt= Dynamics(x,u,params)

xp      = x;
xp_dot      = params.ap*xp + params.bp*u;
dxdt = [xp_dot]';

end

