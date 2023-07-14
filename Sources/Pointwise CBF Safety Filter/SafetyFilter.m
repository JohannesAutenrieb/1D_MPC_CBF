function u= SafetyFilter(x,u_des, params)
%CLF-CBF QP-based conroller

xp      = x;

% lie derivitavies of control barrier function
Lfh     = -params.ap*xp + params.gamma*(params.x_max-xp)^params.orderOfAlpha;
Lgh     = params.bp;

Lfh2    = params.ap*xp + params.gamma*(xp-params.x_min)^params.orderOfAlpha;
Lgh2    = -params.bp;


% Compositve A and b matrix for all inequality constraints

A       = [ Lgh;
            Lgh2;
             1 ;
            -1 ];

b       = [ Lfh;
            Lfh2;
            params.u_0;
            params.u_0];
   
% Fmincon
u_d = [u_des]';
fun = @(u) (u - u_d)'*1*(u - u_d);

options = optimoptions('fmincon','Display','off','Algorithm','sqp');   %"active-set"
u_opt = fmincon(fun,u_d,A,b,[],[],[],[],[],options);

% Return variables
u = u_opt;
    
end