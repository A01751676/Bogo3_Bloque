function zmp_secuencias
    % secuencia deseada de caminata -------------------------------------------
    seq = ["o","w","w","c"];

    % constantes --------------------------------------------------------------
    g = 981; % gravedad (cm/s2)
    h = 0.5; % altura del robot (cm)
    tf = 1; % tiempo que dura cada paso
    [l,tmov] = size(seq); % cantidad de pasos
    tmov = tmov*tf; % tiempo total de la secuencia
    
    cte = [g,h];

    Yzmp = 6; % lo max que se desplaza la cadera (cm)
    Xzmp = 0.1;
    [x,y] = getPoints(cte,seq,tf,Yzmp,Xzmp);
end
function [x,y] = getPoints(cte,seq,tf,yzmp,Xzmp)
    
    x = []; y = [];
    t = 0;
    t_p = [];
    [l,s] = size(seq);
    
    for i = 1:s
        t = 0;
        stop = t +tf;
        while t < stop
            % revisar caso de pies
            if seq(i) == "o"
                [k1,k2] = openFront(cte, tf, Xzmp);
            elseif seq(i) == "c"
                [k1,k2] = closeFront(cte, tf, Xzmp);
            else
                [k1,k2] = front(cte, tf, Xzmp); 
            end
            
            % alternar pie al caminar
            if mod(i,2) == 0
                Yzmp = -yzmp;
            else
                Yzmp = yzmp;
            end
            
            y = [y; py(cte,t, tf, Yzmp)];
            x = [x; px(cte,t, k1, k2)];
            t_p = [t_p;t];
            t = t+0.001;
        end
        figure
        %subplot(1,3,1);
        plot(x,y)
        title('Pos cadera X[t], Y[t]')
        xlabel("x [cm]")
        ylabel("y [cm]")
        %xlim([(min(x)-1),(max(x)+1)])
        %ylim([(min(y)-1),(max(y)+1)])
        
        figure
        %subplot(1,3,2);
        subplot(2,1,1);
        plot(t_p,x)
        title('Pos cadera X[t]')
        xlabel("t [s]")
        ylabel("x [cm]")
        %xlim([-1,(max(t)+1)])
        %ylim([(min(x)-1),(max(x)+1)])
        
        %subplot(1,3,3);
        subplot(2,1,2);
        plot(t_p,y)
        title('Pos cadera Y[t]')
        xlabel("t [s]")
        ylabel("y [cm]")
        %xlim([-1,(max(t)+1)])
        %ylim([(min(y)-1),(max(y)+1)])
        

        x = []; y = []; t_p = [];
    end
end

function y = py(cte,t, tf, Yzmp)
    g = cte(1); h = cte(2);
    k1 = (-Yzmp)/(1+exp(sqrt(g/h)*tf));
    k2 = k1*exp(sqrt(g/h)*tf);
    y = k1*exp(sqrt(g/h)*t) + k2*exp(-sqrt(g/h)*t) + Yzmp;
end

function x = px(cte,t, k1, k2)
    g = cte(1); h = cte(2);
    x = k1*exp(sqrt(g/h)*t) + k2*exp(-sqrt(g/h)*t);
end

function [k1,k2] = front(cte, tf, Xzmp)
    g = cte(1); h = cte(2);
    k1 = (-Xzmp)/(1-exp(sqrt(g/h)*tf));
    k2 = -k1*exp(sqrt(g/h)*tf);
end

function [k1,k2] = openFront(cte, tf, Xzmp)
    g = cte(1); h = cte(2);
    k1 = (-Xzmp*exp(sqrt(g/h)*tf)) / ((1-exp(sqrt(g/h)*tf))*(1+exp(sqrt(g/h)*tf)));
    k2 = -k1;
end

function [k1,k2] = closeFront(cte, tf, Xzmp)
    g = cte(1); h = cte(2);
    k1 = (-Xzmp) / ((1-exp(sqrt(g/h)*tf))*(1+exp(sqrt(g/h)*tf)));
    k2 = -k1*exp(2*sqrt(g/h)*tf);
end

