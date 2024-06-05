% BOGOBOT V4 SIMULACIONES -------------------------------------------------
function BogobotV4_legsSim 
    % parametros del robot
    L1 = 1; L3 = 1; L4 = 1;
    L = [L1 L3 L4];
    
    % valores articulares
    % q = [0 0 0 0 0 0 0 0 0 0 0 0];
    % P = trayectory points, A = alpha angles 
%     P = [-1 0 -2 -1 0 -2]; 
     A = [0 0];
%     q = IK_legs(L,P,A);
%     
%     [pts, T0F] = FK_legs(L,q);
%     showLegs(L,pts)
%     addFrames(T0F,0.3)
    
    xlim = 1; ylim = 0; piso= 1.5; radio = 0.6; 
    T = 5; dt = 0.2;
    % trayectory(xlim, ylim, piso, radio, T, dt, UD)
    PD = trayectory(xlim, ylim, piso, radio, T, dt,1);
    PI = trayectory(xlim, ylim, piso, radio, T, dt,0);
    P = [PD PI];
    
    PD = trayectory(xlim, ylim, piso, radio, T, dt,0);
    PI = trayectory(xlim, ylim, piso, radio, T, dt,1);
    P = [P; PD PI];
    q = IK_legs2(L,P,A);
    [pts, T0F] = FK_legs2(L,q);
    dt = 0.2;
    % showLegs2(L,pts,T0F,dt,view)
    showLegs2(L,pts,T0F,dt,0)

end

% FUNCION CON CINEMATICA INVERSA DE AMBAS PIERNAS -------------------------
function q = IK_legs(L,P,A)
    % traectory points left and right legs
    pR = P(1:3); pL = P(4:6); 
    % alpha angle left and right legs
    aR = A(1); aL = A(2);
    
    qR = IK_rightLeg(L,pR,aR);
    qL = IK_leftLeg(L,pL,aL);
    q = [qR, qL];
end

% FUNCION CON CINEMATICA INVERSA DE AMBAS PIERNAS -------------------------
function q = IK_legs2(L,P,A)
    q = [];
    for i=1:size(P,1)
        % traectory points left and right legs
        pR = P(i,1:3);
        pL = P(i,4:6);
        % alpha angle left and right legs
        aR = A(1); aL = A(2);
        qR = IK_rightLeg(L,pR,aR);
        qL = IK_leftLeg(L,pL,aL);
        q = [q; qR, qL];
    end
end

% FUNCION CON CINEMATICA DIRECTA DE AMBAS PIERNAS -------------------------
function [pts, T0F] = FK_legs(L,q)
    [ptsPD, T0D, TFD] = FK_rightLeg(L,q);
    [ptsPI, T0I, TFI] = FK_leftLeg(L,q);
    
    T0F = [T0D, TFD, T0I, TFI];
    pts = [ptsPD, ptsPI];
end

function [pts, T0F] = FK_legs2(L,q)
    pts = [];
    T0F = [];
    for i=1:size(q,1)
        [ptsPD, T0D, TFD] = FK_rightLeg(L,q(i,:));
        [ptsPI, T0I, TFI] = FK_leftLeg(L,q(i,:));

        T0F = [T0F; T0D, TFD, T0I, TFI];
        pts = [pts; ptsPD, ptsPI];
        
    end
end

% FUNCION PARA MOSTRAR LAS PIERNAS DEL ROBOT ------------------------------
function showLegs(L,pts)
    % parametros del robot
    L3 = L(2); L4 = L(3);
    
    % graficar robot en 3D
    %   renglon, elemento, profundidad
    p = plot3(pts(:,:,1)',pts(:,:,2)',pts(:,:,3)',"-o");
    p.Color = "#3E424B";
    p.MarkerFaceColor = "#3E424B";
    p.LineWidth = 2;
    xlabel("X"); ylabel("Y"); zlabel("Z")
    grid on; grid minor
    axis([-1 1 -1 1 0 1]*(L3+L4+0.5)); view(60,60)
end

function showLegs2(L,pts,T0F,dt,v)
    % parametros del robot
    L3 = L(2); L4 = L(3);
    % graficar robot en 3D
    for i=1:size(pts,1)
        %   renglon, elemento, profundidad
        
        plot3(pts(:,2,1)',pts(:,2,2)',pts(:,2,3)',"-", "color", "#008ECC");
        hold on
        plot3(pts(:,7,1)',pts(:,7,2)',pts(:,7,3)',"-", "color", "#ED2939");
        p = plot3(pts(i,:,1)',pts(i,:,2)',pts(i,:,3)',"-o");
        p.Color = "#3E424B";
        p.MarkerFaceColor = "#3E424B";
        p.LineWidth = 2;
        xlabel("X"); ylabel("Y"); zlabel("Z")
        grid on; grid minor
        axis([-1 1 -1 1 0 1]*(L3+L4+0.5)); view(v,v)
        hold off
        j = (i-1)*4 + 1;
        addFrames(T0F(j:j+3,:),0.3)
        
        pause(dt)
    end
end

% FUNCION PARA AGREGAR EJES COORDENADOS A LAS PIERNAS ---------------------
function addFrames(T,Tsize)
    T0D = T(:,1:4); TFD = T(:,5:8);
    T0I = T(:,9:12); TFI = T(:,13:16); 
    
    hold on
    trplot(T0D,'length',Tsize,"color","#008ECC","frame","T0D")
    trplot(TFD,'length',Tsize,"color","#008ECC","frame","TFD")
    trplot(T0I,'length',Tsize,"color","#ED2939","frame","T0I")
    trplot(TFI,'length',Tsize,"color","#ED2939","frame","TFI")
    hold off
end

% FUNCION CON LA CINEMATICA DIRECTA DE LA PIERNA DERECHA ------------------
function [pts, T0D, TFD] = FK_rightLeg(L,q)
    % signos de las q adaptados a bogo3
    % parametros del robot
    L1 = L(1); L3 = L(2); L4 = L(3);
    
    % valores articulares
    q1 = q(1); q2 = q(2); q3 = q(3); q4 = q(4); q5 = q(5); q6 = q(6);
    
    % cinematica directa de los puntos
    T01 = [rotz(-q1),[0;0;0]; 0 0 0 1]; % cadera
    T12 = [rotx(-q2),[0;0;0]; 0 0 0 1]; % cadera
    T23 = [roty(q3),[0;0;0]; 0 0 0 1]; % cadera
    T34 = [roty(q4),[0;0;-L3]; 0 0 0 1]; %rodilla
    T45 = [roty(-q5),[0;0;-L4]; 0 0 0 1]; %tobillo
    T56 = [rotx(q6),[0;0;0]; 0 0 0 1]; %tobillo

    T03 = T01*T12*T23;
    T04 = T03*T34;
    T06 = T04*T45*T56;
    TpieD = T06*[eye(3), [0.2;0;0]; 0 0 0 1];
    
    PD = [0,-L1,L3+L4]; % para acomodar la pata derecha en dibujo
    
    % puntos para graficar
    P1 = T06(1:3,4)' + PD; % punto tobillo
    P2 = TpieD(1:3,4)' + PD; % punto punta pie
    P3 = T04(1:3,4)' + PD; % punto rodilla
    P4 = T03(1:3,4)' + PD; % punto cadera
    
    pts(:,:,1) = [P2(1), P1(1), P3(1), P4(1)]; % valores X
    pts(:,:,2) = [P2(2), P1(2), P3(2), P4(2)]; % valores Y
    pts(:,:,3) = [P2(3), P1(3), P3(3), P4(3)]; % valores Z
    
    T0D = [eye(3),PD';0 0 0 1];
    TFD = [eye(3),PD';0 0 0 1]*T06;
end

% FUNCION CON LA CINEMATICA DIRECTA DE LA PIERNA IZQUIERDA ----------------
function [pts, T0I, TFI] = FK_leftLeg(L,q)
    % signos de las q adaptados a bogo3
    % parametros del robot
    L1 = L(1); L3 = L(2); L4 = L(3);
    
    % valores articulares
    q7 = q(7); q8 = q(8); q9 = q(9); q10 = q(10); q11 = q(11); q12 = q(12);
    
    % cinematica directa de los puntos 
    T07 = [rotz(-q7),[0;0;0]; 0 0 0 1]; % cadera
    T78 = [rotx(-q8),[0;0;0]; 0 0 0 1]; % cadera
    T89 = [roty(-q9),[0;0;0]; 0 0 0 1]; % cadera
    T910 = [roty(-q10),[0;0;-L3]; 0 0 0 1]; % rodilla
    T1011 = [roty(q11),[0;0;-L4]; 0 0 0 1]; % tobillo 
    T1112 = [rotx(q12),[0;0;0]; 0 0 0 1]; % tobillo 

    T09 = T07*T78*T89;
    T010 = T09*T910;
    T012 = T010*T1011*T1112;
    TpieI = T012*[eye(3), [0.2;0;0]; 0 0 0 1];

    PI = [0,L1,L3+L3]; % para acomodar pata izquierda en dibujo
    
    % puntos para graficar
    P5 = T09(1:3,4)' + PI; % punto cadera
    P6 = T010(1:3,4)' + PI; % punto rodilla
    P7 = T012(1:3,4)' + PI; % punto tobillo
    P8 = TpieI(1:3,4)' + PI; % punto punta pie
    
    pts(:,:,1) = [P5(1), P6(1), P7(1), P8(1)]; % valores X
    pts(:,:,2) = [P5(2), P6(2), P7(2), P8(2)]; % valores Y
    pts(:,:,3) = [P5(3), P6(3), P7(3), P8(3)]; % valores Z
    
    T0I = [eye(3),PI';0 0 0 1];
    TFI = [eye(3),PI';0 0 0 1]*T012;
end 

% FUNCION CON LA CINEMATICA INVERSA DE LA PIERNA DERECHA ------------------
function q = IK_rightLeg(L,pts,alpha)
    
    % parametros del robot
    L1 = L(1); L3 = L(2); L4 = L(3);
    % puntos deseados en el plano
    Px_alpha = pts(1); Py_alpha = pts(2); Pz_alpha = pts(3);
    
    % nuevos puntos en el plano considerando la rotacion en z
    % q12
    q12 = alpha; c12 = cos(q12); s12 = sin(q12);
    Px = Px_alpha*c12 - Py_alpha*s12;
    Py = Px_alpha*s12 + Py_alpha*c12;
    Pz = Pz_alpha;

    % nota signos en q4, q3
    %q9
    c9 = (Px^2 + Py^2 + Pz^2 - L3^2 - L4^2) / (2*L3*L4);
    c9 = max(-1,min(c9,1));
    q9 = acos(c9); % + - Nota: debe ir positivo 
    s9 = sin(q9);
    
    % q11
    q11 = atan2(-Py, -Pz); % o atan2(Py, Pz)
    c11 = cos(q11); s11 = sin(q11);
    
    % q10
    q10 = atan2(-Px, -Pz*c11 -Py*s11) - atan2(L4*s9, L3 + L4*c9);
    s10 = sin(q10); c10 = cos(q10);
    q10 = atan2(s10, c10);
    
    % q8
    q8 = q9 + q10;

    % q7
    q7 = q11;
    
    % acomodados de cadera a pies
    q = [q12, q11, q10, q9, q8, q7];
end 

% FUNCION CON LA CINEMATICA INVERSA DE LA PIERNA IZQUIERDA ----------------
function q = IK_leftLeg(L,pts,alpha)
    
    % parametros del robot
    L1 = L(1); L3 = L(2); L4 = L(3);
    % puntos deseados en el plano
    Px_alpha = pts(1); Py_alpha = pts(2); Pz_alpha = pts(3);
    
    % nuevos puntos en el plano considerando la rotacion en z
    % q6
    q6 = alpha; c6 = cos(q6); s6 = sin(q6);
    Px = Px_alpha*c6 - Px_alpha*s6;
    Py = Py_alpha*s6 + Py_alpha*c6;
    Pz = Pz_alpha;

    %q3
    c3 = (Px^2 + Py^2 + Pz^2 - L3^2 - L4^2) / (2*L3*L4);
    c3 = max(-1,min(c3,1));
    q3 = - acos(c3); % + -
    s3 = sin(q3);

    % q5
    q5 = atan2(-Py, -Pz); % o atan2(-Py, -Pz)
    c5 = cos(q5); s5 = sin(q5);

    % q4
    q4 = atan2(Px, -Pz*c5 -Py*s5) - atan2(L4*s3, L3 + L4*c3);
    s4 = sin(q4); c4 = cos(q4);
    q4 = atan2(s4, c4);

    
    % q2
    q2 = q3 + q4;

    % q1
    q1 = q5;
    
    q = [q6, q5, q4, q3, q2, q1];
end 

% FUNCION QUE REALIZA LA TRAYECTORIA EN ESPACIO DE TRABAJO DEL LAS PIERNAS
function pts = trayectory(xlim, ylim, piso, radio, T, dt, UD)
    pts = []; t0 = 0;
    Nstep = (T-t0)/dt;
    
    dx = (2*xlim)/Nstep;
    dy = ylim/Nstep;
    
    if UD == 1 % significa que el pie se levanta del piso
        Px = -xlim-dx; Py = dy;
        for t = t0:dt:T
            %w=inter5(0,1,t0,tf,t);

            %a0 = 0; af = 4*pi; a =(1-w)*a0 + w*af;
            Px = Px + dx;
            %Py = Py + dy;
            Py = 0;
            Pz = -piso +radio*(2/T)*sqrt(t.*(T-t));

            pts = [pts;Px, Py, Pz];
        end 
    else
        Px = xlim+dx; Py = dy;
        for t = t0:dt:T
            %w=inter5(0,1,t0,tf,t);

            %a0 = 0; af = 4*pi; a =(1-w)*a0 + w*af;
            Px = Px - dx;
            %Py = Py + dy;
            Py = 0;
            Pz = -piso;

            pts = [pts;Px, Py, Pz];
        end 
    end
    
end

% FUNCION QUE REALIZA LA INTERPOLACION DE 5TO ORDEN -----------------------
function q = inter5(q0,qf,t0,tf,t)
    Mat1 = [ 1 t0 t0^2 t0^3 t0^4 t0^5; 1 tf tf^2 tf^3 tf^4 tf^5; ...
             0 1 2*t0 3*t0^2 4*t0^3 5*t0^4; 0 1 2*tf 3*tf^2 4*tf^3 5*tf^4;...
             0 0 2 6*t0 12*t0^2 20*t0^3; 0 0 2 6*tf 12*tf^2 20*tf^3];
    Mat2 = [q0;qf;0;0;0;0];

    a = inv(Mat1)*Mat2; q = a(1) + a(2)*t + a(3)*t^2 + a(4)*t^3 + a(5)*t^4 + a(6)*t^5;
end

