p = 0
r = 1
T = 1
t = linspace(0,1,100)
plot(t,-p+r*(2/T)*sqrt(t.*(T-t)))
title('Pos pie en eje Z')
xlabel("t [s]")
ylabel("Z [cm]")