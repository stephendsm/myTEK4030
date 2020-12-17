t = 0:0.05:10;
A = [0 1 0;0 0 1;-10 -17 -8];
B = [0;0;0];
C = [1 0 0];
D = [0];
y = initial(A,B,C,D,[2;1;0.5],t);
plot(t,y)
grid
title('Response to Initial Condition')
xlabel('t(sec)')
ylabel('Output y')