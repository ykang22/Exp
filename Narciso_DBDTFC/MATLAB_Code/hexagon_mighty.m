t = 0:0.0001:10;
l = 2/3;
x = 2*l*(abs(sawtooth(t))-0.5);

y = min(l*sqrt(3)/2,max(-l*sqrt(3)/2,2*l*sqrt(3)*(abs(sawtooth(t-pi/2))...
    -0.5)));

for i = 1:length(t)
    abc(:,i) = [1, 0, 1; -0.5, sqrt(3)/2, 1; -0.5, -sqrt(3)/2, 1]*[x(i);...
        y(i); 0];
end

xc = l*sqrt(3)/2*cos(2*pi*t);
yc = l*sqrt(3)/2*sin(2*pi*t);

figure(1),plot(xc,yc,'LineWidth',1.5), hold on,...
    plot(x,y,'LineWidth',1.5),axis equal;
figure(1),plot(linspace(0.3,0.3,100),linspace(-2/3,2/3,100),'LineWidth',2);
figure(1),plot(linspace(0,0.3,100),-0.5859/0.3*...
    linspace(0,0.3,100),'LineWidth',2),grid on;
figure(1),plot(0.3,-0.5859,'*','MarkerSize',2);

figure(2), plot(t,x), hold on, plot(t,y);

figure(3),plot(t,abc(1,:)),hold on,plot(t,abc(2,:)),plot(t,abc(3,:));

figure(4), plot(t,sqrt(x.^2+y.^2));