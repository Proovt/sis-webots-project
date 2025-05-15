dt = .01;

T = 1000;

t = (0:(T - 1))';
a = ones(T, 1);
v = zeros(T, 1);
x1 = zeros(T, 1);
x2div2 = zeros(T, 1);
x2 = zeros(T, 1);
x2witht = zeros(T, 1);

acum = cumsum(a);

x2witht = a(1) * (t*dt).^2 / 2;

for i = 2:T
    v(i) = v(i-1) + a(i-1) * dt;
    x1(i) = x1(i-1) + v(i) * dt;

    %x2div2(i) = x2div2(i-1) + (acum(i-1) * dt^2 / 2); % <-- wrong we need
    %to plug in t not dt
    %x2witht = x2(i-1) + (acum(i-1) * t(i-1)^2 / 2);
    x2(i) = x2(i-1) + (acum(i-1) * dt^2);
end
%%
hold on
plot(t, x1, DisplayName="V integration")
%plot(t, x2, DisplayName="A integration")
plot(t, x2witht, DisplayName="A / 2 integration")
legend
%%
disp(sum(abs(x2 - x1)))
disp(sum(abs(x2div2 - x1)))
disp(sum(abs(x2witht - x1)))