T_step = [0:0.05:5];

r_pos = [];
th_pos = [];
t = 0;

while t < 5
    th = pi*t;
    r = exp(0.2*th);
    th_pos = [th_pos, th];
    r_pos = [r_pos, r];
    
    t = t+.05;
end

figure (1)
polarplot(th_pos, r_pos, 'b-o')