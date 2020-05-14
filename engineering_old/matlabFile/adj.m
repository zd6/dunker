function Adj = adj(T)
R = T(1:end-1, 1:end-1);
p = T(1:end-1, end);
p_m = [0 -p(3) p(2); p(3) 0 -p(1); -p(2) p(1) 0];
Adj = [R zeros(3); p_m*R R];
end