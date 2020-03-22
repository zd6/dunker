function s = screw_matrix(o, q)
if norm(o) == 0
    s = [zeros(3) q'; zeros(1,4)];
else
    v = -cross(o,q)';
    o = o/norm(o);
    omega = [0 -o(3) o(2); o(3) 0 -o(1); -o(2) o(1) 0];
    s = [omega v; zeros(1, length(o) +1)];
end