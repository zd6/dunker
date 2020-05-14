function s = screw(o, p)
if norm(o) == 0
    v = p';
else
    o = o/norm(o);
    v = -cross(o,p)';
end
s = [o'; v];
end