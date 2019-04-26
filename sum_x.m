function y=sum_x(u)
persistent sx
if isempty(sx)
    sx=0;
end
sx=sx+u+plus1(u);
y=sx;
end
function y=plus1(u)
persistent sx
sx=0;
y=1;
end