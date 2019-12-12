function y = lineEquation(pt1,pt2,x)
%LINEEQUATION Calculate y w.r.t x on line from pt1 to pt2.
y = (pt2(2)-pt1(2))/(pt2(2)-pt1(1))*(x-pt1(1))+pt1(2);
end

