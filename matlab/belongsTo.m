function [element,vertices]=belongsTo(fn,elem,nodes,p,file)
% eval(file);
if file
    eval(fn)
end

[f,c]=size(elem);
for i=1:f
    for j=1:c
        vertices(j,:)=nodes(elem(i,j),:);
    end
    [~,inside]=baryCoord(vertices,p);
    if inside==1
        break
    end
end
element=i;
end

function [alphas, isInside]=baryCoord(vertices, point)
isInside=1;
A=[ones(3,1),vertices];
B=eye(3);
alphas = zeros(1,3);
for i=1:3
    c=A\B(:,i);
    Psi=@(x,y)c(1)+c(2)*x+c(3)*y;
    alphas(i)=Psi(point(1),point(2));
    if alphas(i)<0
        isInside=0;
    end
end
end