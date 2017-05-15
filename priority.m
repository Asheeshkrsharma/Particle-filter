function [ indx ] = priority( weights, num, trim )
%Normalize weights
w = weights;
percentage=round(num*trim);
Index=zeros(percentage,1);
%Fin top 'trim' percentage of weights
for j = 1:percentage
   [a, Index(j)] = max(w);
   w(Index(j)) = -inf;
end
indx=sort(Index);
end