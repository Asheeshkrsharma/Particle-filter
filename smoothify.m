function [ out ] = smoothify( path )
bez = @(t,P) ...
  bsxfun(@times,(1-t).^3,P(1,:)) + ...
  bsxfun(@times,3*(1-t).^2.*t,P(2,:)) + ...
  bsxfun(@times,3*(1-t).^1.*t.^2,P(3,:)) + ...
  bsxfun(@times,t.^3,P(4,:));
  x=path(:,2);
  y=path(:,1);
  t = linspace(0,1,4)';
  for i=1:size(x,1)-4
       P=[x([i i+1 i+2 i+4]) y([i i+1 i+2 i+4])];
       X = bez(t,P);
       x([i i+1 i+2 i+4])=X(:,1);
       y([i i+1 i+2 i+4])=X(:,2);
  end
  out=[x y];
end

