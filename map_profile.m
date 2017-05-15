function [particles_needed, sampling_strategy, convergence_threshold] = map_profile(map)
%find the area of map
map=abs(map);
x=map(:,1);
y=map(:,2);
% Get the number of vertices
n = length(x);
% Initialize the area
p_area = 0;
% Apply the formula
for i = 1 : n-1
    p_area = p_area + (x(i) + x(i+1)) * (y(i) - y(i+1));
end
p_area = abs(p_area)/2;

%find the area of its convex hull
hulleo=map(convhull(map),:);
x=hulleo(:,1);
y=hulleo(:,2);
% Get the number of vertices
n = length(x);
% Initialize the area
hulleo_area = 0;
% Apply the formula
for i = 1 : n-1
    hulleo_area = hulleo_area + (x(i) + x(i+1)) * (y(i) - y(i+1));
end
hulleo_area = abs(hulleo_area)/2;

%next calculate the convexity given as
%convexity=area(convex_hull(polygon))-area(pol)
%       --------------------------------------
%             area(convex_hull(polygon))
convexity=(hulleo_area-p_area)/hulleo_area;

%Well, this is a crude way of finding the area, but works
limsMin = min(map); % minimum limits of the map
limsMax = max(map); % maximum limits of the map
res=7;
dims = limsMax-limsMin; %dimension of the map

dims=dims/res;
if (convexity>=0.2)
    %only do systematic resampling if the map is fairly convex. i.e.
    %complex enough
    particles_needed=round(dims(1)*dims(2));
    sampling_strategy=2;
    convergence_threshold=1;
else
    %Otherwise do simplified PR resampling
    particles_needed=round(dims(1)*dims(2)*2);
    sampling_strategy=1;
    convergence_threshold=1;
end
end
