function [ indx ] = resampling(w,num,technique)
%Do the resampling based on the technique suggested by map profiling.
if technique==1
    %Simplified PR
    N = length(w); %Number of particles
    T = 5*N; %The threshold beyond which we do not look ahead.
    M = length(w);
    w = w / sum(w);
    indx = zeros(1, N);
    h = 0;
    j = 0;
    while j < M
        j = j + 1;
        if w(j) > 1/T
            h = h + 1;
            A(h) = j;
        end
    end
    r = 1;
    i = 0;
    while i < N
        i = i + 1;
        indx(i) = A(r);
        r = mod(r,h) + 1;
    end
end
if technique==2
    %systematic
    N = length(w);
    M = length(w);
    w = w / sum(w);
    Q = cumsum(w);
    indx = zeros(1, N);
    T = linspace(0,1-1/N,N) + rand/N;

    i = 1;
    j = 1;
    while(i<=N && j<=M),
        while Q(j) < T(i)
            j = j + 1;
        end
        indx(i) = j;
        i = i + 1;
    end
end
end