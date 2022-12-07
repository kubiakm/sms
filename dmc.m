N = 17;
N_u = 3;
D = 80;
lambda = 100;


s=odpskok;

M = zeros(N, N_u);
for i=1:N_u
    for j=1:N
        if j - i + 1 >= 1
            M(j, i) = s(j - i + 1);
        else
            M(j, i) = 0;
        end
    end
end

L = lambda * eye(N_u);
K = (M.' * M + L) \ M.';

M_p = zeros(N, D-1);
for i=1:D-1
    for j=1:N
        M_p(j, i) = s(i + j) - s(i);
    end
end
