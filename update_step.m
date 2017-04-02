function G2 = update_step(G)
[m,n] = size(G);
G2 = zeros(m,n);
for ii = 1:m
    for jj = 1:n
        if G(ii,jj) == 1
            if G(ii,jj+1) == 0
                G2(ii,jj+1) = 1;
            elseif G(ii,jj+1) == 1
                G2(ii,jj) = 1;
            end
        end
    end
end
