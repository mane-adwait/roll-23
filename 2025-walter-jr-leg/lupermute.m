% lupermute.m
% Christian Hubicki
% 2023-09-25 CE

clc; clear; close all;

A = magic(4);
A(1,1) = 0;
A(2,2) = 0;
A(3,3) = 0;
A(4,4) = 0;
A(1,1) = 0;
A(1,2) = 0;
A(1,3) = 0

s = size(A);
N = s(1);
P = eye(s);
% Check for zeros on the diagonal of A and swap rows on P to permute it;
for iter = 1:N

    if(A(iter,iter) == 0)
        done = 0;
        iter2 = iter + 1;
        while ~done
            if(A(iter2,iter) ~= 0) % We can swap!!!
                A([iter2 iter],:) = A([iter, iter2],:);
                P([iter2 iter],:) = P([iter, iter2],:);
                done = 1;
            end
        end
    end
end

A

