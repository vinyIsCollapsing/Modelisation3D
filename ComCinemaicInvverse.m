% Ad = [Sdx Ndx Adx; Sdy Ndy Ady; Sdz Ndz Adz];
% At = [Sex Nex Aex: Sey Ney Aey; Sez Nez Aez];
% RuTheta = Ad * At';

% u = [RuTheta(3,2) - RuTheta(2,3);
%      RuTheta(1,3) - RuTheta(3,1);
%      RuTheta(2,1) - RuTheta(1,2)];
% epsilon = 0.5 * theta * u;

%X = [1 3 8 9 13 19 25]';
%Y = [40 34 28 22 19 15 11]';

%H = [X ones(size(X))];

%theta = pinv(H) * Y;

%alpha = theta(1); % inclinacao
%b = theta(2); % interceptacao

%disp(['Alpha ', num2str(alpha)]);
%disp(['B ', num2str(b)]);

%xPlot = linspace(min(X), max(X), 100);
%yPlot = alpha * xPlot + b;

%figure;
%plot(X, Y, 'o');
%hold on;
%plot(xPlot, yPlot, '-r');
%xlabel('X');
%ylabel('Y');
%grid on;

function solution = sistemeDetermine(A, b)
    [m, n] = size(A);

    augmented = [A b];
    for k = 1 : n-1
        for i = k+1 : n
            factor = augmented(i, k)/augmented(k, k);
            augmented(i,:) = augmented(i,:) - factor*augmented(k,:);
        end;
    end;

    x = zeros(n,1);
    x(n) = augmented(n, n+1)/ augmented(n,n);
    for i = n-1 : -1 : 1
        soma = 0;
        for j = i + 1 : n
            soma = soma + augmented(i,j)*x(j);
        end;
        x(i) = (augmented(i, n+1) - soma)/augmented(i,i);
    end;

    solution = x;
end;

A = [0.5 -1 3 0.1; 2 1 1.5 2];
b = [12; 25];
x = sistemeDetermine(A, b);