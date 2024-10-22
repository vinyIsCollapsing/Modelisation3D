    function M = MGD(q)
    %MGD Renvoie la matrice de transformation de la cinématique directe
    %Parametros fixos de DH
    a = [0 -pi/2 0 -pi/2 pi/2 -pi/2];
    d = [0 0 265.69 30 0 123]; %décalé du crayon
    r = [159 0 0 258 0 0];
    varQ = [0; -1.4576453; -0.898549163; 0; 0; -pi]; %décalé du crayon
    q = q+varQ;
    %Enregistrer les matrices homogènes dans un tableau
    T = [cos(q(1)) -sin(q(1)) 0 d(1); cos(a(1))*sin(q(1)) cos(a(1))*cos(q(1)) -sin(a(1)) -r(1)*sin(a(1)); sin(a(1))*sin(q(1)) sin(a(1))*cos(q(1)) cos(a(1)) r(1)*cos(a(1)); 0 0 0 1];
    for i=2:6
        Tm = [cos(q(i)) -sin(q(i)) 0 d(i); cos(a(i))*sin(q(i)) cos(a(i))*cos(q(i)) -sin(a(i)) -r(i)*sin(a(i)); sin(a(i))*sin(q(i)) sin(a(i))*cos(q(i)) cos(a(i)) r(i)*cos(a(i)); 0 0 0 1];
        T= cat(2,T,Tm);
    end

    % Calcule la matrice MGD à partir des transformations homogènes
    R = T(:,1:4);
    for i = 1:5
        M = R(:,(i-1)*4+1:(i-1)*4+4)*T(:,i*4+1:i*4+4);
        R= cat(2,R,M);
    end
end

