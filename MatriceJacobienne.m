q =[0.143019, -0.109465, -0.011994, -1.1788, -0.154233, 0.93555, 0.264868];
p = [-0.3668886207137053, -0.03791609269591508, 0.8634137719059174];
%function J = MatriceJacobinne(q,p0)
    %l’algorithme calculant la matrice Jacobienne géométrique définie par les paramètres DH modifiés
    %q l'anlge
    %p0 coordoner du point pour la Jacobienne

    %Paramètres DH fixes
    a = [0 -pi/2 0 -pi/2 pi/2 -pi/2];
    d = [0 0 265.69 30 0 123];  %décalé du crayon
    r = [159 0 0 258 0 0];
    AAaux = [0; -1.4576453; -0.898549163; 0; 0; -pi];  %décalé du crayon
    q = q+AAaux;
    
    T=zeros(4,4); l=zeros(3,6); z=zeros(3,6);
    Tp =eye(4);
    J=zeros(6,6);
    for i=1:6
        cq=cos(q(i)); sq=sin(q(i)); db=d(i); sa=sin(a(i)); ca=cos(a(i)); rb=r(i);
        T=  Tp*  [ cq     -sq    0.0    db;
            ca*sq  ca*cq   -sa   -rb*sa;
            sa*sq  sa*cq    ca    rb*ca;
            0.0    0.0     0.0    1.0 ];

        l(:,i)=p0(:)-T(1:3,4);
        z(:,i)=T(1:3,3);
        J(1:3,i)=cross(z(:,i),l(:,i));
        J(4:6,i)=z(:,i);
        Tp=T;
    end
%end


    %
    % %antiga matriz de tranlacao n-1....n
    % % T01 = MatriceT (0,0,Q(1),159)
    % % T12 = MatriceT (-pi/2,0,Q(2)-1.4576453,0);
    % % T23 = MatriceT (0,265.69,Q(3)-0.898549163,0);
    % % T34 = MatriceT (-pi/2,30,Q(4),258);
    % % T45 = MatriceT (pi/2,0,Q(5),0);
    % % T56 = MatriceT (-pi/2,0,Q(6),0);
    % % T = [T01;T12;T23;T34;T45;T56];
    % %antigo  Tableau de matrice translation 01...0n
    % % R1=T01;
    % % R2= T01*T12;
    % % R3= T01*T12*T23;
    % % R4= T01*T12*T23*T34;
    % % R5 = T01*T12*T23*T34*T45;
    % % R6 = T01*T12*T23*T34*T45*T56;
    % %R2=R1*T12; R3=R2*T23; R4=R3*T34; R5=R4*T45; R6=R5*T56; R = [R1 R2 R3 R4 R5
    % %R6];
    %
    %
    %
    %
    % %aj c'est un vector 1:3
    % % J c'est un matrice 6:6
    % % OP c'est un vector 1:3
    %
    % % Demander qui est R du tableau 36 et  qui c'est OP.
    %
    %
    %
    % %taille = size(R);
    %     for j = 1: 6
    %         for i = 1:3
    %             a(i) = R(i,j*3);
    %             OP(i) = R(i,24)-R(i,j*4);
    %         end
    %             aOP = cross(a,OP);
    %             J(:,j) = [ aOP  a]';
    %
    % %               J(1:3,j)=cross(a(j),OP(j));
    % %             J(2,j)=a(j);
    %     end
    % %     for j=1:5
    % %         for i = 1:3
    % %         J(j,i)=cross(a(j,i),OP(j,i)
    % %         J(j+1,i)=a(j,i);
    % %     end
    %     %end
    % end
    %
