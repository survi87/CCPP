clc; clear all; close all;
%tic;
%% user defined input
Rs = sqrt(2); %Sensing Radius
Rc = 2*Rs; %Communication Radius
cr = 1;
%% Network Grid
x = 1:12; 
y = 1:12;
b=x; a=y; %making it (row,column) grid instead of (x,y)
[X,Y] = meshgrid(a,b);
Nx=length(x);
Ny=length(y);
TG = [Y(:),X(:)];
sz=[Nx,Ny];
P_sink = 79; % Index position of sink
[row,col] = ind2sub(sz,P_sink);
sink = [row,col]; % 2D location of sink
Obs = [];
%Obs = [2,2;2,3;3,2;3,3];%obstacle
if ~isempty(Obs)
G_index = ~ismember(TG,Obs,'rows');
G = [TG(G_index,1),TG(G_index,2)];
G1 = sub2ind(sz,G(:,1),G(:,2));
Cmax = Nx*Ny-length(Obs)-1;
else
G = TG;
G1 = sub2ind(sz,TG(:,1),TG(:,2));
Cmax = Nx*Ny-1;
end
display(G1); %target set
%% Communicable and Sensing grid points sets
[Irc,Irc_sink] = Communicable_Gpt(P_sink,G,sz,Rc); % communication matrix
[L,Irs,Irs_sink] = Sensing_Gpt(P_sink,G,sz,Rs); % sensing matrix
N=3; %(number of UAVs)
T=10; %(max no. of time steps)
%% Variables
%Ci = zeros(Nx*Ny,1);     % overall coverage
%Ct_i,n = zeros(N*T*Nx*Ny,1); % individual UAVs' coverage
%Zt_i,n = zeros(N*T*Nx*Ny,1); % optimizing variable
%S is our variable vector with Nx*Ny number of Ci, T*N*Nx*Ny no. of Ct_i,n and T*N*L*Nx*Ny no. of Zt_i,n variables respectively
% S = [Ci; Ct_i,n (n=1:N,t=1:T); Zt_i,n (n= 1:N,t=1:T)];
% mobile(1:N) = P_sink;
% mob_node = [mobile]';
% display(mob_node);
%% Objective function
ff = zeros((1+ 2*N*T)*Nx*Ny ,1);
%ff(((1+T*N)*Nx*Ny + 1) : end)=1;
for t=1:T
    for n=1:N
    ff(G1+(1 + T*N)*Nx*Ny + ((t-1)*N*Nx*Ny)+(n-1)*Nx*Ny) = 1 ;
    ff((1 + T*N)*Nx*Ny + ((t-1)*N*Nx*Ny)+(n-1)*Nx*Ny + P_sink) = 0 ;
    end
end
%% Initial Position Constraint
% R = zeros(N, (1+ 2*N*T)*Nx*Ny);
% for l=1:N
% r1= zeros((Nx*Ny+ 2*N*T*Nx*Ny) ,1);
% r1(((N*T+1)*Nx*Ny)+ (l-1)*Nx*Ny + mob_node(l,1))=1;
% R(l,:)=r1;
% end
%% Position Costraint
%%1(a)%%one-UAV-at-single-pt
F=zeros(N*T,(1+ 2*N*T)*Nx*Ny);
k=1;
for t = 1 : T
    for n = 1:N
         f1= zeros((1+ 2*N*T)*Nx*Ny ,1);
    
    f1(G1+ (1 + T*N)*Nx*Ny + ((t-1)*N*Nx*Ny)+(n-1)*Nx*Ny) = 1 ;
    f1((1 + T*N)*Nx*Ny + ((t-1)*N*Nx*Ny)+(n-1)*Nx*Ny + P_sink) = 0 ;
    F(k,:) = f1; 
    k=k+1;
    end
end
%
%%1(b)%%single-UAV-at-one-pt
E=zeros(T*length(G),(1+ 2*N*T)*Nx*Ny);
k=1;
for t = 1 : T
    for p = 1:length(G)
         f2= zeros((1+ 2*N*T)*Nx*Ny ,1);
         if p~=L
         for n=1:N    
         f2((1 + T*N)*Nx*Ny + ((t-1)*N*Nx*Ny)+(n-1)*Nx*Ny + G1(p)) = 1 ;
         end
         end
         E(k,:) = f2; 
         k=k+1;
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% Connectivity Constraint
%%2a%%Sink
g = zeros(T, (1+ 2*N*T)*Nx*Ny);
k=1;
for t =1:T
    g1 = zeros((1+ 2*N*T)*Nx*Ny , 1);
    for n =1:N
    for i = 1:length(Irc_sink)
        g1((1 + T*N)*Nx*Ny + ((t-1)*N*Nx*Ny)+(n-1)*Nx*Ny + Irc_sink(i)) = -1;
          
    end
    end
    g(k,:) = g1;
    k=k+1;
end
%%2b%%Inter-UAV
H = zeros(T*(N-1)*length(G), (1+ 2*N*T)*Nx*Ny);
k=1;
for t =1:T
    for n =2:N
    
          for p = 1:length(G)
            h11 = zeros((1+2*N*T)*Nx*Ny ,1);
%             if p == L
%             h11(Nx*Ny + T*N*Nx*Ny+ ((t-1)*N*Nx*Ny)+ (n-1)*Nx*Ny + P_sink) = 0;
%             end
            if p ~= L
            h11(Nx*Ny + T*N*Nx*Ny+ ((t-1)*N*Nx*Ny)+ (n-1)*Nx*Ny + G1(p)) = 1 ;
            for i = 1:length(Irc{p})
                h11(Nx*Ny +(N*T*Nx*Ny)+((t-1)*N*Nx*Ny)+(n-2)*Nx*Ny + Irc{p}(i)) = -1;
            end
            end
             H(k,:)=h11;
             k=k+1;
          end
     end        
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% mobility Constraint
I = zeros((T-1)*N*length(G), (1+2*N*T)*Nx*Ny);
k=1;
for t =1:(T-1)
    for n =1:N
        for p = 1:length(G)
          h12 = zeros((1+2*N*T)*Nx*Ny ,1);
%           if p == L
%           h12(G1+ Nx*Ny +(N*T*Nx*Ny)+((t)*N*Nx*Ny)+(n-1)*Nx*Ny + P_sink) = 0;
%           end
          if p ~= L
          h12(Nx*Ny +(N*T*Nx*Ny)+((t)*N*Nx*Ny)+(n-1)*Nx*Ny + G1(p)) = 1;
          for i = 1:length(Irc{p})
          h12(Nx*Ny +(N*T*Nx*Ny)+((t-1)*N*Nx*Ny)+(n-1)*Nx*Ny + Irc{p}(i)) = -1;
          end
        end  
           
          I(k,:)=h12;
          k=k+1;
          
        end     
    end
end           
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% cell coverage constraint variables
%4(a)
K=zeros(T*N*length(G), (1+2*N*T)*Nx*Ny);
k=1;
for t =1:T
    for n =1:N  
      for p = 1:length(G)
          h1 = zeros((1+2*N*T)*Nx*Ny ,1);
%           if p == L
%           h1(Nx*Ny + ((t-1)*N*Nx*Ny)+ (n-1)*Nx*Ny + P_sink) = 0 ; 
%           end
          if p ~= L
          h1(Nx*Ny + ((t-1)*N*Nx*Ny)+ (n-1)*Nx*Ny + G1(p)) = 1 ; 
          for i = 1:length(Irs{p})
          h1(Nx*Ny +(N*T*Nx*Ny)+((t-1)*N*Nx*Ny)+ (n-1)*Nx*Ny + Irs{p}(i)) = -1;
          end
       end   
          
      K(k,:)=h1;
      k=k+1;
      end
    end
end        

%4(b)
J=zeros(N*T*length(G), (1+2*N*T)*Nx*Ny);
k=1;
for t = 1 : T
    for n=1:N
    for q = 1:length(G)
    h2 = zeros((1+2*N*T)*Nx*Ny ,1);
         h2(G1(q)) = -1 ;  h2(G1(L)) = 0;  
         h2(Nx*Ny + ((t-1)*N*Nx*Ny)+ (n-1)*Nx*Ny + G1(q) ) = 1;
         h2(Nx*Ny + ((t-1)*N*Nx*Ny)+ (n-1)*Nx*Ny + G1(L) ) = 0;

    J(k,:)=h2;
    k=k+1;
    end 
    end
end

%4(c)    
Q = zeros(length(G), (1+2*N*T)*Nx*Ny);
k=1;
for q = 1:length(G)   
     h3 = zeros((1+2*N*T)*Nx*Ny ,1);
          
     h3(G1(q)) = 1 ; h3(G1(L)) = 0;
     for t = 1: T
     for n=1:N
     h3(Nx*Ny +((t-1)*N*Nx*Ny)+ (n-1)*Nx*Ny + G1(q)) = -1;
     h3(Nx*Ny + ((t-1)*N*Nx*Ny)+ (n-1)*Nx*Ny + G1(L) ) = 0;
     end 
     end
     Q(k, :)=h3;
     k=k+1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%Desired Coverage Constraint
W = zeros((1+2*N*T)*Nx*Ny ,  1);
%W(G1) = -1; W(P_sink) = 0;
for l= 1:length(G)
  W(G1(l),:)= -1;
  W(G1(L))= 0;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% optimization problem
% minimize ff'.S
% subject to   z_k is integer (binary)
%              E.S <= 1
%              F.S = 1
%              g.S <= -1
%              H.S <= 0
%              I.S <= 0
%              K.S = 0
%              J.S <= 0
%              Q.S <= 0
%              W'.S <= (-1).cr.Cmax
%              0<= value at G <= 1

Z1 = ff ; %objective function
%A = [F;g;H;I;J;Q;W']; %inequality constraint LHS
%b = [ones(size(F,1),1); (-1)*ones(size(g,1),1); zeros(size(H,1),1); zeros(size(I,1),1); zeros(size(J,1),1); zeros(size(Q,1),1); (-1)*cr*Cmax];%inequality constraint RHS
A = [E;F;g;H;I;J;Q;W']; %inequality constraint LHS
b = [ones(size(E,1),1); ones(size(F,1),1); (-1)*ones(size(g,1),1); zeros(size(H,1),1); zeros(size(I,1),1); zeros(size(J,1),1); zeros(size(Q,1),1); (-1)*cr*Cmax];%inequality constraint RHS
Aineq = A;
bineq = b;
Aeq = [K];              %equality constraint LHS
beq = [zeros(size(K,1),1)];  %equality constraint RHS
%Aeq = [R; K];              %equality constraint LHS
%beq = [ones(size(R,1),1);zeros(size(K,1),1)];  %equality constraint RHS
%% Set the bounds
lb1= zeros(Nx*Ny,1);
lb2= zeros((N*T*Nx*Ny),1);
lb3 = zeros((N*T*Nx*Ny) ,1);
lb=[lb1;lb2;lb3];          %lower bound
ub1= ones(Nx*Ny,1);
ub2= ones((N*T*Nx*Ny),1);
ub3 = ones((N*T*Nx*Ny) ,1);
ub=[ub1;ub2;ub3];          %upper bound 
%% declaring integer variables
%intcon = (1:T*N*Nx*Ny)' + ((1+T*N)*Nx*Ny);


for p = 1 : (1+T*N)*Nx*Ny
     ctype(p) = 'C';
 end
for p = (1:T*N*Nx*Ny) + (Nx*Ny+(T*N*Nx*Ny));
     ctype(p) = 'B';
end
%% calling solver

options = cplexoptimset('cplex');
options.Display = 'on';
options.mip.tolerances.mipgap =  0;                                                                                                                                                           
options.timelimit= 18000; % 24hours
options.mip.strategy.search=1;
tic;
[S,fval] = cplexmilp(Z1, Aineq,bineq,Aeq,beq,[],[],[],lb,ub,ctype,[],options);

t_matlab = toc;
cov_time = t_matlab;
%% calling solver
% execution time limit = 36hrs = 129600 seconds
% options = optimoptions('intlinprog','BranchRule', 'maxpscost', 'Heuristics', 'rss',...
%      'ObjectiveImprovementThreshold', 1e-4, 'OutputFcn', 'savemilpsolutions','MaxTime', 129600);
% tic; 
% [S , fval , exitflag, output] = intlinprog(Z1, intcon, A, b, Aeq, beq, lb, ub, options);
% t_matlab = toc;

%% result calculations
% dimension of S = Nx*Ny (c(ij)) + T*N*Nx*Ny (c_n^t(ij)) + T*N*L*Nx*Ny (z_n^l,t(ij)) 
ss= round(S,1);
c = find(ss);
path_loc = [];
path_loc = c(find(c>(1+T*N)*Nx*Ny));
U = ss(path_loc);
V = [path_loc, U];

%% route Tracing

for n=1:N
 P1=[];
for t = 1:T
    for i=1:length(path_loc)
    Rmin = (1+ T*N)*Nx*Ny + (t-1)*N*Nx*Ny + (n-1)*Nx*Ny ;
    Rmax = (1+ T*N)*Nx*Ny + Nx*Ny + (t-1)*N*Nx*Ny + (n-1)*Nx*Ny ;
    if ((Rmax <= (1+ T*N + T*N)*Nx*Ny) && path_loc(i)>Rmin && path_loc(i)<=Rmax)
    c1=path_loc(i);
   
    c2 = c1 - ((1+T*N)*Nx*Ny + (t-1)*N*Nx*Ny +  (n-1)*Nx*Ny) ;
    P1= [P1,c2];
    end
    end
end

cov_path(n,1:length(P1))= P1;
end
%movements = length(find(unique(cov_path)));
%%
%Cost=distance=path traversing time
cost=0;
for r=1:N
    sz = [Nx,Ny];
    p1=cov_path(r,:);
    p2=find(p1);
    ind = p1(p2) ;
    if length(ind)==1
     break;
    else
    [row, col] = ind2sub(sz,ind);
    path=[row;col]';
    end
    for i=1:(length(path)-1)
        
        dist=sqrt((path(i+1,1)-path(i,1))^2 + (path(i+1,2)-path(i,2))^2);
        cost=cost+dist;
    end
end
path_cost=cost;
display(path_cost);
