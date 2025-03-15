function [Irc,Irc_sink] = Communicable_Gpt(P_sink,G,sz,Rc)
%clc; clear all; close all;
% %% user defined input
% Rs = 1;%sqrt(2); %Sensing Radius
% Rc = 2;%2*Rs; %Communication Radius
% %A = 0.6 ; B = 0.4; %Weights
% %% Network Grid
% x = 1:10; 
% y = 1:10;
% b=x; a=y; %making it (row,column) grid instead of (x,y)
% [X,Y] = meshgrid(a,b);
% Nx=length(x);
% Ny=length(y);
% sz=[Nx,Ny];
% TG = [Y(:),X(:)];
% sink = [6,6]; % 2D position of sink
% P_sink = sub2ind(sz,sink(1),sink(2)); % Index position of sink
% Cmax = Nx*Ny-1;
% Obs = [2,2;3,2;4,2;2,3;3,3;4,3;2,4;3,4;4,4];%obstacle
% G_index = ~ismember(TG,Obs,'rows');
% G = [TG(G_index,1),TG(G_index,2)];
% display(G); %target set
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[row,col] = ind2sub(sz,P_sink);
sink=[row,col];
Crc = []; Irc = []; % communication matrix
for p = 1: length(G)
    s1=[]; s2=[]; s22=[];
    Gpt = G(p,:);
for q = 1: length(G)
    v = (G(p,1)-G(q,1)).^2 + (G(p,2)-G(q,2)).^2 ;
    if v <= (Rc).^2 
      s1 = [G(q,1) G(q,2)];
      s2 = [s2 ; s1];
      s22 = unique(s2,'rows');
    end
end
Crc{p} = [Gpt; s22];
i = []; 
for k=1:length(s22)
    i1=0;
i1 = sub2ind(sz,s22(k,1),s22(k,2));
%i1 =s22(k,1) + (s22(k,2)-1)*Nx;
i = [i;i1];
end
Icom{p} = sort(i);
Irc{p} = setdiff(Icom{p}, P_sink);
end
L=find(ismember(G,sink,'rows')); %sink position in updated network
Irc_sink = Irc{L};
end
