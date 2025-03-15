function [L,Irs,Irs_sink] = Sensing_Gpt(P_sink,G,sz,Rs)
% clc; clear all; close all;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[row,col] = ind2sub(sz,P_sink);
sink=[row,col];
Crs = []; Irs = []; % sensing matrix
for p = 1: length(G)
    s1=[]; s2=[]; s22=[];
    Gpt = G(p,:);
for q = 1: length(G)
    v = (G(p,1)-G(q,1)).^2 + (G(p,2)-G(q,2)).^2 ;
    if v <= (Rs).^2 
      s1 = [G(q,1) G(q,2)];
      s2 = [s2 ; s1];
      s22 = unique(s2,'rows');
    end
end
Crs{p} = [Gpt; s22];
i = [];
for k=1:length(s22)
    i1=0;
i1 = sub2ind(sz,s22(k,1),s22(k,2));
%i1 =s22(k,1) + (s22(k,2)-1)*Nx;
i = [i;i1];
end
Icov{p} = sort(i);
Irs{p} = setdiff(Icov{p}, P_sink);
end
%Irs_sink = Irs{P_sink};
L=find(ismember(G,sink,'rows')); %sink position in updated network
Irs_sink = Irs{L};
end