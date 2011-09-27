function [h]=nmcolor(x,e,idx,name);

argc = nargin;
dim=size(e);
l=max(dim); n=min(dim);

if (argc==1)
   e=x; [l n]=size(e); x=1:l;
   argc = argc+1;
end;
if (argc==2)
    idx=-1;
    argc = argc+1;
end;
if (argc==3)
   name = 'Plot ';
   argc = argc+1;
end;

if (idx==-1)
    idx=1:n;
end

nb_task=n;

for i=find(idx>n)
    warning(sprintf('Plot index %d is not valid.',i));
end    
idx=idx(find(idx<=n));

color = [     0.75 0.75 0;    0 0 0.4;    1 0  0;    0 .5 0    ];
lwb = [ .5 2 .5 2 .5 2 .5 2 .5 2 .5 2];
lwb = [ .1 8 .5 2 .5 2 .5 2 .5 2 .5 2];

nbcolor=length(color(:,1));
color=[color;color];color=[color;color];color=[color;color];color=[color;color];
lw=[]; for i=lwb     lw=[lw;ones(nbcolor,1)*i]; end

sty = ['- ';'- ';'- ';'- '];
sty = [sty;sty];sty = [sty;sty];sty = [sty;sty];sty = [sty;sty];

% gcf; 
h=[];
seq=0;
for i=idx
    idxs=mod(seq,length(sty))+1;
    idxw=mod(seq,length(lw))+1;
    idxc=mod(seq,length(color))+1;
    hi=plot(x, e(:,i), 'color',color(idxc,:),'LineWidth',lw(idxw),'LineStyle',sty(idxs,:));
    h=[h hi];
    hold on;
    seq=seq+1;
end;

%if separation
%   ax =axis;
%   hold on;
%   st=stem(task*ax(3), '.k');
%   set(st,'MarkerSize',1);
%   st=stem(task*ax(4), '.k');
%   set(st,'MarkerSize',1);
%   hold off;
%   axis(ax);
%end;

xlabel('Iterations');
ylabel('Values');
legende=[];

for i=idx
    legende = [legende; sprintf('%s % 4d',name,i)];
end;
%legend(legende);
