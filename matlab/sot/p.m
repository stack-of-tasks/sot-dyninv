function [e t] = p (filename,idx,dirname);

argc = nargin;

if (argc==0)
   filename='pos';
   argc = argc+1;
end;
if (argc==1)
   idx=-1;
   argc = argc+1;
end;
if (argc==2)
   dirname=ddname;
   argc = argc+1;
end;


name=sprintf ('%s/%s.dat',dirname,filename);
e=load(name);
%size(e);
t=e(:,1);
e=e(:,2:end);
nmcolor(t,e,idx,'P ');

