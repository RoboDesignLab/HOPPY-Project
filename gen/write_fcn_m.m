function write_fcn(fcn_name,arguments,replace_list,list)

[pathstr, name, ext] = fileparts(fcn_name) ;
disp(['- writing ' name ext]);

% open file and write first line

fid = fopen(fcn_name,'w');
fprintf(fid,'function [');

for item = 1:1:size(list,1)
  currentname = list{item,2};
  if (item > 1) fprintf(fid,',');
  end
  fprintf(fid,'%s',currentname);
end

[path,name,ext] = fileparts(fcn_name);
fprintf(fid,'] = %s(',name);

for item = 1:1:size(arguments,2)
  % write arguments
  currentname = arguments{1,item};
  if (item > 1) fprintf(fid,',');
  end
  fprintf(fid,'%s',currentname);
end
fprintf(fid,')\n\n');
for ii=1:size(list,1)
    [n m] = size(list{ii,1});
    fprintf(fid,'%s = zeros(%d,%d);\n\n', list{ii,2}, n, m);
end

%fprintf(fid,'  model_params;\n\n');

for item = 1:1:size(list,1)
  % write variables to file
  currentvar = list{item,1};
  currentname = list{item,2};
  [n,m]=size(currentvar);
  for i=1:n
    for j=1:m
      Temp0=(currentvar(i,j));
      if (isnumeric(Temp0))
        Temp1 = num2str(Temp0);
      else
        Temp1=replace(char(Temp0),replace_list);
      end
      Temp2=['  ',currentname,'(',num2str(i),',',num2str(j),')=',Temp1,';'];
      fixlength_m(Temp2,'*+',100,'         ',fid);      fprintf(fid,'\n');
    end
  end
  fprintf(fid,'\n%s',' ');
end
status = fclose(fid);
%disp(' - done');

%=============================================
function str = replace(str,replace_list)
% process the longest strings first to keep from accidentally replacing substrings of longer strings
% perform a bubble sort on 'replace_list' according to decending length of the first element
% <del>
% if (size(replace_list,1) > 0)
%   for loop=1:1:size(replace_list,1)
%     for i=1:1:size(replace_list,1)-1
%       if (length(replace_list{i,1}) < length(replace_list{i+1,1}))
%         temp = replace_list(i,:);
%         replace_list(i,:) = replace_list(i+1,:);
%         replace_list(i+1,:) = temp;
%       end
%     end
%   end
%   for i=size(replace_list,1):-1:1
%     orig_str = char(replace_list(i,1));
%     new_str = char(replace_list(i,2));
%     str = strrep(str,orig_str,new_str);
%   end
% end
% </del>

% <add>
delimiters = ' +-/*^%&|!()' ;
rem = str ;
str = '' ;
while(length(rem))
    % Handle leading delimiters
    if(length(findstr(rem(1), delimiters))) % First character is a delimiter
        str = [str rem(1)] ;
        rem(1) = [] ;
        continue ;
    end
    [tok rem] = strtok(rem, delimiters) ;
    if(length(tok))
        found = false ;
        for i=1:size(replace_list,1)
            if(strcmp(tok, char(replace_list(i,1))))
                str = [str char(replace_list(i,2))] ; % Replace token with replacement string and concatenate.
                found = true ;
                break ;
            end
        end
        if(~found)
            str = [str tok] ; % Concatenate token to the string if token is not being replaced with another string.
        end
    end
    if(length(rem)) % If more to process
        str = [str rem(1)] ; % Add found delimiter to the string
        rem(1) = [] ; % Remove delimiter
    end
end
% </add>


function fixlength_m(s1,s2,len,indent,fid)

%FIXLENGTH Returns a string which has been divided up into < LEN
%character chuncks with the '...' line divide appended at the end%of each chunck.
%   FIXLENGTH(S1,S2,L) is string S1 with string S2 used as break points into
%   chuncks less than length L.
%Eric Westervelt%5/31/00
%1/11/01 - updated to use more than one dividing string
%4/29/01 - updated to allow for an indent
%3/24/05 - bjm - non-iterative operation, much faster for large files

A=[];
for c = 1:length(s2)
  A = [A findstr(s1,s2(c))];
  A = sort(A);
end
if (length(A)>0)
  dA = [diff(A)];
  B = A;
  runlen = A(1);
  for i=1:1:length(dA)
    if (runlen + dA(i) <= len)
      runlen = runlen + dA(i);
      B(i) = 0;
    else
      runlen = 0;
    end
  end
  B(end) = 0;
  % don't make a newline after the last entry
  B=[0 B(B>0) length(s1)];
  for i=1:1:length(B)-1
    if (i==1)
      fprintf(fid,'%s',s1(B(i)+1:B(i+1)));
    else
      fprintf(fid,'...\n%s%s',indent,s1(B(i)+1:B(i+1)));
    end
  end
else
  fprintf(fid,'%s',s1);
end