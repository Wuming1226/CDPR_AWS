function colorList = slanCM(type, varargin)
% @author : slandarer
% -----------------------
% type : 色带类型
% truncation  : 截取色段区间
% num  : 在色带上均匀采样的颜色的数量（默认取截取区间上的所有颜色）

p = inputParser;            % 函数的输入解析器
addParameter(p,'truncation', [1, 256]);      % 默认截断区间为全区间
addParameter(p,'num', -1);  % 设置特殊初始值，用于判断是否输入可选参数 num
parse(p,varargin{:});       % 对输入变量进行解析，如果检测到前面的变量被赋值，则更新变量取值

if nargin < 1
    type='';
end

if p.Results.num == -1      % 若未输入 num
    num = p.Results.truncation(2) - p.Results.truncation(1) + 1;
else
    num = p.Results.num;
end

slanCM_Data = load('slanCM_Data.mat');
CList_Data = [slanCM_Data.slandarerCM(:).Colors];
% disp(slanCM_Data.author);

if isnumeric(type)
    Cmap = CList_Data{type};
else
    Cpos = strcmpi(type,slanCM_Data.fullNames);
    Cmap = CList_Data{Cpos};
end

Ci = 1:256;
Cq = linspace(p.Results.truncation(1), p.Results.truncation(2), num);
colorList = [interp1(Ci, Cmap(:,1),Cq, 'linear')', ...
           interp1(Ci, Cmap(:,2), Cq, 'linear')', ...
           interp1(Ci ,Cmap(:,3), Cq, 'linear')'];
end