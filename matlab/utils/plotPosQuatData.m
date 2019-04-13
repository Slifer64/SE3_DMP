function [axP, axQ, axPQ] = plotPosQuatData(varargin)

%% =================   Parse arguments  =====================
inArgs = parseInputArguments(varargin{:});

pAxes = inArgs.pAxes;
qAxes = inArgs.qAxes;
pqAxes = inArgs.pqAxes;

line_style = inArgs.LineStyle;
line_width = inArgs.LineWidth;
xAxisColor = inArgs.xAxisColor;
yAxisColor = inArgs.yAxisColor;
zAxisColor = inArgs.zAxisColor;
posColor = inArgs.PosColor;
velColor = inArgs.VelColor;
accelColor = inArgs.AccelColor;
pathLineColor = inArgs.PathLineColor;
    
Data = inArgs.Data;
if isempty(Data)
    filename = inArgs.filename;
    load(filename, 'Data');
end

Time = Data.Time;
Pos = Data.Pos;
Vel = Data.Vel;
Accel = Data.Accel;
Quat = Data.Quat;
RotVel = Data.RotVel;
RotAccel = Data.RotAccel;


label_font = 15;
title_font = 17;

%% ====================================================================

axP = pAxes;
if (isempty(axP))
    figP = figure;
    axP = cell(3,3);
    for i=1:3
        for j=1:3
            axP{i,j} = subplot(3,3,(i-1)*3+j,'Parent',figP); 
            hold(axP{i,j}, 'on');
        end
    end
end

for i=1:3
    plot(Time, Pos(i,:), 'LineWidth',line_width, 'LineStyle',line_style, 'Color',posColor, 'Parent',axP{1,i});
    plot(Time, Vel(i,:), 'LineWidth',line_width, 'LineStyle',line_style, 'Color',velColor,'Parent',axP{2,i});
    plot(Time, Accel(i,:), 'LineWidth',line_width, 'LineStyle',line_style, 'Color',accelColor,'Parent',axP{3,i});
    if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',label_font, 'Parent',axP{3,i}); end
end
title('$X$', 'interpreter','latex', 'fontsize',title_font, 'Parent',axP{1,1});
title('$Y$', 'interpreter','latex', 'fontsize',title_font, 'Parent',axP{1,2});
title('$Z$', 'interpreter','latex', 'fontsize',title_font, 'Parent',axP{1,3});
ylabel('$p$ [$m$]', 'interpreter','latex', 'fontsize',label_font, 'Parent',axP{1,1});
ylabel('$\dot{p}$ [$m/s$]', 'interpreter','latex', 'fontsize',label_font, 'Parent',axP{2,1});
ylabel('$\ddot{p}$ [$m/s^2$]', 'interpreter','latex', 'fontsize',label_font, 'Parent',axP{3,1});

%% ====================================================================

axQ = qAxes;
if (isempty(axQ))
    figQ = figure;
    axQ = cell(3,3);
    for i=1:3
        for j=1:3
            axQ{i,j} = subplot(3,3,(i-1)*3+j,'Parent',figQ);
            hold(axQ{i,j}, 'on');
        end
    end
end

qPos = zeros(3, size(Quat,2));
Q0 = Quat(:,end);
for i=1:size(qPos,2)
    qPos(:,i) = quatLog( quatProd( Quat(:,i), quatInv(Q0) ) );
end

for i=1:3
    plot(Time, qPos(i,:), 'LineWidth',line_width, 'LineStyle',line_style, 'Color',posColor, 'Parent',axQ{1,i});
    plot(Time, RotVel(i,:), 'LineWidth',line_width, 'LineStyle',line_style, 'Color',velColor, 'Parent',axQ{2,i});
    plot(Time, RotAccel(i,:), 'LineWidth',line_width, 'LineStyle',line_style, 'Color',accelColor, 'Parent',axQ{3,i});
    if (i==3), xlabel('time [$s$]', 'interpreter','latex', 'fontsize',label_font, 'Parent',axQ{3,i}); end
end
title('$X$', 'interpreter','latex', 'fontsize',title_font, 'Parent',axQ{1,1});
title('$Y$', 'interpreter','latex', 'fontsize',title_font, 'Parent',axQ{1,2});
title('$Z$', 'interpreter','latex', 'fontsize',title_font, 'Parent',axQ{1,3});
ylabel('$log(Q*Q_f^{-1})$ [$rad$]', 'interpreter','latex', 'fontsize',label_font, 'Parent',axQ{1,1});
ylabel('$\omega$ [$rad/s$]', 'interpreter','latex', 'fontsize',label_font, 'Parent',axQ{2,1});
ylabel('$\dot{\omega}$ [$rad/s^2$]', 'interpreter','latex', 'fontsize',label_font, 'Parent',axQ{3,1});


%% ====================================================================

axPQ = pqAxes;
if isempty(axPQ)
    fig = figure;
    axPQ = axes('Parent',fig);
    hold(axPQ, 'on');
end
n_data = size(Pos,2);
step = floor(n_data/450);
plot_3Dpath_with_orientFrames(Pos(:,1:step:end), Quat(:,1:step:end), 'axes',axPQ, 'title','$3D$ path with orientation', 'xlabel','$x$ [$m$]', 'ylabel','$y$ [$m$]', 'zlabel','$z$ [$m$]', ...
        'LineWidth',3, 'LineColor',[0.45 0.26 0.26], 'LineStyle',line_style, 'Interpreter','latex', 'fontSize',14, 'numberOfFrames',inArgs.Nframes, ...
        'frameScale',0.1, 'frameLineWidth',2.0, 'animated',inArgs.animated, 'Time',0.002, ...
        'FrameXAxisColor',xAxisColor, 'FrameYAxisColor',yAxisColor, 'FrameZAxisColor',zAxisColor, 'LineColor',pathLineColor);
    

end


function [inArgs, usingDefaults, unmatchedNames] = parseInputArguments(varargin)

% initialize parser with the names and default values of the input arguments
inPars = inputParser;
inPars.KeepUnmatched = true;
inPars.PartialMatching = false;
inPars.CaseSensitive = false;

inPars.addParameter('Data', [], @(x)assert( isstruct(x)||isempty(x), 'Data must be a struct containing the data to be plotted.' ) );
inPars.addParameter('filename', '', @(x)assert( ischar(x), 'filename must be a character array.' ) );
inPars.addParameter('pAxes', [], @(x)assert( true, 'pAxes must be of type axes.' ) );
inPars.addParameter('qAxes', [], @(x)assert( true, 'qAxes must be of type axes.' ) );
inPars.addParameter('pqAxes', [], @(x)assert( true, 'pqAxes must be of type axes.' ) );

inPars.addParameter('LineStyle', '-', @(x)assert( strcmp(x,'-')||strcmp(x,'--')||strcmp(x,':')||strcmp(x,'-.'), 'Invalid or unsupported LineStyle' ) );
inPars.addParameter('LineWidth', 1.5, @(x)assert( isnumeric(x), 'Invalid LineWidth' ) );
inPars.addParameter('xAxisColor', [1 0 0], @(x)assert( isvector(x) && numel(x)==3, 'Invalid xAxisColor' ) );
inPars.addParameter('yAxisColor', [0 1 0], @(x)assert( isvector(x) && numel(x)==3, 'Invalid xAxisColor' ) );
inPars.addParameter('zAxisColor', [0 0 1], @(x)assert( isvector(x) && numel(x)==3, 'Invalid xAxisColor' ) );

inPars.addParameter('PosColor', [0 0 1], @(x)assert( isvector(x) && numel(x)==3, 'Invalid PosColor' ) );
inPars.addParameter('VelColor', [0 0.7 0], @(x)assert( isvector(x) && numel(x)==3, 'Invalid VelColor' ) );
inPars.addParameter('AccelColor', [0.6 0 0], @(x)assert( isvector(x) && numel(x)==3, 'Invalid AccelColor' ) );

inPars.addParameter('PathLineColor', [0.45 0.26 0.26], @(x)assert( isvector(x) && numel(x)==3, 'Invalid PathLineColor' ) );
inPars.addParameter('Nframes', 10, @(x)assert( isnumeric(x), 'Invalid Nframes param' ) );
inPars.addParameter('animated', false, @(x)assert( isscalar(x), 'Invalid animated param' ) );

% Parse input arguments
inPars.parse(varargin{:});
inArgs = inPars.Results;

unmatchedNames = fieldnames(inPars.Unmatched);
usingDefaults = inPars.UsingDefaults;

if (~isempty(unmatchedNames))
        str = sprintf('plot_3Dpath_with_orientFrames: Found unmatched argument names:\n');
        for i=1:length(unmatchedNames)
            str = [str sprintf('%s\n', unmatchedNames{i})];
        end
        warning('%s', str); 
end

end

