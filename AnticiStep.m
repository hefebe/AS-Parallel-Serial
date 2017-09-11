function[anticiMap, anticiMap2, anticiMap3, anticiMap4, X, Y] = AnticiStep(model, krange, trange, deci, feedback)

% This function displays the degree of anticipation between the target
% dynamics and the movement of the robot's end effector (along the x-axis)

% Arguments:
% model = string containing name of the desired Simulink model
% krange = max value for the coupling constant (K)
% trange = maximum delay (in seconds)
% deci = resolution of the figures (number of equal length steps in delay and K)
% feedback = if 1 delay only feedback, if 0 delay target and feedback (sensory delay)

oStep = trange/deci;% delay step
tStep = krange/deci;% coupling strength step

% gStep = 10;% positional gain step

const = 0.01;% time constant

start = 0;

oRange = oStep:oStep:trange;
tRange = tStep-start:tStep:krange;% I don't know why t is K, but whatever

%gRange = gStep:gStep:200;

X = oRange;
Y = tRange;

%Z = gRange;

anticiMap = zeros(max(size(X)),max(size(Y)));
anticiMap2 = zeros(max(size(X)),max(size(Y)));
anticiMap3 = zeros(max(size(X)),max(size(Y)));
anticiMap4 = zeros(max(size(X)),max(size(Y)));

if(feedback)
    del = '/FeedbackDelay';
    set_param(strcat(model,'/Feedback delay on'),'value',mat2str(1));
    set_param(strcat(model,'/Sensory delay on'),'value',mat2str(0));
else
    del = '/SenseDelay';
    set_param(strcat(model,'/Feedback delay on'),'value',mat2str(0));
    set_param(strcat(model,'/Sensory delay on'),'value',mat2str(1));
end

% for g = gRange

for o = oRange
    for t = tRange
        
        set_param(strcat(model,del),'value',mat2str(o));
        
        set_param(strcat(model,'/K - coupling term'),'gain',mat2str(t));
        
%         set_param(strcat(model,'/TheKay2'),'gain',mat2str(t/50));
        
%         set_param(strcat(model,'/Visual Gains'),'value',mat2str([70;100]*t));% change gain multiplier while retaining ratio
        
        save_system(model);
        sim(model);
        
        bigness = max(size(doop.data(:,1)));% size of time vector, to subtract from xcorr index
        
        [vals, lags] = xcorr(doop.data(1:end,1)/10,xy.data(1:end,1)-5,'coeff');% cross-correlate the scaled target and target (minus offset)
        
        [big, in] = max(vals);% find the lag at which the correlation coefficient is highest

        
        
        ind = in-bigness;%subtract length of time vector to get negative and positive lags
        
        snaX = [xy.data(:,1)-5,xy.data(:,2)];% get the two vectors (just for convenience)
        
        if(ind<0)% if the lag is positive (i.e. it is not a lead)
            stump = sqrt(sum(sum((doop.data(1:end-abs(ind),:)./10-snaX(1+abs(ind):end,:)).^2))/numel(doop.data(1:end-abs(ind),:)));% calculate lag-adjusted RMS error
        else
            stump = sqrt(sum(sum((doop.data(1+abs(ind):end,:)./10-snaX(1:end-abs(ind),:)).^2))/numel(doop.data(1+abs(ind):end,:)));% calculate lag-adjusted RMS error
        end
        
        anticiMap4(round(o/oStep),round((t+start)/tStep)) = stump;% create map of lag-adjusted RMS error 
        
        anticiMap(round(o/oStep),round((t+start)/tStep)) = -lags(in)*const;% create map of the lags

        anticiMap2(round(o/oStep),round((t+start)/tStep)) = big;% very rough measure of anticipation
        
%         [vals, lags] = xcorr(truetarg.data(80:end,1),targ.data(80:end,1),'coeff');
%         
%         [big, in] = max(abs(vals));
        
%         anticiMap3(round(o/oStep),round(t/tStep)) = lags(in);
        if(max(size(doop.data(:,1))))
            [vals1, pks1] = findpeaks(doop.data(:,1));
            [vals2, pks2] = findpeaks(xy.data(:,1)-5);
            holder = 0;
            if(max(size(pks1))>1)
                for a =1:max(size(pks1))
                    set = pks2;% get all lag values that are within one delay unit of the true peak
                    if(size(set)>0)
                    [small, in] = min(abs(set-pks1(a)));% extract the location of the closest peak
                    duf = set-pks1(a);
                    holder = holder+duf(in);

                    end
                end
            end
        else
            holder = 0;% failure value - replaces NaN
            pks1=1;
        end
        anticiMap3(round(o/oStep),round((t+start)/tStep)) = holder/max(size(pks1))*const;% output peak-to-peak lag
    end
    o
end

% Output figures, with delay plotted along the x axis and coupling strength
% along y
figure(1), pcolor(X,Y,anticiMap');shading interp;% lag/lead of robot movement vs. the target

figure(2), pcolor(X,Y,anticiMap2');shading interp;% Correlation coefficient

figure(3), pcolor(X,Y,anticiMap3');shading interp;% averaged

figure(4), pcolor(X,Y,anticiMap4');shading interp;% lag-adjusted RMS error

o