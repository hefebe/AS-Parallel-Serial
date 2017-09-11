function[anticiMap, anticiMap2, anticiMap3, anticiMap4, X, Y] = AnticiStepDisturbance(model, krange, trange, deci)

% This function displays the degree of lag/anticipation between the target
% dynamics and the movement of the robot's end effector (along the x-axis),
% when the system is subject to a torque impulse at the joints.
% It is only designed to work with sensory delay (delay applied to both
% target input and feedback) because the effect of the disturbance is
% measured at 0 lag

% currently this does not automatically set the disturbance term in case a
% different form of disturbance is desired, and the disturbance onset is
% hardcoded

% Arguments:
% model = string containing name of the desired Simulink model
% krange = max value for the coupling constant (K)
% trange = maximum delay (in seconds)
% deci = resolution of the figures (number of equal length steps in delay and K)

onset = 20000;% onset of disturbance in simulation time units (0.01s)

oStep = trange/70;% delay step cut into 0.01s pieces
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

% for g = gRange

for o = oRange
    for t = tRange
        
        set_param(strcat(model,'/SenseDelay'),'value',mat2str(o));
        
        set_param(strcat(model,'/TheKay'),'gain',mat2str(t));
        
%         set_param(strcat(model,'/Visual Gains'),'value',mat2str([70;100]*t));% change gain multiplier while retaining ratio
        
        save_system(model);
        sim(model);
        
        bigness = max(size(doop.data(:,1)));% size of time vector, to subtract from xcorr index
        
        [vals, lags] = xcorr(doop.data(1:end,1)/10,xy.data(1:end,1)-5,'coeff');
        
        [big, in] = max(vals);% removed the absolute command, because I'm not interested in negative synchronisation

        
        
%         ind = in-bigness;%subtract length of time vector to get negative and positive lags
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        ind = 0;% FOR DISTURBANCE TEST - the effects of the disturbance are measured for 10s after onset
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        snaX = [xy.data(:,1)-5,xy.data(:,2)];
        
        if(max(size(doop.data))<30000)% if the timeseries is sufficiently long (i.e. instability hasn't stopped it early)
            stump = sqrt(sum(sum((doop.data(onset:end-abs(ind),:)./10-snaX(onset+abs(ind):end,:)).^2))/numel(doop.data(onset:end-abs(ind),:)));% calculate lag-adjusted RMS error
        else if(max(size(doop.data))<onset)% if instability occurs before disturbance
            stump = 100;% error value
            else% else calculate RMS error up to end of timeseries
            stump = sqrt(sum(sum((doop.data(onset+abs(ind):30000,:)./10-snaX(onset:30000-abs(ind),:)).^2))/numel(doop.data(onset+abs(ind):30000,:)));% calculate lag-adjusted RMS error
            end
        end
        
        anticiMap4(round(o/oStep),round((t+start)/tStep)) = stump;
        
        anticiMap(round(o/oStep),round((t+start)/tStep)) = -lags(in)*const;

        anticiMap2(round(o/oStep),round((t+start)/tStep)) = big;% very rough measure of anticipation
        
%         [vals, lags] = xcorr(truetarg.data(80:end,1),targ.data(80:end,1),'coeff');
%         
%         [big, in] = max(abs(vals));
        
%         anticiMap3(round(o/oStep),round(t/tStep)) = lags(in);
        if(max(size(doop.data(:,1))))
            [vals1, pks1] = findpeaks(doop.data(:,1));% I'm very unsure about this dodgy, dodgy algorithm
            [vals2, pks2] = findpeaks(xy.data(:,1)-5);
            holder = 0;
            if(max(size(pks1))>1)
                for a =1:max(size(pks1))
                    set = pks2;%pks2(pks2<=pks1(a)+o/const & pks2>=pks1(a)-o/const);% get all lag values that are within one delay unit of the true peak
                    if(size(set)>0)
                    [small, in] = min(abs(set-pks1(a)));% extract the location of the closest peak
                    duf = set-pks1(a);
                    holder = holder+duf(in);
                    % output it?  I sure hope so!
                    end
                end
            end
        else
            holder = 0;
            pks1=1;
        end
        anticiMap3(round(o/oStep),round((t+start)/tStep)) = holder/max(size(pks1))*const;
    end
    o
end

% end

anticiMap4(isnan(anticiMap4))=1;% eliminate NaNs

figure(1), pcolor(X,Y,anticiMap');shading interp;

figure(2), pcolor(X,Y,anticiMap2');shading interp;

figure(3), pcolor(X,Y,anticiMap3');shading interp;

figure(4), pcolor(X,Y,anticiMap4');shading interp;% RMS error following disturbance - the other outputs are purely for validation, as no significant
%                                                   anticipation/lag is
%                                                   observed

o