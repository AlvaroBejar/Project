%{
%Code to use data gathered from the ARCCS sensor
accelerationData = csvread('acceleration.csv', 1, 0);
time = accelerationData(:, 1);
acceleration = accelerationData(:, 2:4);
acceleration = acceleration - mean(acceleration)

gyroscopeData = csvread('gyroscope.csv', 1, 0);
time_gyroscope = gyroscopeData(:, 1);
gyroscope = accelerationData(:, 2:4);
gyroscope = gyroscope - mean(gyroscope)
%}

m=mobiledev;
m.SampleRate = 100; %Sets the sample rate for acquired data
m.AccelerationSensorEnabled = 1; %Enables acceleration sensor
m.OrientationSensorEnabled = 1; %Enables orientation sensor
m.Logging = 1; %Starts loggging the data from the phone
pause(10) %Introduces a delay of 10s to log data
m.Logging = 0; %Stops looging data
connector off

Fs = m.SampleRate; % Sampling frequency 

%Acquisition of acceleration, gyroscope and position values
[acceleration, time] = accellog(m); %Gathers accelerometer and time data
acceleration = acceleration - mean(acceleration);
filteredAccelHF = filter(IIRFilterProject, acceleration); %Filters original acceleration to obtain high frequency component
filteredAccelLF = filter(BandpassFilterProject, acceleration);

[position, time_position] = orientlog(m);  %Gathers orientation and time data

axisEnergy = sum(filteredAccelHF.^2); %Calculates signal energy by calculating the total magnitude

totalLF = sum(sum(filteredAccelLF.^2)); %Calculates total high frequency energy
totalHF = sum(sum(filteredAccelHF.^2)); %Calculates total low frequency energy
ratio = totalHF/(totalHF+totalLF); 

if(ratio > 0.2) %Value derived from experimental data
    
    if axisEnergy(1)>axisEnergy(2) && axisEnergy(1)>axisEnergy(3) %X-axis dominant
        
        xAccel = filteredAccelHF(:, 1); %Gets X component of accelerometer data
        xOrientation = position(:, 1);

        [xAccSignal, accTime, signalPoints] = detectSignal(xAccel, time); %Detects relevant part of the signal
        [xOrienSignal, xTime, ~] = detectSignal(xOrientation, time_position);
        zeroMeanXOrien = xOrienSignal - mean(xOrienSignal); %Subtracts mean from data to remove offset
        
        tremorType(filteredAccelLF, signalPoints); %Detects type of tremor
        
        [f, mx, transform] = frequencyAnalysis(xAccel, Fs); %Frequency analysis of the signal
       
        analyseData(xAccSignal, zeroMeanXOrien, accTime); %Magnitude analysis of the signal
        
        %Code for ARCCS sensor processing
        %[gyroscopeData, gyro_time, gyroPoints] = detectSignal(gyroscope(:, 1), time);
        %[~, gyroMax, gyroMin] = rollercoaster(gyroscopeData);
        
    else if axisEnergy(2)>axisEnergy(1) && axisEnergy(2)>axisEnergy(3) %Y-axis dominant
            
        yAccel = filteredAccelHF(:, 2);
        yOrientation = position(:, 2);
        
        [yAccSignal, accTime, signalPoints] = detectSignal(yAccel, time);
        [yOrienSignal, yTime, ~] = detectSignal(yOrientation, time_position);
        zeroMeanYOrien = yOrienSignal - mean(yOrienSignal);
        
        tremorType(filteredAccelLF, signalPoints);
        
        [f, mx, transform] = frequencyAnalysis(yAccel, Fs);
       
        analyseData(yAccSignal, zeroMeanYOrien, accTime);
       
    else if axisEnergy(3)>axisEnergy(1) && axisEnergy(3)>axisEnergy(2) %Z-axis dominant
            
        zAccel = filteredAccelHF(:, 3);
        zOrientation = position(:, 3);  
            
        [zAccSignal, accTime, signalPoints] = detectSignal(zAccel, time);
        [zOrienSignal, zTime] = detectSignal(zOrientation, time_position);
        zeroMeanZOrien = zOrienSignal - mean(zOrienSignal);
       
        tremorType(filteredAccelLF, signalPoints);
        
        [f, mx, transform] = frequencyAnalysis(zAccel, Fs);
        
        analyseData(zAccSignal, zeroMeanZOrien, accTime);
        
        end
        end
    end
else
    disp('Intentional movement');
end


function analyseData(accelerometerData, positionData, time)

%   Function: analyseData(accelerometerData, positionData, time)
%   Input:
%       accelerometerData: Acceleration signal
%       positionData: Orientation signal
%       time: Time series
%   Procedure: Calls relevant functions and displays average of each
%   calculated data
%   

    [angleAmplitude, ~, ~] = rollercoaster(positionData); 

    [~, maxTremor, minTremor] = rollercoaster(accelerometerData);
    tremorAmplitude = findDistance(minTremor, maxTremor, accelerometerData, time);

    disp('Average tremor amplitude in degrees:');
    disp(mean(angleAmplitude));

    disp('Average tremor amplitude in cm:');
    disp(mean(tremorAmplitude));
end


function [f, absoluteSpectrum, transform] = frequencyAnalysis(accelerometerData, Fs)
    
%   Function: frequencyAnalysis(accelerometerData, Fs)
%   Input:
%       accelerometerData: Accelerometer signal
%       Fs: Sample frequency
%   Output:
%       f: Frequency spectrum
%       absoluteSpectrum: Absolute magnitudes of frequency spectrum
%       transform: Fourier transform values of data
%   Procedure: Calculates one sided frequency spectrum of signal to find
%   the frequency with maximum power.
%   

    dataLength = length(accelerometerData);
    transform = fft(accelerometerData,dataLength); %Fourier Transform
    transform = transform(1:round(dataLength/2)); %Spectrum is symmetric so discard negative half
    absoluteSpectrum = abs(transform); %Absolute value of spectrum
    f = (0:dataLength/2)*Fs/dataLength; %Calculate frequency spectrum
    f = f(2:end); %Make vectors the same length

    maxAmplitude = max(transform); %To find dominant frequency of motion
    dominantIndex = find(transform == maxAmplitude);
    dominantFrequency = f(dominantIndex); 

    disp('Dominant frequency');
    disp(dominantFrequency);
end

function tremorType(accelLF, signalPoints)

%   Function: tremorType(accelLF, signalPoints)
%   Input:
%       accelLF: Low frequency component of acceleration signal
%       signalPoints: Index's corresponding to tremor part of the signal
%   Procedure: Checks if there is a significant LF amplitude signal at the same
%   time when there is tremor. If there are sufficient of these cases, the
%   motion is classified as action tremor. 
%   

    xCount = 0; 
    yCount = 0; 
    zCount = 0;
    
    for n=1:2:length(signalPoints)
        
        xAxis = mean(abs(accelLF(signalPoints(n):signalPoints(n+1), 1))); %Average LF amplitude for each tremor part
        yAxis = mean(abs(accelLF(signalPoints(n):signalPoints(n+1), 2)));
        zAxis = mean(abs(accelLF(signalPoints(n):signalPoints(n+1), 3)));
        
        if xAxis>3 %Threshold to be considered relevant
            xCount = xCount + 1; %Adds to count if considered of relevant magnitude
        end
        if yAxis>3
            yCount = yCount + 1;
        end
        if zAxis>3
            zCount = zCount + 1;
        end
    end
    
    if xCount > length(signalPoints)/4 || yCount > length(signalPoints)/2 || zCount > length(signalPoints)/4 %If enough parts of the signal have relevant LF magnitudes
        disp('Action tremor')
    else
        disp('Rest tremor')
    end
end

function [angleAmplitude, maxAmplitudeIndex, minAmplitudeIndex] = rollercoaster(data) 

%   Function: rollercoaster(data)
%   Input:
%       data: Time series data
%   Output:
%       angleAmplitude: Tremor amplitude in degrees
%       maxAmplitudeIndex: Index where the maximum is located
%       minAmplitudeIndex: Index where the minimum is located
%   Procedure: Traverses the signal updating the value of maximum and
%   minumum. When both have been found it resets their values and does the
%   same for the next oscillation.
%       

    currentX = 0;
    state=1;
    angle = 0;
    turningPoints=0;
    initialAngle = data(1);
    minAngle = initialAngle;
    maxAngle = initialAngle;
    
    angleAmplitude = []; %Empty arrays to save data
    maxAmplitudeIndex = [];
    minAmplitudeIndex = [];
    
    lastElement = data(end,end); %Finds last element of the array
    lastElementIndex = find(data == lastElement);
    lastElementIndex = lastElementIndex(end, end);
    
    while 1

        if  currentX == lastElementIndex %To break the loop at the end of the data
            break;
        end
        
        previousAngle = angle;
        currentX = currentX + 1;
        angle = data(currentX);
        
        if angle > maxAngle %Updates maximum value
            maxAngle = angle;
        end

        if angle < minAngle %Updates minimum value
            minAngle = angle;
        end

        if state == 1 && previousAngle < angle %If a minimum is found
            turningPoints = turningPoints + 1;
            minAmplitudeIndex = [minAmplitudeIndex; currentX-1]; %Saves minimum index
            state = 2;
        end

        if state == 2 && previousAngle > angle %If a maximum is found
            turningPoints = turningPoints + 1;
            maxAmplitudeIndex = [maxAmplitudeIndex; currentX-1];
            state = 1;
        end


        if turningPoints == 2 %Once minimum and maximums have been found
            turningPoints = 0;
            angleAmplitude = [angleAmplitude; abs(maxAngle)+abs(minAngle)]; %Saves total tremor amplitude in degrees
            maxAngle = initialAngle; %Resets maximum value
            minAngle = initialAngle;
        end
    end
    
    if data(1)<data(2) %State=1 at the start, so the first point is registered as a minmum in this case
        minAmplitudeIndex = minAmplitudeIndex(2:end, :);
    end

end


function [tremorAmplitude] = findDistance(minAmplitudeIndex, maxAmplitudeIndex, data, time)

%   Function: findDistance(minAmplitudeIndex, minAmplitudeIndex, data, time)
%   Input:
%       minAmplitudeIndex: Index where the minimum is located
%       maxAmplitudeIndex: Index where the maximum is located
%       data: Time series data
%       time: Time data
%   Output:
%       tremorAmplitude: Amplitude of tremor in centimetres
%   Procedure: Checks there is equal number of minimums and maximums and
%   integrates acceleration data to get distance
%       

    if length(maxAmplitudeIndex)>length(minAmplitudeIndex)
        
        maxAmplitudeIndex = maxAmplitudeIndex(1:end-(length(maxAmplitudeIndex)-length(minAmplitudeIndex))); %Reduces arrays to be the same length
        tremorAmplitude = 100 * integrateData(minAmplitudeIndex, maxAmplitudeIndex, data, time); %Calculates amplitude of tremor in centimetres
 
    else if length(maxAmplitudeIndex)<length(minAmplitudeIndex)
            
        minAmplitudeIndex = minAmplitudeIndex(1: end-(length(minAmplitudeIndex)-length(maxAmplitudeIndex)));
        tremorAmplitude = 100 * integrateData(minAmplitudeIndex, maxAmplitudeIndex, data, time);
    
    else
        tremorAmplitude = 100 * integrateData(minAmplitudeIndex, maxAmplitudeIndex, data, time);
        end
    end
end



function [tremorAmplitude] = integrateData(minAmplitudeIndex, maxAmplitudeIndex, data, time)
    
%   Function: integrateData(minAmplitudeIndex, minAmplitudeIndex, data, time)
%   Input:
%       minAmplitudeIndex: Index where the minimum is located
%       maxAmplitudeIndex: Index where the maximum is located
%       data: Time series data
%       time: Time series
%   Output:
%       tremorAmplitude: Total amplitude of tremor in metres
%   Procedure: Checks if minimum or maximum goes first. It then finds time
%   limits and data to be integrated and integrates data with respect to
%   time.
%   

    tremorAmplitude= [];
    
    if data(1)>data(2) %Checks if minmum or maximum comes first to do integration
        for n=1:length(minAmplitudeIndex)
            timeLimits = time(minAmplitudeIndex(n):maxAmplitudeIndex(n)); %Integration time limits
            dataLimits = data(minAmplitudeIndex(n):maxAmplitudeIndex(n)); %Data to integrate
            velocity = trapz(timeLimits, abs(dataLimits)); %Calculates velocity for motion to be assumed constant
            distance = velocity*(timeLimits(end)-timeLimits(1));
            if distance<0.0001 %Checks velocity calculates is not noise
                continue;
            else
                tremorAmplitude = [tremorAmplitude; distance];
            end
        end
    else 
        for n=1:length(maxAmplitudeIndex)
            timeLimits = time(maxAmplitudeIndex(n):minAmplitudeIndex(n));
            dataLimits = data(maxAmplitudeIndex(n):minAmplitudeIndex(n));
            velocity = cumtrapz(timeLimits, abs(dataLimits));
            accel = trapz(timeLimits, velocity);
            if accel<0.0001
                continue;
            else
                tremorAmplitude = [tremorAmplitude; accel];
            end
        end
    end
end

function [realSignal, realTime, signalPoints] = detectSignal(data, time)
    
%   Function: detectSignal(data, time)
%   Input:
%       data: Time series data
%       time: Time data
%   Output:
%       realSignal: Signal values corresponding to tremor
%       realTime: Time values corresponding to tremor part of time-series
%       data
%       signalPoints: Index where tremor signal starts or ends
%   Procedure: Calculates the mean absolute amplitude of each signal
%   division. Each average is then compared to the maximum average to see
%   if that part of the signal is tremor or noise.
%       

    differencePoints = findPoints(data);
    averageAmplitude = zeros(size(differencePoints)); %Array of zeros to store values
    signalPoints = [];
    realSignal = [];
    realTime = [];

    for i = 1:length(differencePoints)-1 %Calculates average amplitude of intervals located by findPoints()
        averageAmplitude(i) = mean(abs(data(differencePoints(i):differencePoints(i+1)))); %Stores values in averageAmplitude
    end

    for j = 1:length(averageAmplitude)
        if averageAmplitude(j) >= 0.5 * max(averageAmplitude) %Checks magnitude of current average compared to maximum average
           signalPoints = [signalPoints; differencePoints(j:j+1)];
           
           if isempty(realSignal) ~= 1 %If no tremor signal has been found yet
               
               if data(differencePoints(j)) == realSignal(end) 
                   realSignal = [realSignal; data(differencePoints(j)+1:differencePoints(j+1))]; %Saves values of tremor signal
                   realTime = [realTime; time(differencePoints(j)+1:differencePoints(j+1))]; %Saves time values corresponding to tremor
               else
                   realSignal = [realSignal; data(differencePoints(j):differencePoints(j+1))];
                   realTime = [realTime; time(differencePoints(j):differencePoints(j+1))];
               end
               
           else
           
               realSignal = [realSignal; data(differencePoints(j):differencePoints(j+1))];
               realTime = [realTime; time(differencePoints(j):differencePoints(j+1))];

           end
        end
    end
   
end

function [actualSignal] = findPoints(data)

%   Function: findPoints(data)
%   Input:
%       data: Time series data
%   Output:
%       actualSignal: Signal points where there is a change in entropy
%       slope (point where signal changes from tremor to noise or viceversa)
%   Procedure: Calculates the entropy for different lengths of the signal.
%   The entropy then gets smoothed by a moving average filter. Changes in
%   slope of the smoothed entropy are tracked with these points being
%   stored in "actualSignal"
%   
    
    entropy = [];
    actualSignal = [1]; %Adds first index to signal indexes
    m = 1;
    
    signalCheck = 1;
    noiseCheck = 1;
    
    for n=1:length(data) %Calculates entropy for different lengths of signal
        entropy = [entropy; SampEn(3, 0.25*std(data), data(1:n))];
    end
    
    numbersInAverage = 30;
    coefficients = ones(1, numbersInAverage)/numbersInAverage;
    filteredAverage = filter(coefficients, 1, entropy); %Smooths entropy curve with moving average filter
    
    while 1 %Tracks positive and negative slopes of entropy
        if m == length(filteredAverage)-1 %Breaks while loop
            break;
        end
        
        if isnan(filteredAverage(m)) %If current entropy value is Not a Number (NaN)
            m = m + 1;
            continue;
        end
        
        if isinf(filteredAverage(m)) %If current entropy value is infinity
            m = m + 1;
            continue;
        end
        
        if filteredAverage(m+1) - filteredAverage(m) == 0 %If there is no slope
            m = m + 1;
            continue,
        end
        
        if filteredAverage(m+1) - filteredAverage(m) > 0 %For positive slopes
            while filteredAverage(m+1) - filteredAverage(m) > 0
                if m == length(filteredAverage)-1
                    break;
                end
                if signalCheck == 1 %For first point where slope changes
                    actualSignal = [actualSignal; m]; %Store index where slope changes
                    signalCheck = 2;
                end
                m = m + 1;
            end
            signalCheck = 1; %Resets signal start
        end
            
        if filteredAverage(m+1) - filteredAverage(m) < 0 %For negative slopes
            while filteredAverage(m+1) - filteredAverage(m) < 0
                if m == length(filteredAverage)-1
                    break;
                end
                if noiseCheck == 1
                    actualSignal = [actualSignal; m];
                    noiseCheck = 2;
                end
                m = m + 1;
            end
            noiseCheck = 1;
        end
        
    end
    actualSignal = [actualSignal; length(filteredAverage)]; %Includes last index to consider last interval
end