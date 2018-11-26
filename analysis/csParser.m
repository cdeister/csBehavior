function [parsedStruct,csBData,csBehavHDFInfo]=csParser(csBehaviorHDFPath,versionNum)

% csParser, a function that parses behavior data from csDiscrim/csVisual
% arguments: "csBehaviorHDFPath" is a path to the hdf file you want to
% parse. "versionNum" is an optional argument that isn't implemented yet.
% You can also pass 0 arguments and a dialog to select an hdf will pop up.
% returns: "parsedStruct" is a struct of parsed behavior data and what the 
% function is intended to produce."csBData" is the raw dataset in the hdf
% only. "csBehavHDFInfo" is the hdf5 library info.
%
% v0.25 -- Chris Deister - cdeister@brown.edu



if nargin==0
    [tHDF,tPth]=uigetfile('*.hdf','what what?');
    csBehaviorHDFPath=[tPth tHDF];
    versionNum=0;
elseif nargin==1
    versionNum=0;
end


csBehavHDFInfo=h5info(csBehaviorHDFPath);
curDatasetPath=['/' csBehavHDFInfo.Datasets.Name];
csBData=h5read(csBehaviorHDFPath,curDatasetPath);


disp(size(csBData,2))
if size(csBData,1)==10
    versionNum=0.2;
    % map of data labels and numerical indicies (for the data in the dataset only).
    chStrMap={'interrupts','sessionTime','stateTime','teensyStates','loadCell','lickSensor',...
        'motionTracker','scopeState','pythonStates','thresholdedLicks'};
    chIndMap=[1,2,3,4,5,6,7,8,9,10];
    % simple anonymous function to easily index the channels by name. This allows flexibility.
    % use: chFind('interrupts') will return 1.
    
    
elseif size(csBData,1)==11
    versionNum=0.25;
    % map of data labels and numerical indicies (for the data in the dataset only).
    chStrMap={'interrupts','sessionTime','stateTime','teensyStates','loadCell','lickSensor',...
        'motionTracker','scopeState','aOut1','pythonStates','thresholdedLicks'};
    chIndMap=[1,2,3,4,5,6,7,8,9,10,11];
    % simple anonymous function to easily index the channels by name. This allows flexibility.
    % use: chFind('interrupts') will return 1.
elseif size(csBData,1)==12
    versionNum=0.3;
    % map of data labels and numerical indicies (for the data in the dataset only).
    chStrMap={'interrupts','sessionTime','stateTime','teensyStates','loadCell','lickSensor',...
        'motionTracker','scopeState','aOut1','aOut2','pythonStates','thresholdedLicks'};
    chIndMap=[1,2,3,4,5,6,7,8,9,10,11,12];
    % simple anonymous function to easily index the channels by name. This allows flexibility.
    % use: chFind('interrupts') will return 1.
end

chFind=@(b) chIndMap(find(strcmp(chStrMap,b)==1));



% A) Parse the dataset.
for n=1:numel(chStrMap)
	eval(['parsedStruct.' chStrMap{n} '=csBData(chFind("' chStrMap{n} '"),:);'])
end

parsedStruct.sessionTime=parsedStruct.sessionTime/1000;
parsedStruct.stateTime=parsedStruct.stateTime/1000;

% B) Define a trial by the onset of the stimulus state (usually state2). 
% Store the stim-onset samples, make a binary vector and count trials.
% State is logged on teensy and in python, and can differ during transitions. However, the teensyState
% is the actual state.
[parsedStruct.stimSamps,parsedStruct.stimVector]=getStateSamps(parsedStruct.teensyStates,2,1);
parsedStruct.completedTrials=numel(parsedStruct.stimSamps);


% C) Resolve 'attribute' data.
% Data that do not change within a 'trial' are stored as attributes for the
% parent dataset, which is the ms-ms data that comprise all trials.
% For the most part, these attribute data are stimulus parameters. Here we
% parse the attribute data. Some stimulus parameters are generated for all 
% trials at run-time. This prevents having to recalculate things that are pre-determined. 
% For these I pad the arrays by a large amount, just in case the user wants to
% increase the number of trials. Thus, we will usually end up with more of
% these than trials we ran. So, we need to trim these to the length of the trials ran.
% This stuff is most likley to change with format changes.

% contrasts as a fraction.
attDataScalars=[1,1,1,0.001,0.001];
attDataLabels={'contrasts','orientations','spatialFreqs','waitTimePads','trialDurs','stimTrials'};
attDataNames={'contrasts','orientations','spatialFreqs','waitTimes','trialDurations','stimTrials'};

for n=1:numel(attDataScalars)
    try
        eval(['parsedStruct.' attDataNames{n} '=double(h5readatt(csBehaviorHDFPath,curDatasetPath, char(attDataLabels{' num2str(n) '})));']);
        eval(['parsedStruct.' attDataNames{n} '=transpose(parsedStruct.' attDataNames{n} '(1:parsedStruct.completedTrials)*attDataScalars(' num2str(n) '));'])
    catch
    end
end

% motion is usually parsed offline (the following assumes position was tracked with a rotary encoder, not a quadrature).
parsedStruct.position=decodeShaftEncoder(parsedStruct.motionTracker,3);
parsedStruct.velocity=nPointDeriv(parsedStruct.position,parsedStruct.sessionTime,1000);
parsedStruct.velocity(find(isnan(parsedStruct.velocity)==1))=0;
parsedStruct.binaryVelocity=binarizeResponse(parsedStruct.velocity,0.005);
parsedStruct.motionBoutStarts=find(diff(parsedStruct.binaryVelocity)>0.8);
parsedStruct.versionNum=versionNum;



end

