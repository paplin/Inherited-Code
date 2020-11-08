clear all
close all
clc
clearvars
%Discover Radio

connectedRadios = findsdru;
if strncmp(connectedRadios(1).Status, 'Success', 7)
  radioFound = true;
  platform = connectedRadios(1).Platform;
  switch connectedRadios(1).Platform
    case {'B200','B210'}
      address = connectedRadios(1).SerialNum;
    case {'N200/N210/USRP2','X300','X310'}
      address = connectedRadios(1).IPAddress;
  end
else
  radioFound = false;
  address = '192.168.10.2';
  platform = 'N200/N210/USRP2';
end

% Transmitter Initialization
rfTxFreq = 20e3; % RF Transmitter Center Frequency (Hz)
frsFDTxParams = configureFDTx(platform, rfTxFreq);

% Receiver Initialization
rfRxFreq = 20e3; % RF Receiver Center Frequency (Hz)
rfRxFreqCorrection = -4e3; % Frequency calibration compensation value (Hz)
rfRxFreqActual = rfRxFreq + rfRxFreqCorrection;
frsFDRxParams = configureFDRx(platform, rfRxFreqActual);
% Create a data source to transmit the contents of a sound file at a
% sampling frequency of 8 kHz.
source = FRSGMRSDemoSource('Chirp', frsFDTxParams.SourceSampleRate);
% source.AudioFileName = 'speech_dft.avi';

% The Continuous Tone-Coded Squelch System (CTCSS) filters out
% undesired communication or interference from these other users by
% generating a tone between 67 Hz and 250 Hz and transmitting it along with
% the source signal.
ctcss = dsp.SineWave(frsFDTxParams.CTCSSAmplitude, ...
    frsFDTxParams.CTCSSToneFrequencies(frsFDTxParams.CTCSSCode), ...
    'SampleRate', frsFDTxParams.SourceSampleRate, ...
    'SamplesPerFrame', frsFDTxParams.SourceFrameLength, ...
    'OutputDataType', 'single');

% The interpolator and FM modulator convert the sampling rate of the sum of
% the modulating signal and the CTCSS tone to match the USRP(R) hardware
% sampling rate of 200 kHz.
interpolator = dsp.FIRInterpolator(frsFDTxParams.InterpolationFactor, ...
    frsFDTxParams.InterpolationNumerator);

fmMod = comm.FMModulator('SampleRate', frsFDTxParams.RadioSampleRate, ...
    'FrequencyDeviation', frsFDTxParams.FrequencyDeviation);
% Set up transmitter radio object to use the found radio
switch platform
  case {'B200','B210'}
    radioTx = comm.SDRuTransmitter('Platform', platform, ...
        'SerialNum', address, ...
        'MasterClockRate', frsFDTxParams.RadioMasterClockRate, ...
        'CenterFrequency', frsFDTxParams.CenterFrequency,...
        'Gain', frsFDTxParams.RadioGain, ...
        'InterpolationFactor', frsFDTxParams.RadioInterpolationFactor)
  case {'X300','X310'}
    radioTx = comm.SDRuTransmitter('Platform', platform, ...
        'IPAddress', address, ...
        'MasterClockRate', frsFDTxParams.RadioMasterClockRate, ...
        'CenterFrequency', frsFDTxParams.CenterFrequency,...
        'Gain', frsFDTxParams.RadioGain, ...
        'InterpolationFactor', frsFDTxParams.RadioInterpolationFactor)
  case {'N200/N210/USRP2'}
    radioTx = comm.SDRuTransmitter('Platform', platform, ...
        'IPAddress', address, ...
        'CenterFrequency', frsFDTxParams.CenterFrequency,...
        'Gain', frsFDTxParams.RadioGain, ...
        'InterpolationFactor', frsFDTxParams.RadioInterpolationFactor)
end
 


% Set up transmitter radio object to use the found radio
switch platform
  case {'B200','B210'}
    radioRx = comm.SDRuReceiver('Platform', platform, ...
        'SerialNum', address, ...
        'MasterClockRate', frsFDRxParams.RadioMasterClockRate, ...
        'CenterFrequency', frsFDRxParams.CenterFrequency,...
        'Gain', frsFDRxParams.RadioGain, ...
        'DecimationFactor', frsFDRxParams.RadioDecimationFactor, ...
        'SamplesPerFrame', frsFDRxParams.RadioFrameLength, ...
        'OutputDataType', 'single')
  case {'X300','X310'}
    radioRx = comm.SDRuReceiver('Platform', platform, ...
        'IPAddress', address, ...
        'MasterClockRate', frsFDRxParams.RadioMasterClockRate, ...
        'CenterFrequency', frsFDRxParams.CenterFrequency,...
        'Gain', frsFDRxParams.RadioGain, ...
        'DecimationFactor', frsFDRxParams.RadioDecimationFactor, ...
        'SamplesPerFrame', frsFDRxParams.RadioFrameLength, ...
        'OutputDataType', 'single')
  case {'N200/N210/USRP2'}
    radioRx = comm.SDRuReceiver('Platform', platform, ...
        'IPAddress', address, ...
        'CenterFrequency', frsFDRxParams.CenterFrequency,...
        'Gain', frsFDRxParams.RadioGain, ...
        'DecimationFactor', frsFDRxParams.RadioDecimationFactor, ...
        'SamplesPerFrame', frsFDRxParams.RadioFrameLength, ...
        'OutputDataType', 'single')
end

% AGC
agc = comm.AGC;

% Low pass filter for channel separation
channelFilter = frsFDRxParams.ChannelFilter;

% FM demodulator
fmDemod = comm.FMDemodulator('SampleRate', frsFDRxParams.RadioSampleRate, ...
    'FrequencyDeviation', frsFDRxParams.FrequencyDeviation);

% Decimation filter to resample to 8 kHz
decimator = dsp.FIRDecimator(frsFDRxParams.DecimationFactor, ...
    frsFDRxParams.DecimationNumerator);

% The CTCSS decoder compares the estimated received code with the
% preselected code and then sends the signal to the audio device if the two
% codes match.
decoder = FRSGMRSDemoCTCSSDecoder(...
    'MinimumBlockLength', frsFDRxParams.CTCSSDecodeBlockLength, ...
    'SampleRate', frsFDRxParams.AudioSampleRate);

% High pass filter to filter out CTCSS tones
audioFilter = frsFDRxParams.AudioFilter;

% Audio device writer
audioPlayer = audioDeviceWriter(frsFDRxParams.AudioSampleRate);



% Perform stream processing if a radio is found.
if radioFound
    % Loop until the example reaches the target stop time.
    timeCounter = 0;

    while timeCounter < frsFDTxParams.StopTime
        % Transmitter stream processing
        % -----------------------------------------------------------------
        dataTx = step(source);  % Generate audio waveform
        dataWTone = dataTx + step(ctcss);  % Add CTCSS tones

        % Interpolation FM modulation
        outResamp = step(interpolator, dataWTone);  % Resample to 200 kHz
        outMod = step(fmMod, outResamp);
        step(radioTx, outMod); % Transmit to USRP(R) radio

        % Receiver stream processing
        % -----------------------------------------------------------------
        [dataRx, lenRx] = step(radioRx);
        if lenRx > 0
            outAGC = step(agc, dataRx); % AGC
            outChanFilt = step(channelFilter, outAGC); % Adjacent channel filtering
            rxAmp = mean(abs(outChanFilt));
            if rxAmp > frsFDRxParams.DetectionThreshold
                outThreshold = outChanFilt;
            else
                outThreshold = complex(single(zeros(frsFDRxParams.RadioFrameLength, 1)));
            end

            % FM demodulation and decimation
            outFMDemod = step(fmDemod, outThreshold); % FM demodulate
            outDecim  = step(decimator, outFMDemod); % Resample to 8 kHz

            % CTCSS decode and conditionally send to audio output
            rcvdCode = step(decoder, outDecim);
            if (rcvdCode == frsFDRxParams.CTCSSCode) || (frsFDRxParams.CTCSSCode == 0)
                rcvdSig = outDecim;
            else
                rcvdSig = single(zeros(frsFDRxParams.AudioFrameLength, 1));
            end

            audioSig = step(audioFilter, rcvdSig); % Filter out CTCSS tones
            step(audioPlayer, audioSig); % Audio output
            timeCounter = timeCounter + frsFDRxParams.RadioFrameTime;
        end
    end
else
    warning(message('sdru:sysobjdemos:MainLoop'))
end

% Release all SDRu and audio resources, FM Modulator and Demodulator
release(fmMod)
release(radioTx)
release(fmDemod)
release(radioRx)
release(audioPlayer)
