signal = antesdepramble.signals.values(:,1,80);

% BarkerCoding = BarkerQam(:,1,2);

% prbdet = comm.PreambleDetector(BarkerCoding,'Input','Symbol','Threshold',11);

%%QPSK

prbdet = comm.PreambleDetector(Barker,'Input','Symbol','Threshold',10.5);

[idx,detmet] = step(prbdet,signal(1:2813));
idx


[a,b] = max(detmet)