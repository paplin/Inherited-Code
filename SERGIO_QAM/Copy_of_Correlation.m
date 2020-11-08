max = 0;
maxIndex = 0;

transmitterArray = sdrqpskrx.BarkerCode;
receivedArray = antesdepramble.signals.values(:,1,200);
% transmitterArray = (-1-j)*(sdrqpskrx.BarkerCode)';
% receivedArray = receivedArray(107:end)
x= transmitterArray;
% x= receivedArray(1:112);
aux = [];

for k=0:size(receivedArray)-13
    y = receivedArray(1+k:13+k);
%     y = sdrqpsktx.MessageBits(1+k:112+k);
    aux(k+1) = abs(dot(x,y)/(norm(x)*norm(y)));
    
    if aux>max
        max = aux;
        maxIndex=k;
    end
end

plot(aux);
max
maxIndex