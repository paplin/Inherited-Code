max = 0;
maxIndex = 0;

transmitterArray = sdrqpsktx.MessageBits;
receivedArray = ReceivedData(:,1,140);
% receivedArray = receivedArray(107:end)
x= sdrqpsktx.MessageBits(1:112);
% x= receivedArray(1:112);
aux = [];

for k=0:size(receivedArray)-112
    y =(1+k:112+k);
%     y = sdrqpsktx.MessageBits(1+k:112+k);
    aux(k+1) = dot(x,y)/(norm(x)*norm(y));
    
%     if aux>max
%         max = aux;
%         maxIndex=k;
%     end
end

plot(aux);
% max
% maxIndex