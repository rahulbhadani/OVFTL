%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++%
%   Author: Rahul Kumar Bhadani  
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++%

function[patchHandle] = plotCustom(xData, yData, markerDataX, markerDataY, AX, AY, BX, BY, CX, CY, DX, DY, markerSize)


xData = reshape(xData, length(xData),1);
yData = reshape(yData, length(yData),1);

FirstX = xData(1);
FirstY = yData(1);
FirstMarkerDataX = markerSize * 1.3 * reshape(markerDataX,1,length(markerDataX)) ;
FirstMarkerDataY = markerSize * 1.3 * reshape(markerDataY,1,length(markerDataY)) ;


FirstVertexX = zeros(length(FirstMarkerDataX),length(FirstX));
FirstVertexY = zeros(length(FirstMarkerDataY),length(FirstX));

FirstVertexX = FirstVertexX(:);
FirstVertexY = FirstVertexY(:);

k = 1;
for m = 1:length(FirstMarkerDataX)
        theta = atan(FirstY/FirstX);
        if(FirstX < 0)
            theta = theta + pi;
        end
        FirstVertexX(k,1) = ((FirstMarkerDataX(m)).*cos(theta)) - ((FirstMarkerDataY(m)).*sin(theta)) + FirstX;
        FirstVertexY(k,1) = ((FirstMarkerDataX(m)).*sin(theta)) + ((FirstMarkerDataY(m)).*cos(theta)) + FirstY;
        k = k + 1;
end


FirstFaces = 1:length(FirstMarkerDataX)*length(FirstX);
FirstF = zeros(length(FirstX),length(FirstMarkerDataX));
k = 1;
j = 1;
for i = 1:length(FirstFaces)
    if k == 8
        j = j + 1;
        k = 1;
    end
    FirstF(j,k) = FirstFaces(i);
    k = k + 1;

end

%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++%
%==========================================================================%
%==========================================================================%
markerDataX = markerSize * reshape(markerDataX,1,length(markerDataX)) ;
markerDataY = markerSize * reshape(markerDataY,1,length(markerDataY)) ;
%You have to create a matrix

VertexX = zeros(length(markerDataX),length(xData));
VertexY = zeros(length(markerDataY),length(yData));

VertexX = VertexX(:);
VertexY = VertexY(:);

k = 1;
for i = 1:length(xData)
    for m = 1:length(markerDataX)
%        vX = markerDataX(m) + xData(i);
%        vY = markerDataY(m) + yData(i);
%         if i > (length(xData)/2)
%             vY = -markerDataY(m) - yData(i);
%         end
        theta = atan(yData(i)/xData(i));
        if(xData(i) < 0)
            theta = theta + pi;
        end
        VertexX(k,1) = ((markerDataX(m)).*cos(theta)) - ((markerDataY(m)).*sin(theta)) + xData(i);
        VertexY(k,1) = ((markerDataX(m)).*sin(theta)) + ((markerDataY(m)).*cos(theta)) + yData(i);

        %VertexX(k,1) = (vX*cos(theta)) - (vY*sin(theta));
        %VertexY(k,1) = (vX*sin(theta)) + (vY*cos(theta));
        k = k + 1;
    end 
end


faces = 1:length(markerDataX)*length(xData);
f = zeros(length(xData),length(markerDataX));
k = 1;
j = 1;
for i = 1:length(faces)
    if k == 8
        j = j + 1;
        k = 1;
    end
    f(j,k) = faces(i);
    k = k + 1;

end
%==========================================================================%
%++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
AX = markerSize * reshape(AX,1,length(AX)) ;
AY = markerSize * reshape(AY,1,length(AY)) ;
%You have to create a matrix

VertexAX = zeros(length(AX),length(xData));
VertexAY = zeros(length(AY),length(yData));

VertexAX = VertexAX(:);
VertexAY = VertexAY(:);

k = 1;
for i = 1:length(xData)
    for m = 1:length(AX)
        theta = atan(yData(i)/xData(i));
        if(xData(i) < 0)
            theta = theta + pi;
        end
        VertexAX(k,1) = ((AX(m)).*cos(theta)) - ((AY(m)).*sin(theta)) + xData(i);
        VertexAY(k,1) = ((AX(m)).*sin(theta)) + ((AY(m)).*cos(theta)) + yData(i);
        k = k + 1;
    end 
end


facesA = 1:length(AX)*length(xData);
fA = zeros(length(xData),length(AX));
k = 1;
j = 1;
for i = 1:length(facesA)
    if k == 5
        j = j + 1;
        k = 1;
    end
    fA(j,k) = facesA(i);
    k = k + 1;

end
%==========================================================================%
BX = markerSize * reshape(BX,1,length(BX)) ;
BY = markerSize * reshape(BY,1,length(BY)) ;
%You have to create a matrix

VertexBX = zeros(length(BX),length(xData));
VertexBY = zeros(length(BY),length(yData));

VertexBX = VertexBX(:);
VertexBY = VertexBY(:);

k = 1;
for i = 1:length(xData)
    for m = 1:length(BX)
        theta = atan(yData(i)/xData(i));
        if(xData(i) < 0)
            theta = theta + pi;
        end
        VertexBX(k,1) = ((BX(m)).*cos(theta)) - ((BY(m)).*sin(theta)) + xData(i);
        VertexBY(k,1) = ((BX(m)).*sin(theta)) + ((BY(m)).*cos(theta)) + yData(i);
        k = k + 1;
    end 
end


facesB = 1:length(BX)*length(xData);
fB = zeros(length(xData),length(BX));
k = 1;
j = 1;
for i = 1:length(facesB)
    if k == 5
        j = j + 1;
        k = 1;
    end
    fB(j,k) = facesB(i);
    k = k + 1;

end
%==========================================================================%
CX = markerSize * reshape(CX,1,length(CX)) ;
CY = markerSize * reshape(CY,1,length(CY)) ;
%You have to create a matrix

VertexCX = zeros(length(CX),length(xData));
VertexCY = zeros(length(CY),length(yData));

VertexCX = VertexCX(:);
VertexCY = VertexCY(:);

k = 1;
for i = 1:length(xData)
    for m = 1:length(CX)
        theta = atan(yData(i)/xData(i));
        if(xData(i) < 0)
            theta = theta + pi;
        end
        VertexCX(k,1) = ((CX(m)).*cos(theta)) - ((CY(m)).*sin(theta)) + xData(i);
        VertexCY(k,1) = ((CX(m)).*sin(theta)) + ((CY(m)).*cos(theta)) + yData(i);
        k = k + 1;
    end 
end


facesC = 1:length(CX)*length(xData);
fC = zeros(length(xData),length(CX));
k = 1;
j = 1;
for i = 1:length(facesC)
    if k == 5
        j = j + 1;
        k = 1;
    end
    fC(j,k) = facesC(i);
    k = k + 1;

end
%==========================================================================%
DX = markerSize * reshape(DX,1,length(DX)) ;
DY = markerSize * reshape(DY,1,length(DY)) ;
%You have to create a matrix

VertexDX = zeros(length(DX),length(xData));
VertexDY = zeros(length(DY),length(yData));

VertexDX = VertexDX(:);
VertexDY = VertexDY(:);

k = 1;
for i = 1:length(xData)
    for m = 1:length(DX)
        theta = atan(yData(i)/xData(i));
        if(xData(i) < 0)
            theta = theta + pi;
        end
        VertexDX(k,1) = ((DX(m)).*cos(theta)) - ((DY(m)).*sin(theta)) + xData(i);
        VertexDY(k,1) = ((DX(m)).*sin(theta)) + ((DY(m)).*cos(theta)) + yData(i);
        k = k + 1;
    end 
end


facesD = 1:length(DX)*length(xData);
fD = zeros(length(xData),length(DX));
k = 1;
j = 1;
for i = 1:length(facesD)
    if k == 5
        j = j + 1;
        k = 1;
    end
    fD(j,k) = facesD(i);
    k = k + 1;

end
%==========================================================================%
%==========================================================================%


plot(xData, yData,'.')

axis equal;
axis([-10 10 -10 10])
%==========================================================================%
%==========================================================================%
patchcol = 1:size(VertexX);
patchcol = patchcol';
patchHandle = patch('Faces',f,'Vertices',[VertexX VertexY]);

set(patchHandle,'FaceColor','interp','FaceVertexCData',patchcol) ;

%==========================================================================%
FirstpatchHandle = patch('Faces',FirstF,'Vertices',[FirstVertexX FirstVertexY]);

set(FirstpatchHandle,'FaceColor','red') ;

%+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

patchHandle2 = patch('Faces',fA,'Vertices',[VertexAX VertexAY]);

set(patchHandle2,'FaceColor','black') ;
%==========================================================================%

patchHandle3 = patch('Faces',fB,'Vertices',[VertexBX VertexBY]);

set(patchHandle3,'FaceColor','black') ;
%==========================================================================%

patchHandle4 = patch('Faces',fC,'Vertices',[VertexCX VertexCY]);

set(patchHandle4,'FaceColor','black') ;
%==========================================================================%

patchHandle5 = patch('Faces',fD,'Vertices',[VertexDX VertexDY]);

set(patchHandle5,'FaceColor','black') ;

%==========================================================================%
%==========================================================================%
drawnow;
