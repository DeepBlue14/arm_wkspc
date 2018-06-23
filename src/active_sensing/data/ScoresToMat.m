clc; close('all');

%% Save text data to Matlab data file.

scores=dlmread('scores.txt');
phi=dlmread('scoresPhi.txt');
theta=dlmread('scoresTheta.txt');
positionDatabase=zeros(numel(scores),3);
scoreDatabase=zeros(1,numel(scores));

for i=1:length(phi)
    for j=1:length(theta)
        x = cos(theta(j))*cos(phi(i));
        y = sin(theta(j))*cos(phi(i));
        z = sin(phi(i));
        idx = length(theta)*(i-1)+j;
        positionDatabase(idx,:) = [x,y,z];
        scoreDatabase(idx) = scores(i,j);
    end
end

save('ActiveLandscape-2016-04-25.mat','positionDatabase','scoreDatabase');