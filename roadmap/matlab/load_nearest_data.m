function [N, D] = load_nearest_data()

% N = load('../data/nearest.txt')+1;
% D = load('../data/distances.txt');

fileN = '../data/nearest.txt';
fN = fopen(fileN);
fileD = '../data/distances.txt';
fD = fopen(fileD);

i = 1;
tlineN = fgets(fN);
tlineD = fgets(fD);
while ischar(tlineN)
    N{i} = str2num(tlineN) + 1;
    tlineN = fgets(fN);
    
    D{i} = str2num(tlineD);
    tlineD = fgets(fD);
        
    i = i + 1;
end

N = N';
D = D';
% D = [];
