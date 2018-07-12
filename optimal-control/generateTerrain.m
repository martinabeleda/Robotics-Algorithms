function [ terrain ] = generateTerrain( width, length, roughness, height )
%GENERATETERRAIN - Create random terrain with certain properties. 
%   params:  width and length: correspond to the width and length of the area.
% 
%            roughness:  A parameter between 0 and 1. Controls how many higher 
%                       frequencies appear in the terrain map. 
%            height:    Controls the height variation in the map.
% 
%                   
%               

M = 40;
gamma = 1.5;
L = width/2;
D = roughness;

% generate terrain parameters in the x direction.
coeffs  = zeros(M,1);
phase = 2*pi*rand(M,1);
freqs = zeros(M,1);

for n = 1:M
    coeffs(n) = sqrt(log(gamma)/M)* (2*pi*gamma^n/L)^(D-1);
    freqs(n) = 1/(length/(2*n));
end

terrain.xCoeffs =height*coeffs;
terrain.xPhase = phase;
terrain.xFreqs = freqs;


% terrain parameters in the y direction.
coeffs  = zeros(M,1);
phase = 2*pi*rand(M,1);
freqs = zeros(M,1);

for n = 1:M
    coeffs(n) = sqrt(log(gamma)/M)* (2*pi*gamma^n/L)^(D-1);
    freqs(n) = 1/(width/(2*n));
end

terrain.yCoeffs =height*coeffs;
terrain.yPhase = phase;
terrain.yFreqs = freqs;


end

