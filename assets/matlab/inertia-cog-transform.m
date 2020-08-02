clear; close all; clc;
format long

% values for link 1 [kg m^2]

I1xx =  7.0337e-01;
I1xy = -1.3900e-04;
I1xz =  6.7720e-03;
I1yy =  7.0661e-01;
I1yz =  1.9169e-02;
I1zz =  9.1170e-03;

c1x  =   3.875e-03;
c1y  =   2.081e-03;
c1z  =     -0.1750;

m1   =    4.970684;

% computation

% Iq = Ib + m * (q' * q * I - q * q')
% Ib = inertia matrix at COG

Iq = [ I1xx  I1xy  I1xz ;
       I1xy  I1yy  I1yz ;
       I1xz  I1yz  I1zz ]

q  = [c1x c1y c1z]'
m  = m1;

Ib = Iq - m * (q' * q * eye(3) - q * q')
