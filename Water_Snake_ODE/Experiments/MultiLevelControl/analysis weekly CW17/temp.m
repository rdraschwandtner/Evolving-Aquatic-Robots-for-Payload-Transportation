clear all;
close all;

cd rev156wc156\04-22-2015-17-20-00
stopping_bodylength_d50();
cd ..\..
cd rev162wc162\
stopping_bodylength_d50();
cd ..
cd rev163wc163\
stopping_bodylength_d50();
cd ..
cd rev166wc166\
stopping_bodylength_d50();
cd ..

legendstr = {'1. original (100Nm, 0.6Hz)','2. hydrodynamic update (100Nm, 0.6Hz)','3. increased ODE Fmax (1000Nm, 0.6Hz)', '4. increas freq (1000Nm, 2Hz)'};

figure(1);
legend(legendstr);
figure(2);
legend(legendstr);
figure(3);
legend(legendstr);