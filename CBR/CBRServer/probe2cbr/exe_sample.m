function [] = exe_sample();

numberOfOutputSensors = 43;
probe2CBR('data/probe12sal.txt','data/probe12sal.cbr.txt');
probe2CBR('data/probe09ent.txt','data/probe09ent.cbr.txt', numberOfOutputSensors);
