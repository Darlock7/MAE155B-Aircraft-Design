%Path Setup
clc; clear; close all;
repoRoot = fileparts(mfilename('fullpath'));
addpath(genpath(repoRoot));
disp('Project paths loaded.');
disp(['Repo root: ', repoRoot]);
%test