% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly excecuted under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 578.952814872337060 ; 585.355122127640240 ];

%-- Principal point:
cc = [ 645.829043343746890 ; 366.668100334605700 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ -0.177503624372297 ; 0.000000000000000 ; -0.000567136821985 ; 0.004381118840596 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 5.537802906545395 ; 5.922529844110954 ];

%-- Principal point uncertainty:
cc_error = [ 4.131449044202678 ; 3.633085836989010 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.003714894427521 ; 0.000000000000000 ; 0.000863143593786 ; 0.001101711180444 ; 0.000000000000000 ];

%-- Image size:
nx = 1280;
ny = 720;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 19;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 0 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ -2.132771e+00 ; -2.181213e+00 ; 5.178513e-01 ];
Tc_1  = [ -7.315887e+00 ; -3.009157e+00 ; 3.985276e+02 ];
omc_error_1 = [ 9.988446e-03 ; 1.779430e-02 ; 3.373435e-02 ];
Tc_error_1  = [ 2.995888e+00 ; 2.603037e+00 ; 5.417854e+00 ];

%-- Image #2:
omc_2 = [ -2.212968e+00 ; -2.162072e+00 ; 4.564676e-01 ];
Tc_2  = [ -3.335188e+01 ; -4.705033e+00 ; 3.974380e+02 ];
omc_error_2 = [ 1.046137e-02 ; 1.717268e-02 ; 3.253830e-02 ];
Tc_error_2  = [ 2.900194e+00 ; 2.573328e+00 ; 5.400432e+00 ];

%-- Image #3:
omc_3 = [ 2.219866e+00 ; 2.107431e+00 ; 1.953678e-02 ];
Tc_3  = [ -1.314929e+02 ; -1.327186e+01 ; 3.535469e+02 ];
omc_error_3 = [ 1.469206e-02 ; 1.149722e-02 ; 2.707401e-02 ];
Tc_error_3  = [ 2.679311e+00 ; 2.349772e+00 ; 4.953365e+00 ];

%-- Image #4:
omc_4 = [ 2.221282e+00 ; 2.203050e+00 ; 7.152355e-02 ];
Tc_4  = [ -1.038925e+02 ; -2.581643e+01 ; 3.624688e+02 ];
omc_error_4 = [ 1.464439e-02 ; 1.116537e-02 ; 2.755263e-02 ];
Tc_error_4  = [ 2.728307e+00 ; 2.402653e+00 ; 5.191273e+00 ];

%-- Image #5:
omc_5 = [ 2.185144e+00 ; 2.185340e+00 ; 2.740311e-02 ];
Tc_5  = [ -9.970146e+01 ; -2.887794e+01 ; 3.557691e+02 ];
omc_error_5 = [ 1.433206e-02 ; 1.112508e-02 ; 2.685526e-02 ];
Tc_error_5  = [ 2.666333e+00 ; 2.348752e+00 ; 5.052259e+00 ];

%-- Image #6:
omc_6 = [ 2.090730e+00 ; 2.098092e+00 ; -1.017557e+00 ];
Tc_6  = [ -2.161377e+01 ; -1.030767e+02 ; 3.989060e+02 ];
omc_error_6 = [ 9.939992e-03 ; 8.100049e-03 ; 1.738892e-02 ];
Tc_error_6  = [ 2.895733e+00 ; 2.482810e+00 ; 3.556995e+00 ];

%-- Image #7:
omc_7 = [ 2.110293e+00 ; 2.112674e+00 ; -1.015102e+00 ];
Tc_7  = [ -1.857882e+01 ; -1.085615e+02 ; 3.959786e+02 ];
omc_error_7 = [ 1.001708e-02 ; 8.093311e-03 ; 1.760422e-02 ];
Tc_error_7  = [ 2.891512e+00 ; 2.457449e+00 ; 3.547118e+00 ];

%-- Image #8:
omc_8 = [ -2.048116e+00 ; -2.070499e+00 ; 9.090956e-01 ];
Tc_8  = [ -3.770211e+01 ; -1.243993e+02 ; 3.887216e+02 ];
omc_error_8 = [ 9.332360e-03 ; 7.964602e-03 ; 1.723797e-02 ];
Tc_error_8  = [ 2.842869e+00 ; 2.391939e+00 ; 3.604142e+00 ];

%-- Image #9:
omc_9 = [ -2.030420e+00 ; -2.040139e+00 ; 8.158474e-01 ];
Tc_9  = [ -4.899296e+01 ; -1.344072e+02 ; 3.781471e+02 ];
omc_error_9 = [ 9.108406e-03 ; 7.954189e-03 ; 1.711946e-02 ];
Tc_error_9  = [ 2.772693e+00 ; 2.308018e+00 ; 3.635519e+00 ];

%-- Image #10:
omc_10 = [ -1.815135e+00 ; -1.965596e+00 ; 6.205285e-01 ];
Tc_10  = [ -1.009052e+02 ; -1.026544e+02 ; 3.342500e+02 ];
omc_error_10 = [ 6.919014e-03 ; 7.582307e-03 ; 1.293214e-02 ];
Tc_error_10  = [ 2.380799e+00 ; 2.018558e+00 ; 3.215173e+00 ];

%-- Image #11:
omc_11 = [ 1.983251e+00 ; 2.329741e+00 ; 9.846505e-02 ];
Tc_11  = [ -1.124712e+01 ; -7.676134e+01 ; 2.692703e+02 ];
omc_error_11 = [ 9.315663e-03 ; 7.399197e-03 ; 1.692185e-02 ];
Tc_error_11  = [ 2.034749e+00 ; 1.756247e+00 ; 3.546525e+00 ];

%-- Image #12:
omc_12 = [ 1.967350e+00 ; 2.267137e+00 ; 1.736744e-02 ];
Tc_12  = [ -1.794370e+01 ; -8.719603e+01 ; 3.088667e+02 ];
omc_error_12 = [ 1.038739e-02 ; 8.814427e-03 ; 1.842845e-02 ];
Tc_error_12  = [ 2.295028e+00 ; 1.977541e+00 ; 4.063915e+00 ];

%-- Image #13:
omc_13 = [ -2.213144e+00 ; -2.221814e+00 ; 1.194947e-01 ];
Tc_13  = [ -8.955251e+01 ; -1.149833e+02 ; 1.933172e+02 ];
omc_error_13 = [ 5.555242e-03 ; 6.286907e-03 ; 1.197316e-02 ];
Tc_error_13  = [ 1.408958e+00 ; 1.224301e+00 ; 2.341757e+00 ];

%-- Image #14:
omc_14 = [ -2.239012e+00 ; -2.135825e+00 ; 1.582920e-01 ];
Tc_14  = [ -1.422943e+02 ; -9.888031e+01 ; 2.090240e+02 ];
omc_error_14 = [ 5.810710e-03 ; 5.624208e-03 ; 1.149200e-02 ];
Tc_error_14  = [ 1.468435e+00 ; 1.346341e+00 ; 2.507119e+00 ];

%-- Image #15:
omc_15 = [ -2.203428e+00 ; -2.113881e+00 ; 2.348862e-01 ];
Tc_15  = [ -1.645638e+02 ; -8.808429e+01 ; 2.174733e+02 ];
omc_error_15 = [ 6.106717e-03 ; 5.713244e-03 ; 1.135618e-02 ];
Tc_error_15  = [ 1.475539e+00 ; 1.397414e+00 ; 2.595541e+00 ];

%-- Image #16:
omc_16 = [ -2.000844e+00 ; -1.952995e+00 ; 4.524531e-01 ];
Tc_16  = [ -2.055991e+02 ; -9.513276e+01 ; 2.846747e+02 ];
omc_error_16 = [ 7.089822e-03 ; 5.936698e-03 ; 1.122012e-02 ];
Tc_error_16  = [ 1.939960e+00 ; 1.845080e+00 ; 3.087601e+00 ];

%-- Image #17:
omc_17 = [ -1.921808e+00 ; -1.981118e+00 ; -4.613832e-01 ];
Tc_17  = [ -1.298243e+02 ; -7.737683e+01 ; 1.241174e+02 ];
omc_error_17 = [ 4.878757e-03 ; 6.240252e-03 ; 8.998633e-03 ];
Tc_error_17  = [ 1.022159e+00 ; 1.024769e+00 ; 1.852628e+00 ];

%-- Image #18:
omc_18 = [ -1.909956e+00 ; -2.040665e+00 ; -4.062002e-01 ];
Tc_18  = [ -1.174073e+02 ; -7.658039e+01 ; 1.268560e+02 ];
omc_error_18 = [ 4.687194e-03 ; 6.367414e-03 ; 8.908594e-03 ];
Tc_error_18  = [ 1.029530e+00 ; 1.007833e+00 ; 1.774018e+00 ];

%-- Image #19:
omc_19 = [ 1.900040e+00 ; 1.792134e+00 ; -7.681256e-01 ];
Tc_19  = [ -6.143709e+01 ; -7.727657e+01 ; 3.162941e+02 ];
omc_error_19 = [ 7.285386e-03 ; 6.367021e-03 ; 1.155416e-02 ];
Tc_error_19  = [ 2.108123e+00 ; 1.967105e+00 ; 2.819200e+00 ];

