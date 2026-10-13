%% Template Exam Modelling and Control of Manipulators
clc;
close all;
clear;
addpath('include'); 

%% 1. Compute the geometric model
iTj_0 = BuildTree();
disp('------------------------------------------');
disp('1. Link Transformations (iTj_0):');
disp(iTj_0);

jointType = [0 0 0 0 0 1 0]; 
q = [pi/2, -pi/4, 0, -pi/4, 0, 0.15, pi/4]'; % Initial Configuration

%% 2. Define Tool Frame (CORRECT VALUES)
e_eta_t = [pi/10, 0, pi/6]'; 
e_O_t = [0.3, 0.1, 0.0]';   
eRt = YPRToRot(e_eta_t(1), e_eta_t(2), e_eta_t(3));
eTt = [eRt e_O_t; 0 0 0 1];

disp('2. Tool Transformation Matrix (eTt):');
disp(eTt);

%% 3. Initialize Models
gm = geometricModel(iTj_0, jointType, eTt);
gm.updateDirectGeometry(q);
km = kinematicModel(gm);

bTt = gm.getToolTransformWrtBase();
disp('3. Initial Tool Pose w.r.t Base (bTt initial):');
disp(bTt);

%% 4. Define Goal Frame (CORRECT VALUES - Plan B)
b_eta_g = [0, 1.57, 0]';   
b_O_g = [0.2; -0.8; 0.3]; 

bRg = YPRToRot(b_eta_g(1), b_eta_g(2), b_eta_g(3));
bTg = [bRg b_O_g; 0 0 0 1];

disp('4. Goal Transformation Matrix (bTg):');
disp(bTg);

% Gains
k_a = 0.8;
k_l = 0.8;
cc = cartesianControl(gm, k_a, k_l);

%% 5. Simulation Setup
samples = 600;
t_start = 0.0;
t_end = 15.0;
dt = (t_end-t_start)/samples;
t = t_start:dt:t_end; 

qmin = -3.14 * ones(7,1); qmin(6) = 0;
qmax = +3.14 * ones(7,1); qmax(6) = 1;

% Initialize Plot Class
show_simulation = true;
pm = plotManipulators(show_simulation);
pm.initMotionPlot(t, bTg(1:3,4));

disp('------------------------------------------');
disp('STARTING SIMULATION...');

%%%%%%% Kinematic Simulation Loop %%%%%%%
for i = t
    % Update Models
    gm.updateDirectGeometry(q);
    km.updateJacobian(); 
    
    % Get Reference Velocity
    x_dot = cc.getCartesianReference(bTg); 
    
    % Inverse Kinematics
    q_dot = pinv(km.J) * x_dot; 
    
    % Simulation Integration
    q = KinematicSimulation(q, q_dot, dt, qmin, qmax);
    
    % Plotting (Updates history inside the class)
    pm.plotIter(gm, km, i, q_dot);

    % --- REPORTING SECTION ---
    if i == t(1) % Initial Report
        disp('====== INITIAL CONFIGURATION REPORT ======');
        e_p = x_dot(4:6)/k_l; e_o = x_dot(1:3)/k_a;
        disp('Position Error (ep):'); disp(e_p);
        disp('Orientation Error (eo):'); disp(e_o);
        fprintf('Position Error Norm: %.4f\n', norm(e_p));
        fprintf('Orientation Error Norm: %.4f\n', norm(e_o));
    end

    if(norm(x_dot(1:3)) < 0.01 && norm(x_dot(4:6)) < 0.01) % Convergence
        disp(' ');
        disp('====== FINAL CONFIGURATION REPORT ======');
        e_p = x_dot(4:6)/k_l; e_o = x_dot(1:3)/k_a;
        disp('Position Error (ep):'); disp(e_p);
        disp('Orientation Error (eo):'); disp(e_o);
        fprintf('Position Error Norm: %.4f\n', norm(e_p));
        fprintf('Orientation Error Norm: %.4f\n', norm(e_o));
        disp('Reached Requested Pose (Converged)');
        break;
    end
end

% This function inside the class will generate the Velocity Plot automatically
pm.plotFinalConfig(gm);