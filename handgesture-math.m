clear; clc; close all;

%% ================= PARAMETER GLOBAL =================
L_upper_arm  = 30;
L_forearm    = 25;
L_palm       = 9;
W_palm       = 8;
T_palm       = 2;

finger_names = {'Jempol','Telunjuk','Tengah','Manis','Kelingking'};

finger_link_lengths = {
    [4.0, 3.0, 2.5];
    [5.0, 3.5, 2.5];
    [5.5, 4.0, 3.0];
    [5.0, 3.5, 2.5];
    [4.5, 3.0, 2.0]
};

finger_base_pos = {
    [ 2.0, -W_palm/2-1.0, 0];
    [ L_palm*0.9,  W_palm*0.35, 0];
    [ L_palm*0.95, W_palm*0.10, 0];
    [ L_palm*0.90, -W_palm*0.15, 0];
    [ L_palm*0.75, -W_palm*0.40, 0]
};

finger_base_rot = {
    roty(-pi/2.6)*rotz(pi/2.2);
    rotz(0)*roty(pi/12);
    rotz(0)*roty(pi/12);
    rotz(0)*roty(pi/12);
    rotz(-pi/16)*roty(pi/10)
};

%% ================= MODEL LENGAN 7-DOF =================
L(1) = Link('d',0,'a',0,'alpha',-pi/2,'offset',0);
L(2) = Link('d',0,'a',0,'alpha', pi/2,'offset',0);
L(3) = Link('d',L_upper_arm,'a',0,'alpha',-pi/2,'offset',0);
L(4) = Link('d',0,'a',0,'alpha', pi/2,'offset',0);
L(5) = Link('d',L_forearm,'a',0,'alpha',-pi/2,'offset',0);
L(6) = Link('d',0,'a',0,'alpha', pi/2,'offset',0);
L(7) = Link('d',6,'a',0,'alpha',0,'offset',0);

for i = 1:7
    L(i).qlim = [-2.8, 2.8];
end
L(3).qlim = [-2.6, 0.05];

arm = SerialLink(L,'name','Lengan Manusia');
arm.base = transl(0,0,0);

%% ================= FUNGSI PEMBUATAN JARI =================
function finger = create_finger(link_lengths, name_prefix, is_thumb)
    n = length(link_lengths) + 1;
    Ln(1) = Link('d',0,'a',0,'alpha',pi/2,'offset',0);
    Ln(1).qlim = [-pi/6, pi/6];
    for i = 1:length(link_lengths)
        Ln(i+1) = Link('d',0,'a',link_lengths(i),'alpha',0,'offset',0);
        if i == 1
            Ln(i+1).qlim = [0, pi/2.1];
        else
            Ln(i+1).qlim = [0, pi/1.8];
        end
    end
    if is_thumb
        Ln(1).qlim = [-pi/3, pi/3];
        Ln(2).qlim = [0, pi/2.5];
    end
    finger = SerialLink(Ln,'name',[name_prefix ' ' num2str(n) 'DOF']);
end

fingers = cell(1,5);
for i = 1:5
    fingers{i} = create_finger(finger_link_lengths{i}, finger_names{i}, i==1);
    fingers{i}.base = transl(finger_base_pos{i}) * finger_base_rot{i};
end

%% ================= DEFINISI POSE TANGAN =================
n_joints = zeros(1,5);
for i = 1:5, n_joints(i) = fingers{i}.n; end

pose_open   = cell(1,5);
pose_fist   = cell(1,5);
pose_point  = cell(1,5);
pose_pinch  = cell(1,5);
pose_ok     = cell(1,5);
pose_peace  = cell(1,5);
pose_thumb  = cell(1,5);

for i = 1:5
    pose_open{i}  = zeros(1,n_joints(i));
    pose_fist{i}  = [0, ones(1,n_joints(i)-1)*(pi/2)];
    pose_point{i} = [0, ones(1,n_joints(i)-1)*(pi/2)];
    pose_pinch{i} = [0, ones(1,n_joints(i)-1)*(pi/3)];
    pose_ok{i}    = [0, ones(1,n_joints(i)-1)*(pi/2.5)];
    pose_peace{i} = [0, ones(1,n_joints(i)-1)*(pi/2)];
    pose_thumb{i} = [0, ones(1,n_joints(i)-1)*(pi/2)];
end

pose_point{2} = zeros(1,n_joints(2));
pose_point{1} = [pi/8, pi/3, pi/2.5];

pose_peace{2} = zeros(1,n_joints(2));
pose_peace{3} = zeros(1,n_joints(3));
pose_peace{1} = [pi/6, pi/2.2, pi/2];

pose_pinch{1} = [pi/5, pi/2.3, pi/2];
pose_pinch{2} = [0, pi/3, pi/2.5, pi/2.5];

pose_ok{1} = [pi/4, pi/2, pi/1.9];
pose_ok{2} = [0, pi/2.6, pi/1.8, pi/1.8];

pose_thumb{1} = [0, 0, 0];
for i = 2:5
    pose_thumb{i} = [0, ones(1,n_joints(i)-1)*(pi/1.7)];
end

poses = struct('open',{pose_open},'fist',{pose_fist},'point',{pose_point}, ...
    'pinch',{pose_pinch},'ok',{pose_ok},'peace',{pose_peace},'thumbsup',{pose_thumb});
pose_names = {'open','fist','point','pinch','ok','peace','thumbsup'};

%% ================= FUNGSI GABUNGAN PLOT LENGAN+TANGAN =================
function draw_hand(arm, fingers, finger_base_pos, finger_base_rot, q_arm, q_fingers, ax_lims)
    cla;
    arm.plot(q_arm, 'workspace', ax_lims, 'noraise', 'nobase', 'notiles', 'jointdiam', 0.6);
    hold on;
    T_ee = arm.fkine(q_arm);
    colors = {'r','g','b','m','c'};
    for i = 1:5
        T_finger_base = T_ee * transl(finger_base_pos{i}) * finger_base_rot{i};
        fingers{i}.base = T_finger_base;
        fingers{i}.plot(q_fingers{i}, 'workspace', ax_lims, 'noraise', 'nobase', ...
            'notiles', 'jointdiam', 0.5);
        T_tip = fingers{i}.fkine(q_fingers{i});
        p = T_tip.t;
        plot3(p(1), p(2), p(3), 'o', 'MarkerSize', 8, 'MarkerFaceColor', colors{i}, 'MarkerEdgeColor','k');
    end
    hold off;
end

%% ================= POSISI AWAL DAN VISUALISASI STATIS =================
q_arm_init = [0.2, -0.5, -1.4, 0.9, 0.3, 0.6, 0];
ax_lims = [-40, 60, -50, 50, -30, 70];

figure('Name','Lengan dan Tangan Robot Humanoid','Position',[80,80,1000,800]);
draw_hand(arm, fingers, finger_base_pos, finger_base_rot, q_arm_init, poses.open, ax_lims);
title('Postur Awal: Tangan Terbuka');
view(45,20); grid on; axis equal;

%% ================= ANIMASI TRANSISI ANTAR POSE TANGAN =================
figure('Name','Animasi Perubahan Pose Tangan','Position',[100,100,1000,800]);
sequence = {'open','fist','point','pinch','ok','peace','thumbsup','open'};
steps = 25;

q_prev = poses.open;
for s = 1:length(sequence)-1
    q_from = poses.(sequence{s});
    q_to   = poses.(sequence{s+1});
    for t = linspace(0,1,steps)
        q_interp = cell(1,5);
        for i = 1:5
            q_interp{i} = q_from{i} + t*(q_to{i} - q_from{i});
        end
        draw_hand(arm, fingers, finger_base_pos, finger_base_rot, q_arm_init, q_interp, ax_lims);
        title(sprintf('Pose: %s -> %s (t=%.2f)', sequence{s}, sequence{s+1}, t));
        view(45,20); grid on; axis equal;
        drawnow;
    end
end

%% ================= INVERSE KINEMATICS LENGAN MENUJU TARGET =================
target_pos = [35, 15, 25];
T_target = transl(target_pos) * trotx(pi);

q_ik_mask = [1 1 1 1 1 1 0];
try
    q_target_arm = arm.ikine(T_target, 'q0', q_arm_init, 'mask', [1 1 1 0 0 0]);
catch
    q_target_arm = q_arm_init;
end

q_traj = jtraj(q_arm_init, q_target_arm, 40);

figure('Name','Animasi Gerakan Meraih Target','Position',[120,120,1000,800]);
for k = 1:size(q_traj,1)
    draw_hand(arm, fingers, finger_base_pos, finger_base_rot, q_traj(k,:), poses.open, ax_lims);
    hold on;
    plot3(target_pos(1), target_pos(2), target_pos(3), 'p', 'MarkerSize', 14, ...
        'MarkerFaceColor','y', 'MarkerEdgeColor','k');
    hold off;
    title(sprintf('Meraih Target (langkah %d/%d)', k, size(q_traj,1)));
    view(45,20); grid on; axis equal;
    drawnow;
end

for t = linspace(0,1,steps)
    q_close = cell(1,5);
    for i = 1:5
        q_close{i} = poses.open{i} + t*(poses.fist{i} - poses.open{i});
    end
    draw_hand(arm, fingers, finger_base_pos, finger_base_rot, q_target_arm, q_close, ax_lims);
    hold on;
    plot3(target_pos(1), target_pos(2), target_pos(3), 'p', 'MarkerSize', 14, ...
        'MarkerFaceColor','y', 'MarkerEdgeColor','k');
    hold off;
    title(sprintf('Menggenggam Objek (t=%.2f)', t));
    view(45,20); grid on; axis equal;
    drawnow;
end

%% ================= SIMULASI MENGGENGGAM OBJEK BULAT =================
function draw_sphere(center, radius, color)
    [X,Y,Z] = sphere(20);
    surf(X*radius+center(1), Y*radius+center(2), Z*radius+center(3), ...
        'FaceColor', color, 'EdgeColor', 'none', 'FaceAlpha', 0.8);
end

obj_center = target_pos;
obj_radius = 3;

figure('Name','Genggaman Objek Bulat','Position',[140,140,1000,800]);
draw_hand(arm, fingers, finger_base_pos, finger_base_rot, q_target_arm, poses.fist, ax_lims);
hold on;
draw_sphere(obj_center, obj_radius, [0.2, 0.6, 0.9]);
hold off;
title('Genggaman Akhir pada Objek Bulat');
view(45,20); grid on; axis equal; camlight; lighting gouraud;

%% ================= ANALISIS WORKSPACE JARI (OPSIONAL) =================
figure('Name','Workspace Setiap Jari','Position',[160,160,1000,800]);
hold on;
colors = {'r','g','b','m','c'};
n_samples = 400;
for i = 1:5
    n = n_joints(i);
    q_lims = zeros(n,2);
    for j = 1:n
        q_lims(j,:) = fingers{i}.qlim(j,:);
    end
    pts = zeros(n_samples,3);
    for k = 1:n_samples
        q_rand = q_lims(:,1)' + rand(1,n).*(q_lims(:,2)'-q_lims(:,1)');
        T_ee_arm = arm.fkine(q_arm_init);
        fingers{i}.base = T_ee_arm * transl(finger_base_pos{i}) * finger_base_rot{i};
        T = fingers{i}.fkine(q_rand);
        pts(k,:) = T.t';
    end
    scatter3(pts(:,1), pts(:,2), pts(:,3), 6, colors{i}, 'filled');
end
title('Sebaran Workspace Ujung Jari (Sampling Acak)');
xlabel('X'); ylabel('Y'); zlabel('Z');
legend(finger_names, 'Location', 'bestoutside');
view(45,20); grid on; axis equal;
hold off;
