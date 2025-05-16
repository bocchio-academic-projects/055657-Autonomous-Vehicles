%% ASSIGNMENT 3

rosinit
gazebo = ExampleHelperGazeboCommunicator;
% add objects in GAZEBO
redSphere = ExampleHelperGazeboModel("RedSphere");
addLink(redSphere,"sphere",0.1,"color",[200/255 48/255 48/255 1]); %c83030
spawnModel(gazebo,redSphere,[6 1 0.1]);


PurpleSphere = ExampleHelperGazeboModel("PurpleSphere");
addLink(PurpleSphere,"sphere",0.1,"color",[200/255 0/255 103/255 1]); %c80067
spawnModel(gazebo,PurpleSphere,[6 0 0.1]);


%% Plot
My_bag = rosbag('Ass_3.bag');
My_bag.AvailableTopics % to see the topics
Odom_ = select(My_bag,'Topic','/odom');
OdomMsgs_ = readMessages(Odom_,'DataFormat','struct');
X_ = arrayfun(@(msg) (msg{1}.Pose.Pose.Position.X),OdomMsgs_);
Y_ = arrayfun(@(msg) (msg{1}.Pose.Pose.Position.Y),OdomMsgs_);
% Creazione del plot
figure;
plot(X_, Y_,'LineWidth', 2); % '-o' aggiunge un cerchio per ogni punto
hold on; % Mantiene il grafico per aggiungere altri elementi

plot(6, 1, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 10);

% Etichette e titolo
xlabel('X');
ylabel('Y');
title('Trajectory');
grid on; % Aggiunge la griglia al grafico

xlim([-2 7]); % Imposta l'asse X da -3 a 3
ylim([-3 3]); % Imposta l'asse Y da -10 a 10

%% PLOT
t = out.x_sfera.Time(1:67);
x = out.x_sfera.Data(1:67); 
y = out.simout(1,2,1:67);  
y_values = squeeze(y); 
t_ = out.tout(1:67);

figure;
plot(t, x, '-o', 'LineWidth', 2, 'MarkerSize', 6, 'Color', 'r');
hold on
plot(t_, y_values, '-o', 'LineWidth', 2, 'MarkerSize', 6, 'Color', 'b');
xlabel('Time');  
ylabel('X and Y'); 
title('Centroid position');
legend('X position','Y position')
axis([min(t) max(t) -50 500]);  % Impostazione corretta dei limiti dell'asse y
grid on;  
