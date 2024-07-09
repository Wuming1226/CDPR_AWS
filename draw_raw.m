clear
clc

load("raw.mat")
load("raw_add.mat")

A1 = [0.342, 0.342, 0.727];
A2 = [-0.342, 0.342, 0.727];
A3 = [-0.342, -0.342, 0.727];

x_num = 20;
y_num = 20;
z_num = 20;

x_step_len = (A1(1) - A2(1) - 0.02) / (x_num - 1);
y_step_len = (A1(2) - A3(2) - 0.02) / (y_num - 1);
z_step_len = (A1(3) - 0.01) / (z_num - 1);

raw_max = max([max(max(max(raw_matrix))), max(max(max(raw_matrix_add)))]);


figure(1)
% colormap(slanCM('guppy', 'truncation', [60, 220]))
colormap(slanCM('parula', 'truncation', [0, round(max(max(max(raw_matrix))) / raw_max * 256)]));

for x_step=0:(x_num-1)
    for y_step=0:(x_num-1)
        for z_step=0:(x_num-1)

            x = A2(1) + 0.01 + x_step * x_step_len;
            y = A3(2) + 0.01 + y_step * y_step_len;
            z = 0 + z_step * z_step_len;

            raw = raw_matrix(x_step+1, y_step+1, z_step+1);
            if raw > 0
                size = raw * 10;
                color = raw;
                edgecolor = [0 0.4470 0.7410];
                
                scatter3(x, y, z, size, color, 'o', 'filled', MarkerFaceColor='flat', MarkerEdgeColor=edgecolor, MarkerFaceAlpha=.8)
                hold on
            end
            

        end
    end
end

set(gcf,'unit','normalized','position', [0, 0, 0.45, 0.5])

c = colorbar('Ticks', [0, 1.5, 3, 4.5, 6, 7.5, 9], 'FontSize', 20);
c.Label.String = 'r_{AW} (N)';
c.Label.FontSize = 24;
c.Label.FontName = "Times New Roman";
c.Position = [0.81, 0.1, 0.025, 0.8];

set(gca,'FontSize',20)
set(gca,'unit','normalized','position', [0.1, 0.1, 0.65, 0.9])
xlabel('x (m)','FontName','Times New Roman','FontSize',24,'Rotation',-25)  
ylabel('y (m)','FontName','Times New Roman','FontSize',24,'Rotation',32)
zlabel('z (m)','FontName','Times New Roman','FontSize',24,'Rotation',90)

axis equal;

axis([-0.4, 0.4, -0.4, 0.4, 0, 0.8])
view(40, 30)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(2)
colormap(slanCM('parula', 'truncation', [0, round(max(max(max(raw_matrix_add))) / raw_max * 256)]));

for x_step=0:(x_num-1)
    for y_step=0:(x_num-1)
        for z_step=0:(x_num-1)

            x = A2(1) + 0.01 + x_step * x_step_len;
            y = A3(2) + 0.01 + y_step * y_step_len;
            z = 0 + z_step * z_step_len;

            raw = raw_matrix_add(x_step+1, y_step+1, z_step+1);
            if raw > 0
                size = raw * 10;
                color = raw;
                edgecolor = [0 0.4470 0.7410];

                scatter3(x, y, z, size, color, 'o', 'filled', MarkerFaceColor='flat', MarkerEdgeColor=edgecolor, MarkerFaceAlpha=.8)
                hold on
            end

        end
    end
end

set(gcf,'unit','normalized','position', [0, 0, 0.45, 0.5])

c = colorbar('Ticks', [0, 1.5, 3, 4.5, 6, 7.5, 9], 'FontSize', 20);
c.Label.String = 'r_{AW} (N)';
c.Label.FontSize = 24;
c.Label.FontName = "Times New Roman";
c.Position = [0.81, 0.1, 0.025, 0.8];

set(gca,'FontSize',20)
set(gca,'unit','normalized','position', [0.1, 0.1, 0.65, 0.9])
xlabel('x (m)','FontName','Times New Roman','FontSize',24,'Rotation',-25)  
ylabel('y (m)','FontName','Times New Roman','FontSize',24,'Rotation',32)
zlabel('z (m)','FontName','Times New Roman','FontSize',24,'Rotation',90)

axis equal;

axis([-0.4, 0.4, -0.4, 0.4, 0, 0.8])
view(40, 30)

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
figure(3)
colormap(slanCM('parula', 'truncation', [0, round(max(max(max(raw_matrix))) / raw_max * 256)]));

for x_step=0:(x_num-1)
    for y_step=0:(x_num-1)
        for z_step=0:(x_num-1)

            x = A2(1) + 0.01 + x_step * x_step_len;
            y = A3(2) + 0.01 + y_step * y_step_len;
            z = 0 + z_step * z_step_len;

            raw = max([raw_matrix(x_step+1, y_step+1, z_step+1), raw_matrix_add(x_step+1, y_step+1, z_step+1)]);
            if raw > 0
                size = raw * 10;
                color = raw;
                edgecolor = [0 0.4470 0.7410];

                scatter3(x, y, z, size, color, 'o', 'filled', MarkerFaceColor='flat', MarkerEdgeColor=edgecolor, MarkerFaceAlpha=.8)
                hold on
            end

        end
    end
end

set(gcf,'unit','normalized','position', [0, 0, 0.45, 0.5])

c = colorbar('Ticks', [0, 1.5, 3, 4.5, 6, 7.5, 9], 'FontSize', 20);
c.Label.String = 'r_{AW} (N)';
c.Label.FontSize = 24;
c.Label.FontName = "Times New Roman";
c.Position = [0.81, 0.1, 0.025, 0.8];

set(gca,'FontSize',20)
set(gca,'unit','normalized','position', [0.1, 0.1, 0.65, 0.9])
xlabel('x (m)','FontName','Times New Roman','FontSize',24,'Rotation',-25)  
ylabel('y (m)','FontName','Times New Roman','FontSize',24,'Rotation',32)
zlabel('z (m)','FontName','Times New Roman','FontSize',24,'Rotation',90)

axis equal;

axis([-0.4, 0.4, -0.4, 0.4, 0, 0.8])
view(40, 30)
