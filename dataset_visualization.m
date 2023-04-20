clear;
% dir = "D:\东南资料\周报\学长项目\mat_data_3D_镜面反射角度10度\mat_data_3D_镜面反射角度10度\01zhanglinyan\06\";
% dir = "F:\SouthEast\Reid\Reid_data\Rawdata\siyuan\processed_data_00_new_2\23\20\";
dir = "D:\天顺师兄\0419数据预处理代码\gait\2\";
bagselect=3;
prefix = "pc_ti_kinect_key_";
indices =18:36;
dongjiang =0;
if dongjiang==1
    suffix = ".mat_0.mat";
    suffix_ply = ".mat.ply";
else
    suffix = ".mat";
end 


saveAsVideo = 1;
figFrameCount = 0;
videofile = "test.avi";

xyzivb_ti_list = {};
xyz_key_list = {};
xyzb_kinect_list = {};


% read the data and find the activity area
x_max = -Inf; y_max = -Inf; z_max = -Inf;
x_min = Inf; y_min = Inf; z_min = Inf;

for index = indices
    data = load(dir + bagselect + prefix + num2str(index,'%02d') + suffix);
    data
    xyzivb_ti = data.pc_xyziv_ti;  %radar point cloud
    xyz_key = data.pc_xyz_key;     %kinect key points
    xyzb_kinect = data.pc_xyzb_kinect;     %kinect points
    
    x_min = min([x_min, min(xyzivb_ti(:,1)), min(xyz_key(:,1))]);
    y_min = min([y_min, min(xyzivb_ti(:,2)), min(xyz_key(:,2))]);
    z_min = min([z_min, min(xyzivb_ti(:,3)), min(xyz_key(:,3))]);
    
    x_max = max([x_max, max(xyzivb_ti(:,1)), max(xyz_key(:,1))]);
    y_max = max([y_max, max(xyzivb_ti(:,2)), max(xyz_key(:,2))]);
    z_max = max([z_max, max(xyzivb_ti(:,3)), max(xyz_key(:,3))]);
    
    xyzivb_ti_list = [xyzivb_ti_list, xyzivb_ti];
    xyz_key_list = [xyz_key_list, xyz_key];
    xyzb_kinect_list= [xyzb_kinect_list, xyzb_kinect];
end

% visualize
fig = figure();
set(gcf,'Position',[100 100 2500 400])
for index = 1:length(indices)
    subplot(1,2,1);
    xyzivb_ti = xyzivb_ti_list{index}; %radar point cloud
    xyz_key = xyz_key_list{index};     %kinect key points
    xyzb_kinect = xyzb_kinect_list{index};     %kinect key points
    
    X_r = xyzivb_ti(:,1);
    Y_r = xyzivb_ti(:,2);
    Z_r = xyzivb_ti(:,3);
    
    plot3(X_r, Y_r, Z_r, "ro", 'MarkerFaceColor','r')
    xlabel('x(t)')
    ylabel('y(t)')
    zlabel('z(t)')
    xlim([x_min x_max])
    ylim([y_min y_max])
    zlim([z_min z_max])
    hold on;
    grid on;
    scatter3(xyzb_kinect(:,1),xyzb_kinect(:,2), xyzb_kinect(:,3),'.','MarkerEdgeColor',[0.8 0.8 0.8],'MarkerEdgeAlpha',0.6);
    hold on;
    grid on;
    drawKinectPose3D(xyz_key, fig)
    hold off;

   
    subplot(1,2,2);
    i = indices(1) + index - 1; 
    if dongjiang==1
        ptCloud = pcread(dir  + prefix + i + suffix_ply);
        kinect_raw_mesh = ptCloud.Location;
        scatter(kinect_raw_mesh(:,2), kinect_raw_mesh(:,3),'.','MarkerEdgeColor',[0.8 0.8 0.8],'MarkerEdgeAlpha',0.6);
    else
        scatter(xyzb_kinect(:,2), xyzb_kinect(:,3),'.','MarkerEdgeColor',[0.8 0.8 0.8],'MarkerEdgeAlpha',0.6);
    end 
    
    hold on;

    plot(Y_r, Z_r, "ro", 'MarkerFaceColor','r')
    xlabel('y(t)')
    ylabel('z(t)')
    xlim([y_min y_max])
    ylim([z_min z_max])
    hold on;
    drawKinectPoseYZ(xyz_key, fig);
    grid on;
    legend("Kinect Mesh",  "Radar", "Joints",  'Location','southwest')
    
    % record
    figFrameCount = figFrameCount + 1;
    if saveAsVideo
       F(figFrameCount) = getframe(gcf); 
    end
    pause(0.1)
end

if (saveAsVideo)
    writeObj =  VideoWriter(videofile);
    writeObj.FrameRate = 5;
    open(writeObj);
    for i = 1:length(F)
        frame = F(i);
        writeVideo(writeObj, frame);
    end
    close(writeObj)
end

function [] = drawKinectPose3D(xyz_key, fig)
    figure(fig)
    parent_joints = 1+ [0,0,1,2,2,4,5,6,7,8,7,2,11,12,13,14,15,14,0,18,19,20,0,22,23,24,3,26,26,26,26,26];
    for idx = 1:size(xyz_key,1)
        joint = [xyz_key(idx,1),xyz_key(idx,2),xyz_key(idx,3)];
        parent =  [xyz_key(parent_joints(idx),1),xyz_key(parent_joints(idx),2),xyz_key(parent_joints(idx),3)];
        plot3([joint(1),parent(1)], ...
            [joint(2),parent(2)], ...
            [joint(3),parent(3)], "k-",  "Linewidth", 2);
        hold on;
    end
    hold off
end

function [] = drawKinectPoseYZ(xyz_key, fig)
    figure(fig)
    parent_joints = 1+ [0,0,1,2,2,4,5,6,7,8,7,2,11,12,13,14,15,14,0,18,19,20,0,22,23,24,3,26,26,26,26,26];
    for idx = 1:size(xyz_key,1)
        joint = [xyz_key(idx,1),xyz_key(idx,2),xyz_key(idx,3)];
        parent =  [xyz_key(parent_joints(idx),1),xyz_key(parent_joints(idx),2),xyz_key(parent_joints(idx),3)];
        plot([joint(2),parent(2)], [joint(3),parent(3)],"k-", "Linewidth", 2);
        hold on;
    end
    hold off
end

