clear all;
clc;
%���ݽ���Ԥ����
%���ս����ݱ��浽mat�ļ���

%%    
%��Kinect������
% bag=rosbag('D:\PSY\PostGraduate\gaitData\04\03\.bag');     %��Ҫ�޸�%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% filedir='D:\PSY\PostGraduate\gaitData\test\';
root_dir = '/media/psy/HIKVISION/gait_back';
person_list = dir(root_dir);
for p = 1:length(person_list)
    if(isequal(person_list(p).name, '.')||isequal(person_list(p).name, '..')||~person_list(p).isdir) %�������Ŀ¼������
        continue;
    end
    person_path = fullfile(root_dir,person_list(p).name);
    person_path
    sample_list = dir(person_path);
for s = 1:length(sample_list)
    if(isequal(sample_list(s).name, '.')||isequal(sample_list(s).name, '..')) %�������Ŀ¼������
        continue;
    end
    bag_path = fullfile(person_path,sample_list(s).name);
    bag_path
    bag=rosbag(bag_path);
    filedir=fullfile(root_dir,'mat',person_list(p).name,num2str(s));
    filedir
    mkdir(filedir);

message_select=select(bag,'Topic','/points2','MessageType','sensor_msgs/PointCloud2');
ptcloudData_kinect=readMessages(message_select);
ptcloudData_kinect=[ptcloudData_kinect{:,1}];
[m_kinect,n_kinect]=size(ptcloudData_kinect); % nΪ����֡������

%���ؼ��������
message_body=select(bag,'Topic','/body_tracking_data','MessageType','visualization_msgs/MarkerArray'); %ѡ����Ӧ��PointCloud2 message���п��ӻ�
MarkerArrayData=readMessages(message_body);
MarkerArrayData=[MarkerArrayData{:,1}];
fprintf('MarkerArrayData Length:%d\n',length(MarkerArrayData));

%��ȡimg����
message_select=select(bag,'Topic','/rgb/image_raw','MessageType','sensor_msgs/Image'); %ѡ����Ӧ��PointCloud2 message���п��ӻ�
imgData_kinect=readMessages(message_select);
imgData_kinect=[imgData_kinect{:,1}];
%%
%��ti������
message_select=select(bag,'Topic','/ti_mmwave/radar_scan_pcl_0','MessageType','sensor_msgs/PointCloud2'); %
ptcloudData=readMessages(message_select); %��ȡ�ܶ�֡�ĵ�������
ptcloudData=[ptcloudData{:,1}]; %��������Ϊcell��ʽ��ģ�תΪ������ʽ
[m,n]=size(ptcloudData); % nΪ����֡������

x_ti=[];y_ti=[];z_ti=[];
intensity_3=[];velocity_3=[];oneall=[];
%%
CheckPoint = 0;
for t=1:length(MarkerArrayData)
    fprintf('%d\n',length(MarkerArrayData(t).Markers));
    if length(MarkerArrayData(t).Markers)==32
        CheckPoint = t;
        break;
    end
end
fprintf('init checkpoint:%d\n',CheckPoint)
for i=2:n %20:42     %��Ҫ�޸�%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    i
    %%
    %������Ҫ�޸� ti֡��Ӧ��Kinect֡
%     if 1.5*n-n_kinect>15
%         ratio=1.15;
%     else
%         ratio=1.5;
%     end
    ratio=n_kinect/n; 
    i_kinect=fix((i)*ratio)-1;       %��Ҫ�޸�%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        %%
    %Kinect ��ȡKinect����
    %ptcloud_kinect=ptcloudData_kinect(i);%round
    ptcloud_kinect=ptcloudData_kinect(i_kinect);%round %*3
    xyz_kinect = readXYZ(ptcloud_kinect);
    %rgb_kinect = readRGB(ptcloud_kinect);
    %����Kinect��ÿ������4��field��x,y,z,rgb
    xyz_kinect=xyz_kinect*rotx(6);
    
    %x_kinect=xyz_kinect(:,1);
    %y_kinect=xyz_kinect(:,3);
    %z_kinect=xyz_kinect(:,2).*(-1);
    %xyz_kinect=[x_kinect,y_kinect,z_kinect];
    xyz_kinect=[xyz_kinect(:,1),xyz_kinect(:,3),xyz_kinect(:,2).*(-1)];
    %xyz_kinect= datasample(xyz_kinect, 50000); 
    %x_kinect=xyz_kinect(:,1);
    %y_kinect=xyz_kinect(:,2);
    %z_kinect=xyz_kinect(:,3);
    
    %xyz2=datasample(xyz2,1000000);
    ptIndexOnBody_kinect=find((xyz_kinect(:,1)>-1.2 & xyz_kinect(:,1)<0) & (xyz_kinect(:,2)>0.9 & xyz_kinect(:,2)<6.5) ...
                                                         & (xyz_kinect(:,3)>-1 & xyz_kinect(:,3)<1));
    xyz_kinect2=xyz_kinect(ptIndexOnBody_kinect,:);%�����ϵ�Kinect����
    % ��������ڷ�Χ֮���ֱ������
    if(mean(xyz_kinect2(:,2))>4.5 || mean(xyz_kinect2(:,2))<1.5)
        continue;
    end
   %%
    %ti ��ȡti����
    for j=0:0
        ptcloud=ptcloudData(i+j);
        % ptcloud = message_select
        xyz = readXYZ(ptcloud);
        intensity = readField(ptcloud,'intensity'); %���ݲ���'intensity'��ȡ��Field������
        velocity=readField(ptcloud,'velocity');
        
        %����ti���ײ���ÿ������4��field��x,y,z,intensity
        % rgb = readRGB(ptcloud);
       
        %�������ĵ���ROS������ϵ�� x-ǰ��y-��z-��
        %ת������������ϵ�ķ��� x-�ң�y-ǰ��z-��
        %x1=(-1).*xyz(:,2);
        %y1=xyz(:,1);
        %z1=xyz(:,3);
        %xyz_ti1=[x1,y1,z1];
        xyz_ti1=[(-1).*xyz(:,2),xyz(:,1),xyz(:,3)];        
        
        %ѡ����Χ�ڵĵ�
        ptIndexOnBody_ti1=find((xyz_ti1(:,1)>-1.2 & xyz_ti1(:,1)<0) & (xyz_ti1(:,2)>1 & xyz_ti1(:,2)<5) ...
                                                         & (xyz_ti1(:,3)>-0.975 & xyz_ti1(:,3)<0.8));                                            
        xyz_ti2=xyz_ti1(ptIndexOnBody_ti1,:); %�����ϵ�ti����
        intensity=intensity(ptIndexOnBody_ti1,:);
        velocity=velocity(ptIndexOnBody_ti1,:);
        %�Ⱦ��࣬��ƴ��
       %%
        %��DBSCAN���о���
        A=xyz_ti2;
        Eps=0.35;%0.25                   %�������%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        MinPts=2;
        Clust = DBSCAN(A,Eps,MinPts);
        table=tabulate(Clust);
        [a,b]=size(table);
        if(~a)
            fprintf("�뽫֡������һ�㣬̫֡��ǰ��û�м�⵽ti��\n"); 
        end
        [F,I]=max(table(:,2));
        I=find(table(:,2)==F,1);
        Clust_id=table(I,1);
        index=find(Clust==Clust_id);

        %xyz_ti3=xyz_ti2;
        xyz_ti3=xyz_ti2(index,:);    
        intensity=intensity(index,:);
        velocity=velocity(index,:);
        
        x_ti=[x_ti;xyz_ti3(:,1)];
        y_ti=[y_ti;xyz_ti3(:,2)];
        z_ti=[z_ti;xyz_ti3(:,3)];
        intensity_3=[intensity_3;intensity];
        velocity_3=[velocity_3;velocity];
        
        [point_num,column]=size(xyz_ti3);
        one=ones(point_num,1);
        oneall=[oneall;one];
    end
    
    %%
    %tiת��Kinect����ϵ
%     xyzk=[x_ti,y_ti,z_ti,oneall];
%     %matrix=[0.8765,0.0702,0.1118;-0.0324,1.0009,0.0118;-0.1161,0.0147,0.9200;0.0705,-0.0994,-0.0199];%2.5cm
%     matrix=[0.8782,0.0281,0.0210;-0.0609,0.9788,-0.0202;0.0182,0.0647,0.8499;0.0043,-0.0025,-0.0632];
%     xyzk=xyzk*matrix;
    
%     xyz_ti4=[xyzk(:,1),xyzk(:,2),xyzk(:,3)];   
    xyz_ti4=[x_ti,y_ti,z_ti];
    %%
    %��ͼ    
     figure(1)
     %subplot(2,3,1)
%     scatter3(xyz_ti4(:,1),xyz_ti4(:,2),xyz_ti4(:,3),20,intensity,'filled'); %20��������Ĵ�С
    scatter3(xyz_ti4(:,1),xyz_ti4(:,2),xyz_ti4(:,3),20,'filled','r'); hold on;%20��������Ĵ�С
    scatter3(xyz_kinect2(:,1),xyz_kinect2(:,2),xyz_kinect2(:,3),4,'filled','k'); hold on;
    xlabel('X'),ylabel('Y'),zlabel('Z');%axis equal;
    set(gca,'XLim',[-1.2 0],'YLim',[1 5],'ZLim',[-1.2 1.2])
    %view(-26,0.3);
    view([-1,0,0]);
    title('����ͼ')
%     colorbar
    drawnow
    hold on;
% 
    figure(2)
    scatter3(xyz_ti4(:,1),xyz_ti4(:,2),xyz_ti4(:,3),20,intensity,'filled');  hold on;%20��������Ĵ�С
    scatter3(xyz_ti4(:,1),xyz_ti4(:,2),xyz_ti4(:,3),20,'filled','r'); hold on;%20��������Ĵ�С
    scatter3(xyz_kinect2(:,1),xyz_kinect2(:,2),xyz_kinect2(:,3),4,'filled','k'); hold on;
%     xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)'); %axis equal;
    set(gca,'XLim',[-1.2 0],'YLim',[1 5],'ZLim',[-1.2 1.2])
%     set(gca,'XLim',[-1 1],'YLim',[0.9 5],'ZLim',[-1.2 1.2])
    %view(-26,0.3);
%     colorbar
title('������Ti��')
    view([-24, -8, 10]);
    
    figure(3) 
    
%     scatter3(xyz_ti4(:,1),xyz_ti4(:,2),xyz_ti4(:,3),20,intensity,'filled'); hold on; %20��������Ĵ�С
    scatter3(xyz_ti1(:,1),xyz_ti1(:,2),xyz_ti1(:,3),20,'filled','r'); hold on;%20��������Ĵ�С
    scatter3(xyz_kinect2(:,1),xyz_kinect2(:,2),xyz_kinect2(:,3),4,'filled','k'); hold on;
    xlabel('x (m)'); ylabel('y (m)'); zlabel('z (m)'); %axis equal;
    set(gca,'XLim',[-1.2 0],'YLim',[1 5],'ZLim',[-1.2 1.2])
%     colorbar
title('ԭʼTi��')
    view([-24, -8, 10]);
    
    %%
    position=[]; % position����Źؼ���
    
    
    try
        MarkerArray=MarkerArrayData(i_kinect);
        MarkerNum=length(MarkerArray.Markers);
        fprintf('MarkerNum:%d\n',MarkerNum);
        if MarkerNum == 32
            CheckPoint = i_kinect;
            fprintf('set checkpoint %d',CheckPoint);
        end
    catch MException
        try
            fprintf('in frame %d: miss MarkerArray, use checkpoint %d\n',i_kinect,CheckPoint);
        catch e
            
        end
    end
    MarkerArray = MarkerArrayData(CheckPoint);
    
    
    for k=1:32
        position=[position;MarkerArray.Markers(k).Pose.Position];
    end
    position=[position(:).X,position(:).Y,position(:).Z];
    %size(position);
    position=reshape(position,32,3);
    position=position*rotx(6);
    %x_position=position(:,1);
    %y_position=position(:,3);
    %z_position=position(:,2).*(-1);
    %position=[x_position,y_position,z_position];
    position=[position(:,1),position(:,3),position(:,2).*(-1)];
    
    % key point check
    if position(7,1)<position(14,1)
        temp = position(5:11,:);
        position(5:11,:) = position(12:18,:);
        position(12:18,:) = temp;

        temp = position(19:22,:);
        position(19:22,:) = position(23:26,:);
        position(23:26,:) = temp;

        temp = position(29:30,:);
        position(29:30,:) = position(31:32,:);
        position(31:32,:) = temp;
    end
    
    
%      figure(3); 
%      scatter3(position(:,1),position(:,2),position(:,3),10,'filled','g');hold on; %20��������Ĵ�С
%      plot3(position(1:4,1),position(1:4,2),position(1:4,3));
%      plot3(position(3:2:5,1),position(3:2:5,2),position(3:2:5,3));
%      plot3(position(5:10,1),position(5:10,2),position(5:10,3));
%      plot3(position(8:3:11,1),position(8:3:11,2),position(8:3:11,3));
%      plot3(position(3:9:12,1),position(3:9:12,2),position(3:9:12,3));
%      plot3(position(12:17,1),position(12:17,2),position(12:17,3));
%      plot3(position(15:3:18,1),position(15:3:18,2),position(15:3:18,3));
%      plot3(position(1:18:19,1),position(1:18:19,2),position(1:18:19,3));
%      plot3(position(19:22,1),position(19:22,2),position(19:22,3));
%      plot3(position(1:22:23,1),position(1:22:23,2),position(1:22:23,3));
%      plot3(position(23:26,1),position(23:26,2),position(23:26,3));
%      plot3(position(4:23:27,1),position(4:23:27,2),position(4:23:27,3));
%      plot3(position(27:30,1),position(27:30,2),position(27:30,3));
%      plot3(position(28:3:31,1),position(28:3:31,2),position(28:3:31,3));
%      plot3(position(31:32,1),position(31:32,2),position(31:32,3));
%      plot3(position(27:28,1),position(27:28,2),position(27:28,3));
    
    %%  ���ɷָ�
    head_key_point = [position(27,:)];
    
    forebreast_key_point = [position(1,:);position(2,:);position(3,:);position(4,:)]; %ǰ��
    forebreast_minus_1 = position(1,:) - position(2,:);
    forebreast_module_1 = norm(forebreast_minus_1);
    forebreast_minus_2 = position(2,:) - position(3,:);
    forebreast_module_2 = norm(forebreast_minus_2);
    forebreast_minus_3 = position(3,:) - position(4,:);
    forebreast_module_3 = norm(forebreast_minus_3);
    
    left_arm_key_point = [position(6,:);position(7,:);position(8,:)];  % ���ֱ�
    left_arm_minus_1 = position(6,:) - position(7,:);
    left_arm_module_1 = norm(left_arm_minus_1);
    left_arm_minus_2 = position(7,:) - position(8,:);
    left_arm_module_2 = norm(left_arm_minus_2);
    
    right_arm_key_point = [position(13,:);position(14,:);position(15,:)]; % ���ֱ�
    right_arm_minus_1 = position(13,:) - position(14,:);
    right_arm_module_1 = norm(right_arm_minus_1);
    right_arm_minus_2 = position(14,:) - position(15,:);
    right_arm_module_2 = norm(right_arm_minus_2);
    
    left_leg_key_point = [position(19,:);position(20,:);position(21,:)];  % ����
    left_leg_minus_1 = position(19,:) - position(20,:);
    left_leg_module_1 = norm(left_leg_minus_1);
    left_leg_minus_2 = position(20,:) - position(21,:);
    left_leg_module_2 = norm(left_leg_minus_2);
    
    right_leg_key_point = [position(23,:);position(24,:);position(25,:)];  % ����
    right_leg_minus_1 = position(23,:) - position(24,:);
    right_leg_module_1 = norm(right_leg_minus_1);
    right_leg_minus_2 = position(24,:) - position(25,:);
    right_leg_module_2 = norm(right_leg_minus_2);
    
    head_point = [];
    forebreast_point = []; %ǰ��
    
    left_arm_point = [];
    left_upper_arm_point = []; % ���ϱ�
    left_lower_arm_point = [];% ���±�
    
    right_arm_point = [];
    right_upper_arm_point = []; % ���ϱ�
    right_lower_arm_point = []; % ���±�
    
    left_leg_point = [];
    left_thigh_point = []; % �����
    left_shank_point = []; % ��С��
    
    right_leg_point = [];
    right_thigh_point = []; % �Ҵ���
    right_shank_point = []; % ��С��
    
    Low_body_distance = [];
    High_body_distance = [];
    
    %%  Head
    head_point_index = find(xyz_kinect2(:,3)>position(27,3));  %����ѡ������
    head_point = xyz_kinect2(head_point_index,:);
%      figure(3)
%      scatter3(head_point(:,1),head_point(:,2),head_point(:,3),3,'filled','m');
    
    xyz_kinect2_unzone = setdiff(xyz_kinect2, head_point,'rows');   % xyz_kinect2��ɾ��ͷ����
%      figure(3)
%      scatter3(xyz_kinect2(:,1),xyz_kinect2(:,2),xyz_kinect2(:,3),3,'filled','b');

    %% Low part of body --- Left leg / Right Leg / part of arms
    low_body_index = find(xyz_kinect2_unzone(:,3)<position(19,3));  %����ѡ������
    Low_body = xyz_kinect2_unzone(low_body_index,:);
    Left_leg_distance_1 = sqrt(sum(cross(repmat(left_leg_minus_1,size(low_body_index,1),1), Low_body-repmat(position(20,:),size(low_body_index,1),1) ,2).^2,2))/left_leg_module_1;
    Left_leg_distance_2 = sqrt(sum(cross(repmat(left_leg_minus_2,size(low_body_index,1),1), Low_body-repmat(position(21,:),size(low_body_index,1),1) ,2).^2,2))/left_leg_module_2;
    Low_body_distance(:,1) = min(Left_leg_distance_1,Left_leg_distance_2);  
    
    Right_leg_distance_1 = sqrt(sum(cross(repmat(right_leg_minus_1,size(low_body_index,1),1), Low_body-repmat(position(24,:),size(low_body_index,1),1) ,2).^2,2))/right_leg_module_1;
    Right_leg_distance_2 = sqrt(sum(cross(repmat(right_leg_minus_2,size(low_body_index,1),1), Low_body-repmat(position(25,:),size(low_body_index,1),1) ,2).^2,2))/right_leg_module_2;
    Low_body_distance(:,2) = min(Right_leg_distance_1,Right_leg_distance_2); 
    
    Left_arm_distance_1 = sqrt(sum(cross(repmat(left_arm_minus_1,size(low_body_index,1),1), Low_body-repmat(position(7,:),size(low_body_index,1),1) ,2).^2,2))/left_arm_module_1;
    Left_arm_distance_2 = sqrt(sum(cross(repmat(left_arm_minus_2,size(low_body_index,1),1), Low_body-repmat(position(8,:),size(low_body_index,1),1) ,2).^2,2))/left_arm_module_2;
    Low_body_distance(:,3) = min(Left_arm_distance_1,Left_arm_distance_2); 
    
    Right_arm_distance_1 = sqrt(sum(cross(repmat(right_arm_minus_1,size(low_body_index,1),1), Low_body-repmat(position(14,:),size(low_body_index,1),1) ,2).^2,2))/right_arm_module_1;
    Right_arm_distance_2 = sqrt(sum(cross(repmat(right_arm_minus_2,size(low_body_index,1),1), Low_body-repmat(position(15,:),size(low_body_index,1),1) ,2).^2,2))/right_arm_module_2;
    Low_body_distance(:,4) = min(Right_arm_distance_1,Right_arm_distance_2);
    
    % ���ֱ�(����)
    left_arm_index = find(min(Low_body_distance,[],2)==Low_body_distance(:,3) & Low_body(:,1)>position(11,1)-0.04 & Low_body(:,3)>position(10,3)-0.04);
    left_arm_point = [left_arm_point;Low_body(left_arm_index,:)];
    
    % ���ֱ�(����)
    right_arm_index = find(min(Low_body_distance,[],2)==Low_body_distance(:,4) & Low_body(:,1)<position(18,1)+0.04 & Low_body(:,3)>position(17,3)-0.04);
    right_arm_point = [right_arm_point;Low_body(right_arm_index,:)]; 
    
    % ����
    left_leg_index = setdiff([find(min(Low_body_distance,[],2)==Low_body_distance(:,1));find(min(Low_body_distance,[],2)==Low_body_distance(:,3))],left_arm_index);
    left_leg_point = [left_leg_point;Low_body(left_leg_index,:)];
    left_thigh_point = left_leg_point(left_leg_point(:,3)>position(20,3),:); % �����
    left_shank_point = left_leg_point(left_leg_point(:,3)<=position(20,3),:); % ��С��
%      figure(3)
%      scatter3(left_thigh_point(:,1),left_thigh_point(:,2),left_thigh_point(:,3),3,'filled','r');
%      scatter3(left_shank_point(:,1),left_shank_point(:,2),left_shank_point(:,3),3,'filled','b');
    
    % ����
    right_leg_index = setdiff([find(min(Low_body_distance,[],2)==Low_body_distance(:,2));find(min(Low_body_distance,[],2)==Low_body_distance(:,4))],right_arm_index);
    right_leg_point = [right_leg_point;Low_body(right_leg_index,:)];
    right_thigh_point = right_leg_point(right_leg_point(:,3)>position(24,3),:); % �Ҵ���
    right_shank_point = right_leg_point(right_leg_point(:,3)<=position(24,3),:); % ��С��
%      figure(3)
%      scatter3(right_thigh_point(:,1),right_thigh_point(:,2),right_thigh_point(:,3),3,'filled','b');
%      scatter3(right_shank_point(:,1),right_shank_point(:,2),right_shank_point(:,3),3,'filled','c');

    xyz_kinect2_unzone = setdiff(xyz_kinect2_unzone, Low_body,'rows'); % xyz_kinect2��ɾ��������
    
    %% High part of body --- Left arm / Right arm / Forebreast
    High_body = xyz_kinect2_unzone;
    
    Forebreast_distance_1 = sqrt(sum(cross(repmat(forebreast_minus_1,size(High_body,1),1), High_body-repmat(position(2,:),size(High_body,1),1) ,2).^2,2))/forebreast_module_1;
    Forebreast_distance_2 = sqrt(sum(cross(repmat(forebreast_minus_2,size(High_body,1),1), High_body-repmat(position(3,:),size(High_body,1),1) ,2).^2,2))/forebreast_module_2;
    Forebreast_distance_3 = sqrt(sum(cross(repmat(forebreast_minus_3,size(High_body,1),1), High_body-repmat(position(4,:),size(High_body,1),1) ,2).^2,2))/forebreast_module_3;
    High_body_distance(:,1) = min([Forebreast_distance_1,Forebreast_distance_2,Forebreast_distance_3],[],2);
    
    Left_arm_distance_1 = sqrt(sum(cross(repmat(left_arm_minus_1,size(High_body,1),1), High_body-repmat(position(7,:),size(High_body,1),1) ,2).^2,2))/left_arm_module_1;
    Left_arm_distance_2 = sqrt(sum(cross(repmat(left_arm_minus_2,size(High_body,1),1), High_body-repmat(position(8,:),size(High_body,1),1) ,2).^2,2))/left_arm_module_2;
    High_body_distance(:,2) = min(Left_arm_distance_1,Left_arm_distance_2);
    
    Right_arm_distance_1 = sqrt(sum(cross(repmat(right_arm_minus_1,size(High_body,1),1), High_body-repmat(position(14,:),size(High_body,1),1) ,2).^2,2))/right_arm_module_1;
    Right_arm_distance_2 = sqrt(sum(cross(repmat(right_arm_minus_2,size(High_body,1),1), High_body-repmat(position(15,:),size(High_body,1),1) ,2).^2,2))/right_arm_module_2;
    High_body_distance(:,3) = min(Right_arm_distance_1,Right_arm_distance_2);
    
    % ���ֱ�(����)
    left_arm_index = find(min(High_body_distance,[],2)==High_body_distance(:,2) & High_body(:,1)>position(6,1));
    left_arm_point = [left_arm_point;High_body(left_arm_index,:)];
    left_upper_arm_point = left_arm_point(left_arm_point(:,3)>position(7,3),:);  % ���ϱ�
    left_lower_arm_point = left_arm_point(left_arm_point(:,3)<=position(7,3),:); % ���±�
%      figure(3)
%      scatter3(left_upper_arm_point(:,1),left_upper_arm_point(:,2),left_upper_arm_point(:,3),3,'filled','g');
%      scatter3(left_lower_arm_point(:,1),left_lower_arm_point(:,2),left_lower_arm_point(:,3),3,'filled','y');
    
    % ���ֱ�(����)
    right_arm_index = find(min(High_body_distance,[],2)==High_body_distance(:,3) & High_body(:,1)<position(13,1));
    right_arm_point = [right_arm_point;High_body(right_arm_index,:)];
    right_upper_arm_point = right_arm_point(right_arm_point(:,3)>position(14,3),:);  % ���ϱ�
    right_lower_arm_point = right_arm_point(right_arm_point(:,3)<=position(14,3),:); % ���±�
%      figure(3)
%      scatter3(right_upper_arm_point(:,1),right_upper_arm_point(:,2),right_upper_arm_point(:,3),3,'filled','y'); 
%      scatter3(right_lower_arm_point(:,1),right_lower_arm_point(:,2),right_lower_arm_point(:,3),3,'filled','g');
    
    forebreast_point = setdiff(setdiff(High_body, left_arm_point,'rows'),right_arm_point,'rows');
%      figure(3)
%      scatter3(forebreast_point(:,1),forebreast_point(:,2),forebreast_point(:,3),3,'filled','k'); 
    
    
    % ͷ��1��| ǰ�أ�2��| ���ϱۣ�3��| ���±ۣ�4��| ���ϱۣ�5��|  ���±ۣ�6��|  ����ȣ�7��|  ��С�ȣ�8��|  �Ҵ��ȣ�9��|  ��С�ȣ�10��
    head_point(:,4) = ones(size(head_point,1),1); 
    forebreast_point(:,4) = ones(size(forebreast_point,1),1)*2; %ǰ��
    
    left_upper_arm_point(:,4) = ones(size(left_upper_arm_point,1),1)*3; % ���ϱ�
    left_lower_arm_point(:,4) = ones(size(left_lower_arm_point,1),1)*4;% ���±�
    
    right_upper_arm_point(:,4) = ones(size(right_upper_arm_point,1),1)*5; % ���ϱ�
    right_lower_arm_point(:,4) = ones(size(right_lower_arm_point,1),1)*6; % ���±�
    
    left_thigh_point(:,4) = ones(size(left_thigh_point,1),1)*7; % �����
    left_shank_point(:,4) = ones(size(left_shank_point,1),1)*8; % ��С��
    
    right_thigh_point(:,4) = ones(size(right_thigh_point,1),1)*9; % �Ҵ���
    right_shank_point(:,4) = ones(size(right_shank_point,1),1)*10; % ��С��
    
    xyz_kinect2 = [head_point;forebreast_point;left_upper_arm_point;left_lower_arm_point;right_upper_arm_point;right_lower_arm_point;
                   left_thigh_point;left_shank_point;right_thigh_point;right_shank_point];

    %%
    pause(0.5);
    figure(1)
    hold off;
    figure(2)
    hold off;
    figure(3)
    hold off;
    
    %%
    %д��mat�ļ���λ���Լ����� ��Ҫ�޸�
    path=filedir;  %��Ҫ�޸�%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    filename=['pc_ti_kinect_key_',num2str(i,'%02d'),'.mat'];  
    filename2=['pc_ti_kinect_key_',num2str(i,'%02d'),'_rgb.mat'];  

    %Ҫ����ľ���
    pc_xyziv_ti=[xyz_ti4,intensity_3,velocity_3];
    pc_xyzb_kinect=xyz_kinect2;
    pc_xyz_key=position;
    %    imgת���ɾ��󣬱����޸ĸ�ʽ���img����
    imageFormatted=single(readImage(imgData_kinect(i_kinect)));
    save([path,filename],'pc_xyziv_ti','pc_xyzb_kinect','pc_xyz_key'); 
    save([path,filename2],'imageFormatted'); 

%    ����ԭʼimg����
%    img_kinect=imgData_kinect(i_kinect).Data; 
% %    save([path,filename],'pc_xyziv_ti','pc_xyzb_kinect','pc_xyz_key','img_kinect');   
%     save([path,filename],'pc_xyziv_ti','pc_xyzb_kinect','pc_xyz_key');   
    %%
    %����
    x_ti=[];y_ti=[];z_ti=[];intensity_3=[];velocity_3=[];oneall=[];
    %clf;
end
end
end
% zuo=[20 30 41];
% you=[26 35];
% key_frame={zuo,you};
% save([path,'0'],'key_frame');


%֮���ȡ�ؼ�֡
% zuo=[key_frame{:,1}]
% you=[key_frame{:,2}]


    