clear all;
clc;

root_dir = 'D:/data';
sample_list = dir(root_dir);
for s = 1:length(sample_list)
    if(isequal(sample_list(s).name, '.')||isequal(sample_list(s).name, '..')) %如果不是目录则跳过
        continue;
    end
    bag_path = fullfile(root_dir,sample_list(s).name);
    bag_path
    bag=rosbag(bag_path);

    message_select=select(bag,'Topic','/points2','MessageType','sensor_msgs/PointCloud2');
    ptcloudData_kinect=readMessages(message_select);
    ptcloudData_kinect=[ptcloudData_kinect{:,1}];
    [m_kinect,n_kinect]=size(ptcloudData_kinect); % n为点云帧的数量
        

    %读ti的数据
    message_select=select(bag,'Topic','/ti_mmwave/radar_scan_pcl_0','MessageType','sensor_msgs/PointCloud2'); %
    ptcloudData=readMessages(message_select); %读取很多帧的点云数据
    ptcloudData=[ptcloudData{:,1}]; %点云数据为cell格式存的，转为数组形式
    [m_ti,n_ti]=size(ptcloudData); % n为点云帧的数量


        i_ti=fix(n_ti/2) ;

        ratio=n_kinect/n_ti;%Kinect和点云帧比率
        i_kinect=fix((i)*ratio)-1;   %向0取整   
        ptcloud_kinect=ptcloudData_kinect(i_kinect);%round %*3
        xyz_kinect = readXYZ(ptcloud_kinect);
        xyz_kinect=xyz_kinect*rotx(6);
        xyz_kinect=[xyz_kinect(:,1),xyz_kinect(:,3),xyz_kinect(:,2).*(-1)];
       
        ptIndexOnBody_kinect=find((xyz_kinect(:,1)>-1.2 & xyz_kinect(:,1)<0) & (xyz_kinect(:,2)>0.9 & xyz_kinect(:,2)<6.5) ...
            & (xyz_kinect(:,3)>-1 & xyz_kinect(:,3)<1));
        xyz_kinect2=xyz_kinect(ptIndexOnBody_kinect,:);%反射器的Kinect点云
        
%             if(mean(xyz_kinect2(:,2))>4.5 || mean(xyz_kinect2(:,2))<1.5)
%                 continue;
%             end

        %ti 读取ti数据
       
        ptcloud=ptcloudData(i_ti);
        xyz = readXYZ(ptcloud);
        xyz_ti1=[(-1).*xyz(:,2),xyz(:,1),xyz(:,3)];
        
        %选出范围内的点
        ptIndexOnBody_ti1=find((xyz_ti1(:,1)>-1.2 & xyz_ti1(:,1)<0) & (xyz_ti1(:,2)>1 & xyz_ti1(:,2)<5) ...
            & (xyz_ti1(:,3)>-0.975 & xyz_ti1(:,3)<0.8));
        xyz_ti2=xyz_ti1(ptIndexOnBody_ti1,:); %人体上的ti点云

        fprintf("xyz_kinect2\n");
        xyz_kinect2;

        fprintf("xyz_ti2\n");
        xyz_ti2;

end
