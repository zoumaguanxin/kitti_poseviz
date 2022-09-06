

#include <string>
#include <iostream>
#include <fstream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/common_headers.h>
#include <Eigen/Dense>
#include <dirent.h>

using namespace std;

ifstream& operator>>(ifstream& in, Eigen::Matrix<float,3,4>& transform)
{
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<4;j++)
		{
			in>>transform(i,j);
			/*
			if(in.fail())
			{
				throw std::runtime_error("can not parse this file, please check file format, this needs 3*4 matrix arranged in row major");
			}
			*/
		}
	}
	return in;
}

pcl::PointCloud<pcl::PointXYZ> readPositionFromKITTI(std::string path,std::vector<Eigen::Matrix<float,3,4> >& transforms)
{
	ifstream file;
	file.open(path, ios::in);
	pcl::PointCloud<pcl::PointXYZ> temCloud;
	if(file.is_open())
	{
		while(file.peek()!=EOF)
		{
			Eigen::Matrix<float,3,4> Transform;
			file>>Transform;
			transforms.push_back(Transform);
			pcl::PointXYZ temPt;
			temPt.x=Transform(0,3);
			temPt.y=Transform(1,3);
			temPt.z=Transform(2,3);
			temCloud.push_back(temPt);
		}
	}
	else
	{
		throw std::runtime_error("can not open file"+path);
	}
	return temCloud;
}

std::map<std::string, Eigen::Matrix<float,3,4> > readCalibfromKitti(std::string path)
{
	ifstream file;
	file.open(path, std::ios::in);
	std::map<std::string, Eigen::Matrix<float,3,4> > calibs;
	
	if(file.is_open())
	{
		char line[1024];
		while(file.getline(line, sizeof(line)))
		{
			Eigen::Matrix<float,3,4> transform;
			stringstream s(line);
			std::string prefix;
			s>>prefix;
			for(int i=0;i<3;i++)
			{
				for(int j=0;j<4;j++)
				{
					s>>transform(i,j);
				}
			}
			calibs.insert(std::make_pair(prefix,transform));
		}
	}
	else
	{
		throw std::runtime_error("can not open file"+path);
	}
	return calibs;
}

boost::shared_ptr<pcl::visualization::PCLVisualizer> twoVis(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc1, pcl::PointCloud<pcl::PointXYZ>::ConstPtr pc2)
{
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("point cloud"));
	viewer->setBackgroundColor(255,255,255);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red_color(pc1, 255, 0, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green_color(pc2, 0,0,255);
	viewer->addPointCloud<pcl::PointXYZ>(pc1,red_color,"pcd1"); 
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE,1,"pcd1");
	viewer->addPointCloud<pcl::PointXYZ >(pc2,green_color,"pcd2");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	while(!viewer->wasStopped())
	{
		viewer->spinOnce(1000);
		boost::this_thread::sleep(boost::posix_time::microsec(1000));
	}
	return viewer;  
}


ofstream & operator <<(ofstream & outf, const Eigen::Matrix<float,3,4> & pose)
{
  for(int i=0;i<3;i++)
  {
    for(int j=0;j<4;j++)
    {
       if(i==2&&j==3)
	{
       		outf<<pose(i,j);
	}
	else
	{
		outf<<pose(i,j)<<' '; 
	}  
    }
  }
  return outf;
}

int main(int argc, char **argv) {


	
	bool laser_to_camera_init_known=false;
	bool laser_to_laser_init_known=true;
	bool camera_to_camera_init_known=false;
	bool camera_to_laser_init_known=false;
        //TODO get path by parsing argv. if there are a global source label, use it directly, otherwise, append it to my default path.


	//string path="/data/datasets/KITTI/dev/poseviz/data/imu/01_imu.txt";
	//string posesdir="../data/imu/01_calibed.txt";
	
	
	//your data
	//string path="/home/data/projects/kitti_reader/data/no_loop_squence00.txt";
        //string path="/data/datasets/KITTI/dev/poseviz/data/my/01/01_licp_imu_calibed.txt";
	//string posesdir="../data/my/01/01_licpo_result.txt";
	//string path="/home/data/projects/kitti_reader/data/00_190408_icp_no_loop.txt";

	//string path="../data/my/00_0409.txt";
	//string posesdir="../data/my/01_calibed.txt";

	//loam        

	//string path="/data/datasets/KITTI/results/aloam/01_aft_mapped_to_init_high_frec.txt";
	//string posesdir="../data/aloam/01.txt";

	string path="/data/datasets/KITTI/results/legoloamjq/result_trace/10_integrated_init.txt";
	//string path="../data/lego_loam/00_camera_2_camera_init.txt";
	string posesdir="../data/lego_loam/10_test.txt";	


	//groud truth
	string path1="/data/datasets/KITTI_odometry/data_odometry_poses/dataset/poses/10.txt";
	
	//calib file
	string calibPath="/data/datasets/KITTI_odometry/data_calibration/sequences/10/calib.txt";

	std::vector<Eigen::Matrix<float,3,4> > my_tfs, my_tfs_calibed, groundtruth_tfs;
	pcl::PointCloud<pcl::PointXYZ> positions= readPositionFromKITTI(path,my_tfs);
	pcl::PointCloud<pcl::PointXYZ> positions_groundth=readPositionFromKITTI(path1, groundtruth_tfs);
	
	std::map<std::string, Eigen::Matrix<float,3,4> > calibs=readCalibfromKitti(calibPath);
	Eigen::Matrix<float,3,4> p0=calibs.at("P0:");
	std::cout<<p0<<std::endl;
	Eigen::Matrix<float,3,4> Tr=calibs.at("Tr:");
	Eigen::Matrix3f Rr=Tr.block(0,0,3,3);
	Eigen::Vector3f tr=Tr.block(0,3,3,1);
	pcl::PointCloud<pcl::PointXYZ> positions2camera0;
	
	
	for(Eigen::Matrix<float,3,4> tf: my_tfs)
	{
		Eigen::Matrix3f R_camera2camerainit;
		Eigen::Vector3f t_camera2camerainit=tf.block(0,3,3,1);
		Eigen::Matrix3f R_t=tf.block(0,0,3,3);
		Eigen::Vector3f trans_t=tf.block(0,3,3,1);
		if(camera_to_camera_init_known)
		{
			R_camera2camerainit=R_t;
			t_camera2camerainit=trans_t;
		
		}
		else if(laser_to_camera_init_known)
		{
			R_camera2camerainit=Rr*R_t;
			t_camera2camerainit=Rr*trans_t+tr;

		}
		else if(laser_to_laser_init_known)
		{
			R_camera2camerainit=Rr*R_t*Rr.inverse();
			t_camera2camerainit=tr+Rr*(trans_t-R_t*Rr.inverse()*tr);
			
		}
		else if(camera_to_laser_init_known)
		{	
			R_camera2camerainit=R_t*Rr.inverse();
			t_camera2camerainit=trans_t-R_t*Rr.inverse()*tr;
		}
		Eigen::Matrix<float,3,4> tf_calibed;
		tf_calibed.block(0,0,3,3)=R_camera2camerainit;
		tf_calibed.block(0,3,3,1)=t_camera2camerainit;
		my_tfs_calibed.push_back(tf_calibed);
		positions2camera0.push_back(pcl::PointXYZ(t_camera2camerainit(0),t_camera2camerainit(1),t_camera2camerainit(2)));
	}

	ofstream ofile;
	ofile.open(posesdir.c_str(),ios_base::out);
	if(ofile.is_open())
	{
		if(ofile.good())
		{
			for(auto pose:my_tfs_calibed)
			{
				
				ofile<<pose<<std::endl;
				
			}
		}
	}
	else
	{
		std::cout<<"cannot open the file"<<std::endl;
		exit(0);
	}
	std::cout<<"poses have been writed into "<<posesdir<<std::endl;  
	
	
	twoVis(positions2camera0.makeShared(),positions_groundth.makeShared());
	return 0;
}


