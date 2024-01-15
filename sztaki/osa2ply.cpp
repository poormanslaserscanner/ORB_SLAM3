#include "System.h"
#include "igl/writePLY.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"


int main(int argc, char **argv)
{
	// rosbag::Bag bag;
    // bag.open("small.bag", rosbag::bagmode::Read);

    // std::vector<std::string> topics;
    // topics.push_back(std::string("/gopro/image_raw/compressed"));

    // rosbag::View view(bag, rosbag::TopicQuery(topics));

	// int siz = view.size();
	// for(rosbag::MessageInstance const m: view)
    // {
		
	// 	double time = m.getTime().toSec();
	// 	sensor_msgs::CompressedImage::ConstPtr rosimgptr = m.instantiate<sensor_msgs::CompressedImage>();
	// 	if (rosimgptr != nullptr)
	// 	{
	// 		cv_bridge::CvImagePtr cv_ptr;
	// 		try
	// 		{
	// 			cv_ptr = cv_bridge::toCvCopy(rosimgptr);
	// 		}
	// 		catch(cv_bridge::Exception& e)
	// 		{
	// 			continue;
	// 		}
	// 	    cv::imshow("image", cv_ptr->image);
	// 		cv::waitKey(20);

	// 	}
    // }

    // bag.close();



	// return 0;
  if(argc < 3)
  {
    cerr << endl << "Usage: osa2ply path_to_vocabulary path_to_settings" << endl;
    return 1;
  }
  bool useviewer = true;
  ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR,useviewer, 0, std::string(), true);
  
  ORB_SLAM3::Viewer *viewer = SLAM.GetViewer();
  if (viewer)
  {
	ORB_SLAM3::LoopClosing *lcptr = SLAM.LoopCloser();
	if (lcptr)
	{
		std::vector<ORB_SLAM3::Map*> maps = SLAM.GetAllMaps();
		for (ORB_SLAM3::Map* map : maps)
		{
			if (map)
			{
				std::vector<ORB_SLAM3::KeyFrame*> points = map->GetAllKeyFrames();
				for (ORB_SLAM3::KeyFrame* point : points)
				{
					if (point)
					{
						point->UpdateConnections();
						point->UpdateBestCovisibles();
					}
				}
			}
		}
	}

	if (argc > 3)
	{
		viewer->InitBag(argv[3]);
	}
	viewer->Run();
  }

  return 0;
  ORB_SLAM3::LoopClosing *lcptr = SLAM.LoopCloser();
  if (lcptr)
  {
	std::vector<ORB_SLAM3::Map*> maps = SLAM.GetAllMaps();
	for (ORB_SLAM3::Map* map : maps)
	{
		if (map)
		{
			std::vector<ORB_SLAM3::KeyFrame*> points = map->GetAllKeyFrames();
			for (ORB_SLAM3::KeyFrame* point : points)
			{
				if (point)
				{
					point->UpdateConnections();
					point->UpdateBestCovisibles();
					lcptr->InsertKeyFrame(point);
				}
			}
		}
	}
	lcptr->RunSeq();
  }


  const std::vector<ORB_SLAM3::Map*> maps = SLAM.GetAllMaps();
  int counter = 0;
  for (ORB_SLAM3::Map* map : maps)
  {
	if (map)
	{
		const std::vector<ORB_SLAM3::MapPoint*> points = map->GetAllMapPoints();
		for (ORB_SLAM3::MapPoint* point : points)
		{
			if (point)
			{
				++counter;
			}
		}
	}
  }
  Eigen::MatrixX3f verts;
  Eigen::MatrixX3f normals;
  verts.setZero(counter,3);
  normals.setZero(counter,3);
  counter = 0;
  for (ORB_SLAM3::Map* map : maps)
  {
	if (map)
	{
		const std::vector<ORB_SLAM3::MapPoint*> points = map->GetAllMapPoints();
		for (ORB_SLAM3::MapPoint* point : points)
		{
			if (point)
			{
				verts.row(counter) = point->GetWorldPos();
				normals.row(counter) = point->GetNormal();
				++counter;
			}
		}
	}
  }
  igl::writePLY("out.ply", verts, Eigen::MatrixX3i(), Eigen::MatrixX2i(), normals, Eigen::MatrixX2i(), Eigen::MatrixXf(), {}, Eigen::MatrixXf(), {}, Eigen::MatrixXf(), {}, {}, igl::FileEncoding::Binary);
  SLAM.Shutdown();
  return 0;
}

