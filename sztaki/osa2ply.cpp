#include "System.h"
#include "igl/writePLY.h"

int main(int argc, char **argv)
{
  if(argc < 3)
  {
    cerr << endl << "Usage: osa2ply path_to_vocabulary path_to_settings" << endl;
    return 1;
  }
  ORB_SLAM3::System SLAM(argv[1],argv[2],ORB_SLAM3::System::IMU_MONOCULAR,false, 0, std::string(), true);

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
  return 0;
}

