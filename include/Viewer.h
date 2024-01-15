/**
* This file is part of ORB-SLAM3
*
* Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-SLAM3.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef VIEWER_H
#define VIEWER_H

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"
#include "Settings.h"

#include <mutex>
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/image_encodings.h"
#include "cv_bridge/cv_bridge.h"

namespace ORB_SLAM3
{

class Tracking;
class FrameDrawer;
class MapDrawer;
class System;
class Settings;

class Viewer
{
public:
    class BagImage;
/*    {
    private:
        rosbag::Bag bag;
    public:
        BagImage(const std::string &bagfile)
        {
            bag.open(bagfile, rosbag::bagmode::Read);
        }

        ~BagImage(void)
        {
            bag.close();
        }

        cv::Mat	GetImage(double sec)
        {
            rosbag::View view(bag, rosbag::TopicQuery("/gopro/image_raw/compressed"), ros::Time(sec - 1.0/240), ros::Time(sec + 1.0/240));
            
            for(rosbag::MessageInstance const m: view)
            {
                double time = m.getTime().toSec();
                sensor_msgs::CompressedImage::ConstPtr rosimgptr = m.instantiate<sensor_msgs::CompressedImage>();
                if (rosimgptr != nullptr)
                {
                    cv_bridge::CvImagePtr cv_ptr;
                    try
                    {
                        cv_ptr = cv_bridge::toCvCopy(rosimgptr);
                        return cv_ptr->image;

                    }
                    catch(cv_bridge::Exception& e)
                    {
                        continue;
                    }
                }
            }
            return cv::Mat();
        }

    };*/

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Viewer(System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking, const string &strSettingPath, Settings* settings);

    void InitBag(const std::string &bagfile);
    // {
    //     bagimgptr = std::make_unique<BagImage>(bagfile);
    // }

    void newParameterLoader(Settings* settings);

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    void RequestFinish();

    void RequestStop();

    bool isFinished();

    bool isStopped();

    bool isStepByStep();

    void Release();

    //void SetTrackingPause();

    bool both;
    bool offline_mode;
private:

    bool ParseViewerParamFile(cv::FileStorage &fSettings);

    bool Stop();

    System* mpSystem;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    Tracking* mpTracker;

    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;
    float mImageViewerScale;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;

    bool mbStopTrack;

    std::unique_ptr<BagImage> bagimgptr;

};

}


#endif // VIEWER_H
	

