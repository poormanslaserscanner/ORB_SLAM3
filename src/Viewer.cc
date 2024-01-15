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


#include "Viewer.h"
#include <pangolin/pangolin.h>

#include <mutex>

namespace ORB_SLAM3
{

    class Viewer::BagImage
    {
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

        void DisplayImage(const char *label, double sec, double scalefactor)
        {
                double timestamp = sec;
                cv::Mat mat = GetImage(timestamp);
                if (!mat.empty())
                {
                    int xs = mat.cols * scalefactor;
                    int ys = mat.rows * scalefactor;
                    cv::resize(mat, mat, cv::Size(xs, ys));
                    cv::imshow(label, mat);
                    cv::waitKey(50);
                }

        }

    };

    void Viewer::InitBag(const std::string &bagfile)
    {
        bagimgptr = std::make_unique<BagImage>(bagfile);
    }

Viewer::Viewer(System* pSystem, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Tracking *pTracking, const string &strSettingPath, Settings* settings):
    both(false), mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking),
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    if(settings){
        newParameterLoader(settings);
    }
    else{

        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        bool is_correct = ParseViewerParamFile(fSettings);

        if(!is_correct)
        {
            std::cerr << "**ERROR in the config file, the format is not correct**" << std::endl;
            try
            {
                throw -1;
            }
            catch(exception &e)
            {

            }
        }
    }

    mbStopTrack = false;
}

void Viewer::newParameterLoader(Settings *settings) {
    mImageViewerScale = 1.f;

    float fps = settings->fps();
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    cv::Size imSize = settings->newImSize();
    mImageHeight = imSize.height;
    mImageWidth = imSize.width;

    mImageViewerScale = settings->imageViewerScale();
    mViewpointX = settings->viewPointX();
    mViewpointY = settings->viewPointY();
    mViewpointZ = settings->viewPointZ();
    mViewpointF = settings->viewPointF();
}

bool Viewer::ParseViewerParamFile(cv::FileStorage &fSettings)
{
    bool b_miss_params = false;
    mImageViewerScale = 1.f;

    float fps = fSettings["Camera.fps"];
    if(fps<1)
        fps=30;
    mT = 1e3/fps;

    cv::FileNode node = fSettings["Camera.width"];
    if(!node.empty())
    {
        mImageWidth = node.real();
    }
    else
    {
        std::cerr << "*Camera.width parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Camera.height"];
    if(!node.empty())
    {
        mImageHeight = node.real();
    }
    else
    {
        std::cerr << "*Camera.height parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.imageViewScale"];
    if(!node.empty())
    {
        mImageViewerScale = node.real();
    }

    node = fSettings["Viewer.ViewpointX"];
    if(!node.empty())
    {
        mViewpointX = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointX parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointY"];
    if(!node.empty())
    {
        mViewpointY = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointY parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointZ"];
    if(!node.empty())
    {
        mViewpointZ = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointZ parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    node = fSettings["Viewer.ViewpointF"];
    if(!node.empty())
    {
        mViewpointF = node.real();
    }
    else
    {
        std::cerr << "*Viewer.ViewpointF parameter doesn't exist or is not a real number*" << std::endl;
        b_miss_params = true;
    }

    return !b_miss_params;
}

struct SlamHandler : public pangolin::Handler3D
{
    void Keyboard(pangolin::View& v, unsigned char key, int x, int y, bool pressed)
    {
        if (!pressed)
        {
            if (key == 231)
            {
                dwreleased = true;
            }
            if (key == 230)
            {
                rgreleased = true;
            }
            if (key == 229)
            {
                upreleased = true;
            }
            if (key == 228)
            {
                lfreleased = true;
            }
        }
        pangolin::Handler3D::Keyboard(v, key, x, y, pressed);
    }

    template <class... Args>
    SlamHandler(Args&&... args) : pangolin::Handler3D(std::forward<Args>(args)...)
    {
        SlamReset();
    }

    void SlamReset(void)
    {
        lfreleased = rgreleased =upreleased = dwreleased = false;
    }

    bool up(void)
    {
        return check(upreleased);
        if (upreleased)
        {
            upreleased = false;
            return true;
        }
        return false;
    }

    bool down(void)
    {
        return check(dwreleased);
        if (dwreleased)
        {
            dwreleased = false;
            return true;
        }
        return false;
    }

    bool left(void)
    {
        return check(lfreleased);
    }

    bool right(void)
    {
        return check(rgreleased);
    }

private:
    bool check(bool &var)
    {
        if (var)
        {
            var = false;
            return true;
        }
        return false;
    }

    bool upreleased;
    bool dwreleased;
    bool lfreleased;
    bool rgreleased;




};

void Viewer::Run()
{
    mbFinished = false;
    mbStopped = false;

    pangolin::CreateWindowAndBind("ORB-SLAM3: Map Viewer",1024,768);

    // 3D Mouse handler requires depth testing to be enabled
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",false,true);
    pangolin::Var<bool> menuCamView("menu.Camera View",false,false);
    pangolin::Var<bool> menuTopView("menu.Top View",false,false);
    // pangolin::Var<bool> menuSideView("menu.Side View",false,false);
    pangolin::Var<bool> menuShowPoints("menu.Show Points",true,true);
    pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames",true,true);
    pangolin::Var<bool> menuShowGraph("menu.Show Graph",false,true);
    pangolin::Var<bool> menuShowInertialGraph("menu.Show Inertial Graph",true,true);
    pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode",false,true);
    pangolin::Var<bool> menuReset("menu.Reset",false,false);
    pangolin::Var<bool> menuStop("menu.Stop",false,false);
    pangolin::Var<bool> menuStepByStep("menu.Step By Step",false,true);  // false, true
    pangolin::Var<bool> menuStep("menu.Step",false,false);

    pangolin::Var<bool> menuShowOptLba("menu.Show LBA opt", false, true);
    pangolin::Var<int> menuNumOfMaps("menu.Num of maps", 0);
    pangolin::Var<int> menuShowMap("menu.Show maps", 0, 0, 10);

    std::unique_ptr<pangolin::Var<bool>> menuSetFrm0;
    std::unique_ptr<pangolin::Var<bool>> menuSetFrm1;
    std::unique_ptr<pangolin::Var<bool>> doloopclose;
    std::unique_ptr<pangolin::Var<unsigned long>> menuFrm0;
    std::unique_ptr<pangolin::Var<unsigned long>> menuFrm1;
    KeyFrame *menukf0 = nullptr;
    KeyFrame *menukf1 = nullptr;
    if (offline_mode)
    {
        menuSetFrm0 = std::make_unique<pangolin::Var<bool>>("menu.Set frm0", false, false);
        menuFrm0 = std::make_unique<pangolin::Var<unsigned long>>("menu.frm0", 0);
        menuSetFrm1 = std::make_unique<pangolin::Var<bool>>("menu.Set frm1", false, false);
        menuFrm1 = std::make_unique<pangolin::Var<unsigned long>>("menu.frm1", 0);
        doloopclose = std::make_unique<pangolin::Var<bool>>("menu.try close loop", false, false);
    }
    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
            .SetHandler(new SlamHandler(s_cam));

    pangolin::OpenGlMatrix Twc, Twr;
    Twc.SetIdentity();
    pangolin::OpenGlMatrix Ow; // Oriented with g in the z axis
    Ow.SetIdentity();
    cv::namedWindow("ORB-SLAM3: Current Frame");

    bool bFollow = true;
    bool bLocalizationMode = false;
    bool bStepByStep = false;
    bool bCameraView = true;

    if(mpTracker->mSensor == mpSystem->MONOCULAR || mpTracker->mSensor == mpSystem->STEREO || mpTracker->mSensor == mpSystem->RGBD)
    {
        menuShowGraph = true;
    }

    float trackedImageScale = mpTracker->GetImageScale();

    cout << "Starting the Viewer" << endl;
    Eigen::Vector3d selected_point = Eigen::Vector3d::Zero();
    int the_shown_map = 0;
    while(1)
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc,Ow);

        if(mbStopTrack)
        {
            menuStepByStep = true;
            mbStopTrack = false;
        }

        if(menuFollowCamera && bFollow)
        {
            if(bCameraView)
                s_cam.Follow(Twc);
            else
                s_cam.Follow(Ow);
        }
        else if(menuFollowCamera && !bFollow)
        {
            if(bCameraView)
            {
                s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000));
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
                s_cam.Follow(Twc);
            }
            else
            {
                s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,3000,3000,512,389,0.1,1000));
                s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,0.01,10, 0,0,0,0.0,0.0, 1.0));
                s_cam.Follow(Ow);
            }
            bFollow = true;
        }
        else if(!menuFollowCamera && bFollow)
        {
            bFollow = false;
        }

        if(menuCamView)
        {
            menuCamView = false;
            bCameraView = true;
            s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,10000));
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
            s_cam.Follow(Twc);
        }

        if(menuTopView && mpMapDrawer->mpAtlas->isImuInitialized())
        {
            menuTopView = false;
            bCameraView = false;
            s_cam.SetProjectionMatrix(pangolin::ProjectionMatrix(1024,768,3000,3000,512,389,0.1,10000));
            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(0,0.01,50, 0,0,0,0.0,0.0, 1.0));
            s_cam.Follow(Ow);
        }

        if(menuLocalizationMode && !bLocalizationMode)
        {
            mpSystem->ActivateLocalizationMode();
            bLocalizationMode = true;
        }
        else if(!menuLocalizationMode && bLocalizationMode)
        {
            mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
        }

        if(menuStepByStep && !bStepByStep)
        {
            //cout << "Viewer: step by step" << endl;
            mpTracker->SetStepByStep(true);
            bStepByStep = true;
        }
        else if(!menuStepByStep && bStepByStep)
        {
            mpTracker->SetStepByStep(false);
            bStepByStep = false;
        }

        if(menuStep)
        {
            mpTracker->mbStep = true;
            menuStep = false;
        }

        Map *the_map = nullptr;
        if (mpSystem)
        {
            const auto maps = mpSystem->GetAllMaps();           
            menuNumOfMaps = maps.size();
            if (menuShowMap > menuNumOfMaps)
            {
                menuShowMap = menuNumOfMaps;
            }
            if (menuShowMap)
            {
                the_map = maps[menuShowMap - 1];
            }
        }


        d_cam.Activate(s_cam);
        s_cam.Apply();
        glClearColor(1.0f,1.0f,1.0f,1.0f);
        mpMapDrawer->DrawCurrentCamera(Twc);
        KeyFrame* selected_frame_ptr = 0;
        if (d_cam.handler)
        {
            pangolin::Handler3D *hpt = dynamic_cast<pangolin::Handler3D*>(d_cam.handler);
            if (hpt && hpt->KeyState() == (pangolin::KeyModifierCtrl | pangolin::MouseButtonLeft) )
            {
                selected_point = hpt->Selected_P_w();
            }
        }

        if(menuShowKeyFrames || menuShowGraph || menuShowInertialGraph || menuShowOptLba)
        {
            selected_frame_ptr = mpMapDrawer->DrawKeyFrames(menuShowKeyFrames,menuShowGraph, menuShowInertialGraph, menuShowOptLba, selected_point, the_map);
        }
        if(menuShowPoints)
            mpMapDrawer->DrawMapPoints(the_map);
        pangolin::FinishFrame();
        the_shown_map = menuShowMap;


        if (offline_mode && mpSystem)
        {
            unsigned long frm_id = selected_frame_ptr ? selected_frame_ptr->mnId : 0UL;
            if (pangolin::Pushed(*menuSetFrm0))
            {
                *menuFrm0 = frm_id;
                menukf0 = selected_frame_ptr;
            }
            if (pangolin::Pushed(*menuSetFrm1))
            {
                *menuFrm1 = frm_id;
                menukf1 = selected_frame_ptr;

            }
            if (bagimgptr)
            {
                if (menukf0)
                {
                    bagimgptr->DisplayImage("Image0", menukf0->mTimeStamp, mImageViewerScale);
                    SlamHandler *hpt = dynamic_cast<SlamHandler*>(d_cam.handler);
                    {
                        if (hpt && hpt->down() )
                        {
                            if (selected_frame_ptr->mPrevKF)
                            {
                                menukf0 = menukf0->mPrevKF;
                                *menuFrm0 = menukf0->mnId;
                            }
                        }                
                        if (hpt && hpt->up() )
                        {
                                menukf0 = menukf0->mNextKF;
                                *menuFrm0 = menukf0->mnId;
                        }
                    }                

                    
                }
                if (menukf1)
                {
                    bagimgptr->DisplayImage("Image1", menukf1->mTimeStamp, mImageViewerScale);
                    SlamHandler *hpt = dynamic_cast<SlamHandler*>(d_cam.handler);
                    {
                        if (hpt && hpt->left())
                        {
                            if (selected_frame_ptr->mPrevKF)
                            {
                                menukf1 = menukf1->mPrevKF;
                                *menuFrm1 = menukf1->mnId;
                            }
                        }                
                        if (hpt && hpt->right())
                        {
                                menukf1 = menukf1->mNextKF;
                                *menuFrm1 = menukf1->mnId;
                        }
                    }                

                }
                if (pangolin::Pushed(*doloopclose))
                {

                    ORB_SLAM3::LoopClosing *lcptr = mpSystem->LoopCloser();
                    if (lcptr)
                    {
                        if (menukf0 && menukf1)
                        {
                        lcptr->InsertKeyFrame(menukf0);
                        lcptr->RunSeq(menukf1);
                        }
                    }
                }

            }
            if (0 && selected_frame_ptr && bagimgptr)
            {
                double timestamp = selected_frame_ptr->mTimeStamp;
                cv::Mat mat = bagimgptr->GetImage(timestamp);
                if (!mat.empty())
                {
                    int xs = mat.cols * mImageViewerScale;
                    int ys = mat.rows * mImageViewerScale;
                    cv::resize(mat, mat, cv::Size(xs, ys));
                    cv::imshow("ORB-SLAM3: Selected Frame",mat);
                    cv::waitKey(mT);
                }
                SlamHandler *hpt = dynamic_cast<SlamHandler*>(d_cam.handler);
                
                if (hpt && hpt->down() )
                {
                    if (selected_frame_ptr->mPrevKF)
                    {
                        selected_point = selected_frame_ptr->mPrevKF->GetCameraCenter().cast<double>();
                    }
                }                
                if (hpt && hpt->up() )
                {
                    if (selected_frame_ptr->mNextKF)
                    {
                        selected_point = selected_frame_ptr->mNextKF->GetCameraCenter().cast<double>();
                    }
                }                
            }
        }


        cv::Mat toShow;
        cv::Mat im = mpFrameDrawer->DrawFrame(trackedImageScale);

        if(both){
            cv::Mat imRight = mpFrameDrawer->DrawRightFrame(trackedImageScale);
            cv::hconcat(im,imRight,toShow);
        }
        else{
            toShow = im;
        }

        if(mImageViewerScale != 1.f)
        {
            int width = toShow.cols * mImageViewerScale;
            int height = toShow.rows * mImageViewerScale;
            cv::resize(toShow, toShow, cv::Size(width, height));
        }

        cv::imshow("ORB-SLAM3: Current Frame",toShow);
        cv::waitKey(mT);

        if(menuReset)
        {
            menuShowGraph = true;
            menuShowInertialGraph = true;
            menuShowKeyFrames = true;
            menuShowPoints = true;
            menuLocalizationMode = false;
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();
            bLocalizationMode = false;
            bFollow = true;
            menuFollowCamera = true;
            mpSystem->ResetActiveMap();
            menuReset = false;
        }

        if(menuStop)
        {
            if(bLocalizationMode)
                mpSystem->DeactivateLocalizationMode();

            // Stop all threads
            mpSystem->Shutdown();

            // Save camera trajectory
            mpSystem->SaveTrajectoryEuRoC("CameraTrajectory.txt");
            mpSystem->SaveKeyFrameTrajectoryEuRoC("KeyFrameTrajectory.txt");
            menuStop = false;
        }

        if(Stop())
        {
            while(isStopped())
            {
                usleep(3000);
            }
        }

        if(CheckFinish())
            break;
    }

    SetFinish();
}

void Viewer::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

bool Viewer::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Viewer::SetFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
}

bool Viewer::isFinished()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinished;
}

void Viewer::RequestStop()
{
    unique_lock<mutex> lock(mMutexStop);
    if(!mbStopped)
        mbStopRequested = true;
}

bool Viewer::isStopped()
{
    unique_lock<mutex> lock(mMutexStop);
    return mbStopped;
}

bool Viewer::Stop()
{
    unique_lock<mutex> lock(mMutexStop);
    unique_lock<mutex> lock2(mMutexFinish);

    if(mbFinishRequested)
        return false;
    else if(mbStopRequested)
    {
        mbStopped = true;
        mbStopRequested = false;
        return true;
    }

    return false;

}

void Viewer::Release()
{
    unique_lock<mutex> lock(mMutexStop);
    mbStopped = false;
}

/*void Viewer::SetTrackingPause()
{
    mbStopTrack = true;
}*/

}
