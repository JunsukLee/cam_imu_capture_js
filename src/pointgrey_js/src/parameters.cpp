#include <pointgrey_js/parameters.h>

cv::Mat RectifyMap_cam0_map1, RectifyMap_cam0_map2;
cv::Mat RectifyMap_cam1_map1, RectifyMap_cam1_map2; 

int readStereoParameters(std::string config_file)
// int readStereoParameters(std::string config_file,
//                          cv::Mat &_cv_body2Cam0,
//                          cv::Mat &_cv_body2Cam1,
//                          cv::Mat &_intrinsicMatrix_Cam0,
//                          cv::Mat &_intrinsicMatrix_Cam1,
//                          cv::Mat &_distCoeffsMatrix_Cam0,
//                          cv::Mat &_distCoeffsMatrix_Cam1)
{
    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
        return -1;
    }
    cv::Size imageSize;
    int imageSizeHeight, imageSizeWidth;

    cv::Mat cv_body2Cam0; // 4x4 matrix {body to Cam0 Transformation-Matrix}
    cv::Mat cv_body2Cam1; // 4x4 matrix {body to Cam1 Transformation-Matrix}
    cv::Mat cv_Cam02Cam1; // 4x4 matrix {Cam0 to Cam1 Transformation-Matrix}
    cv::Mat rvecCam0 = cv::Mat::eye(3, 3, cv::DataType<double>::type);
    cv::Mat rvecCam1 = cv::Mat::eye(3, 3, cv::DataType<double>::type);
    cv::Mat rvecCam1toCam0 = cv::Mat::eye(3, 3, cv::DataType<double>::type);
    cv::Mat tvecCam0 = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
    cv::Mat tvecCam1 = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
    cv::Mat tvecCam1toCam0 = cv::Mat::zeros(3, 1, cv::DataType<double>::type);
    
    cv::Mat intrinsicMatrix_Cam0 = cv::Mat::eye(3, 3, cv::DataType<double>::type);
    cv::Mat intrinsicMatrix_Cam1 = cv::Mat::eye(3, 3, cv::DataType<double>::type);

    cv::Mat distCoeffsMatrix_Cam0 = cv::Mat::zeros(5, 1, cv::DataType<double>::type);
    cv::Mat distCoeffsMatrix_Cam1 = cv::Mat::zeros(5, 1, cv::DataType<double>::type);
    double cam0_distortion[5] = {0, }; // k1, k2, k3, p1, p2
    double cam1_distortion[5] = {0, }; // k1, k2, k3, p1, p2

    cv::Mat stereoRectify_R1;
    cv::Mat stereoRectify_R2;
    cv::Mat stereoRectify_P1;
    cv::Mat stereoRectify_P2;
    cv::Mat stereoRectify_Q;

    

    // Read ImageSize
    {
        imageSizeHeight = fsSettings["image_height"];
        imageSizeWidth  = fsSettings["image_width"];
        imageSize = cv::Size(imageSizeHeight, imageSizeWidth);
        std::cerr << "imageSize: " << imageSize << std::endl;
    }

    // Read 4x4 Transformation-Matrix
    {
        fsSettings["body_T_cam0"] >> cv_body2Cam0;
        fsSettings["body_T_cam1"] >> cv_body2Cam1;
        //cv_Cam02Cam1 = cv_body2Cam1.inv();
        cv_Cam02Cam1 = cv_body2Cam1;
        
        for(int r = 0; r < 3 ; ++r)
        {
            for(int c = 0; c < 3 ; ++c)
            {
                rvecCam0.ptr<double>(r,c)[0] = cv_body2Cam0.ptr<double>(r,c)[0];
                rvecCam1.ptr<double>(r,c)[0] = cv_Cam02Cam1.ptr<double>(r,c)[0];
            }
            tvecCam0.ptr<double>(r,0)[0] = cv_body2Cam0.ptr<double>(r,3)[0];
            tvecCam1.ptr<double>(r,0)[0] = cv_Cam02Cam1.ptr<double>(r,3)[0];
        }
        rvecCam1toCam0 = rvecCam1.inv();
        tvecCam1toCam0 = rvecCam1.inv() * (-1 * tvecCam1);
    }

    // Read intrinsic
    {
        // Cam0
        intrinsicMatrix_Cam0.ptr<double>(0,0)[0] = (double)fsSettings["cam0_projection_parameters"]["fx"];
        intrinsicMatrix_Cam0.ptr<double>(1,1)[0] = (double)fsSettings["cam0_projection_parameters"]["fx"];
        intrinsicMatrix_Cam0.ptr<double>(0,2)[0] = (double)fsSettings["cam0_projection_parameters"]["cx"];
        intrinsicMatrix_Cam0.ptr<double>(1,2)[0] = (double)fsSettings["cam0_projection_parameters"]["cy"];

        // Cam1
        intrinsicMatrix_Cam1.ptr<double>(0,0)[0] = (double)fsSettings["cam1_projection_parameters"]["fx"];
        intrinsicMatrix_Cam1.ptr<double>(1,1)[0] = (double)fsSettings["cam1_projection_parameters"]["fx"];
        intrinsicMatrix_Cam1.ptr<double>(0,2)[0] = (double)fsSettings["cam1_projection_parameters"]["cx"];
        intrinsicMatrix_Cam1.ptr<double>(1,2)[0] = (double)fsSettings["cam1_projection_parameters"]["cy"];
    }

    // Read distortion
    {
        // Cam0
        distCoeffsMatrix_Cam0.ptr<double>(0,0)[0] = (double)fsSettings["cam0_distortion_parameters"]["k1"];
        distCoeffsMatrix_Cam0.ptr<double>(1,0)[0] = (double)fsSettings["cam0_distortion_parameters"]["k2"];        
        distCoeffsMatrix_Cam0.ptr<double>(2,0)[0] = (double)fsSettings["cam0_distortion_parameters"]["p1"];
        distCoeffsMatrix_Cam0.ptr<double>(3,0)[0] = (double)fsSettings["cam0_distortion_parameters"]["p2"];
        distCoeffsMatrix_Cam0.ptr<double>(4,0)[0] = (double)fsSettings["cam0_distortion_parameters"]["k3"];

        // Cam1
        distCoeffsMatrix_Cam1.ptr<double>(0,0)[0] = (double)fsSettings["cam1_distortion_parameters"]["k1"];
        distCoeffsMatrix_Cam1.ptr<double>(1,0)[0] = (double)fsSettings["cam1_distortion_parameters"]["k2"];
        distCoeffsMatrix_Cam1.ptr<double>(2,0)[0] = (double)fsSettings["cam1_distortion_parameters"]["p1"];
        distCoeffsMatrix_Cam1.ptr<double>(3,0)[0] = (double)fsSettings["cam1_distortion_parameters"]["p2"];
        distCoeffsMatrix_Cam1.ptr<double>(4,0)[0] = (double)fsSettings["cam1_distortion_parameters"]["k3"];
    }

    //stereoRectify
    {
        cv::stereoRectify(intrinsicMatrix_Cam0,
                          distCoeffsMatrix_Cam0,
                          intrinsicMatrix_Cam1,
                          distCoeffsMatrix_Cam1,
                          imageSize,
                          rvecCam1toCam0, //rvecCam1, //rvecCam1toCam0,
                          tvecCam1toCam0, //tvecCam1, //tvecCam1toCam0,
                          stereoRectify_R1,
                          stereoRectify_R2,
                          stereoRectify_P1,
                          stereoRectify_P2,
                          stereoRectify_Q);
    }

    //stereo-initUndistortRectifyMap()
    {
        std::cerr << "rvecCam1: " << std::endl;
        std::cerr << rvecCam1 << std::endl;
        std::cerr << "tvecCam1: " << std::endl;
        std::cerr << tvecCam1 << "\n" << std::endl;
        std::cerr << "stereoRectify_R1: " << std::endl;
        std::cerr << stereoRectify_R1 << "\n" << std::endl;
        std::cerr << "stereoRectify_R2: " << std::endl;
        std::cerr << stereoRectify_R2 << "\n" << std::endl;
        std::cerr << "stereoRectify_P1: " << std::endl;
        std::cerr << stereoRectify_P1 << "\n" << std::endl;
        std::cerr << "stereoRectify_P2: " << std::endl;
        std::cerr << stereoRectify_P2 << "\n" << std::endl;
        // Cam0
        cv::initUndistortRectifyMap(intrinsicMatrix_Cam0,
                                    distCoeffsMatrix_Cam0,
                                    stereoRectify_R1,
                                    stereoRectify_P1,
                                    imageSize,
                                    CV_32FC1,
                                    RectifyMap_cam0_map1,
                                    RectifyMap_cam0_map2
                                    );

        // Cam1
        cv::initUndistortRectifyMap(intrinsicMatrix_Cam1,
                                    distCoeffsMatrix_Cam1,
                                    stereoRectify_R2,
                                    stereoRectify_P2,
                                    imageSize,
                                    CV_32FC1,
                                    RectifyMap_cam1_map1,
                                    RectifyMap_cam1_map2
                                    );

    }

    // result copy()
    {
        // _cv_body2Cam0          = cv_body2Cam0.clone();         
        // _cv_body2Cam1          = cv_body2Cam1.clone();
        // _intrinsicMatrix_Cam0  = intrinsicMatrix_Cam0.clone();
        // _intrinsicMatrix_Cam1  = intrinsicMatrix_Cam1.clone();
        // _distCoeffsMatrix_Cam0 = distCoeffsMatrix_Cam0.clone();
        // _distCoeffsMatrix_Cam1 = distCoeffsMatrix_Cam1.clone();
    }

    return 1;
}