#import "TagFinder.h"
#import "ImageConversion.h"
#import "ImageDisplay.h"
#import "Foundation/Foundation.h"
#ifdef __cplusplus
#import <visp3/visp.h>
#import <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#endif

@interface imageToData ()
@end

@implementation imageToData
@synthesize tags;
@synthesize april;
@synthesize cam;
@synthesize detector;
@synthesize cMo;

//! [initializes imageToData object]
- (instancetype)init {
    if (self = [super init]) {
        _tags = [[NSMutableArray alloc] init];
        vpDetectorAprilTag::vpAprilTagFamily tagFamily = vpDetectorAprilTag::TAG_36h11;
        detector = new vpDetectorAprilTag (tagFamily);
        _cMo = std::vector<vpHomogeneousMatrix>();
    }
    return self;
}

//! [return number of april tags in an image]
- (int)getNumberOfTags {
    return (int)detector->getNbObjects();
}

//! [return specificied april tag detected in an image]
// int index = <the specified april tag to return>
- (AprilTags)getTagAtIndex:(int)index {
    april.number = detector->getTagsId()[index];
    for(unsigned int i=0; i < 4; i++) {
        for(unsigned int j=0; j < 4; j++) {
            april.poseData[i*4+j] = _cMo[index][i][j];
        }
    }
    std::vector< std::vector< vpImagePoint> > imagePoints = detector->getTagsCorners();
    std::map< int, double > idSizeMap = {{-1, 0.12065}};
    std::vector<std::vector<vpPoint>> tagsPoints = detector->getTagsPoints3D(detector->getTagsId(), idSizeMap);
    
    std::vector<cv::Point3f> list_points3d;
    std::vector<cv::Point2f> list_points2d;
    
    for (unsigned int i = 0; i < tagsPoints[index].size(); i++) {
        float X, Y, Z;

        X = tagsPoints[index][i].get_oX();
        Y = tagsPoints[index][i].get_oY();
        Z = tagsPoints[index][i].get_oZ();
        list_points3d.push_back(cv::Point3f(X, Y, Z));
    }
    
    int numCorners = 4;
    for (unsigned int i = 0; i < numCorners; i++) {
        double x = 0, y = 0;
        vpPixelMeterConversion::convertPoint(cam, imagePoints[index][i], x, y);
        april.imagePoints[2*i] = imagePoints[index][i].get_j();
        april.imagePoints[2*i+1] = imagePoints[index][i].get_i();
        list_points2d.push_back(cv::Point2f(x, y));
    }

     // Note: trying PnP
     cv::Mat t_matrix_;
     cv::Mat R_matrix_;
     cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);
     cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
     cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);

     bool useExtrinsicGuess = false;
     // image coordinates are normalized, so no need to specify the camera parameter
     cv::Mat A_matrix_ = cv::Mat::eye(3, 3, CV_64FC1);
     // Pose estimation
    
    // reorder 3d points to conform with cv::SOLVEPNP_IPPE_SQUARE
    list_points3d = {list_points3d[3], list_points3d[2], list_points3d[1], list_points3d[0]};
     bool correspondence = cv::solvePnP( list_points3d, list_points2d, A_matrix_, distCoeffs, rvec, tvec,
                                        useExtrinsicGuess, cv::SOLVEPNP_IPPE_SQUARE);

     // Transforms Rotation Vector to Matrix
     Rodrigues(rvec, R_matrix_);
     t_matrix_ = tvec;
    std::cout << R_matrix_ << std::endl << t_matrix_ << std::endl;
    
    
    // I know this is lame TODO: make it better

    float tx = t_matrix_.at<float64_t>(0,0);
    float ty = t_matrix_.at<float64_t>(1,0);
    float tz = t_matrix_.at<float64_t>(2,0);
    float R00 = R_matrix_.at<float64_t>(0,0);
    float R01 = -R_matrix_.at<float64_t>(0,1);
    float R02 = -R_matrix_.at<float64_t>(0,2);
    float R10 = R_matrix_.at<float64_t>(1,0);
    float R11 = -R_matrix_.at<float64_t>(1,1);
    float R12 = -R_matrix_.at<float64_t>(1,2);
    float R20 = R_matrix_.at<float64_t>(2,0);
    float R21 = -R_matrix_.at<float64_t>(2,1);
    float R22 = -R_matrix_.at<float64_t>(2,2);

    april.poseData[0] = R00;
    april.poseData[1] = R01;
    april.poseData[2] = R02;
    april.poseData[3] = tx;
    april.poseData[4] = R10;
    april.poseData[5] = R11;
    april.poseData[6] = R12;
    april.poseData[7] = ty;
    april.poseData[8] = R20;
    april.poseData[9] = R21;
    april.poseData[10] = R22;
    april.poseData[11] = tz;

    return april;
}

//! [detect april tags]
// UIImage *img = <the image in which to search for april tags>
// float fx = <camera focal length along x axis>
// float fy = <camera focal length along y axis>
// float cx = <principal point offset along x axis>
// float cy = <principal point offset along y axis>
- (void)findTags:(UIImage *)img :(float)fx :(float)fy :(float)cx :(float)cy {
     // Convert image to visp
     vpImage<unsigned char> I = [ImageConversion vpImageGrayFromUIImage:img];
    
     // Detect AprilTag
     vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
    double tagSize = 0.12065; // Size of 8.5x11" printed april tags
     //double tagSize = 0.175; // Size of OccamLab april tags
     float quad_decimate = 1.0;
     int nThreads = 1;
    
     // Set camera parameters
     cam.initPersProjWithoutDistortion(fx, fy, cx, cy); // Gets camera intrinsics from ARFrame
    
     // Initialize apriltag detector
     detector->setAprilTagQuadDecimate(quad_decimate);
     detector->setAprilTagPoseEstimationMethod(poseEstimationMethod);
     detector->setAprilTagNbThreads(nThreads);
     detector->setAprilTagRefineEdges(true);
     _cMo.clear();
     // Detect all the tags in the image
     detector->detect(I, tagSize, cam, _cMo);
    
}

@end
