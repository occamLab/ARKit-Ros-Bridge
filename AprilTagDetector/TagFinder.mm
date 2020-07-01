#import "TagFinder.h"
#import "ImageConversion.h"
#import "ImageDisplay.h"
#import "Foundation/Foundation.h"
#ifdef __cplusplus
#import <visp3/visp.h>
#endif

@interface imageToData ()
@end

@implementation imageToData
@synthesize tags;
@synthesize april;
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
    NSArray *posePoints;
    int tagNumber;

    NSString *message = [NSString stringWithCString:detector->getMessage(index).c_str() encoding:[NSString defaultCStringEncoding]];
    [_tags addObject:message];
    std::stringstream buffer;
    buffer << _cMo[index] << std::endl;
    NSString *poseMatrix = [NSString stringWithCString:buffer.str().c_str() encoding:[NSString defaultCStringEncoding]];
    NSString *updatePose1 = [poseMatrix stringByReplacingOccurrencesOfString:@"  " withString:@","];
    NSString *updatePose2 = [[updatePose1 componentsSeparatedByCharactersInSet:[NSCharacterSet newlineCharacterSet]] componentsJoinedByString:@","];
    [_tags addObject:updatePose2];
    NSArray *msg2 = [message componentsSeparatedByString:@" "];
    tagNumber = [msg2[2] intValue];
    posePoints = [updatePose2 componentsSeparatedByString:@","];
    april.number = tagNumber;
    for(Size j=0; j < 16; j++) {
        april.poseData[j] = [posePoints[j] doubleValue];
    }
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
     double tagSize = 0.168275; // Size of 8.5x11" printed april tags
     //double tagSize = 0.175; // Size of OccamLab april tags

     float quad_decimate = 3.0;
     int nThreads = 1;
    
     // Set camera parameters
     vpCameraParameters cam;
     cam.initPersProjWithoutDistortion(fx, fy, cx, cy); // Gets camera intrinsics from ARFrame
    
     // Initialize apriltag detector
     detector->setAprilTagQuadDecimate(quad_decimate);
     detector->setAprilTagPoseEstimationMethod(poseEstimationMethod);
     detector->setAprilTagNbThreads(nThreads);
     _cMo.clear();
     // Detect all the tags in the image
     detector->detect(I, tagSize, cam, _cMo);
}

@end
