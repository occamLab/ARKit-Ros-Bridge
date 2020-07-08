//
//  TagFinder.h
//  AprilTagDetector
//
//  Created by djconnolly27 on 7/11/18.
//  Copyright © 2018 OccamLab. All rights reserved.
//

#ifndef TagFinder_h
#define TagFinder_h
#import <UIKit/UIKit.h>
#import <Foundation/Foundation.h>
#import <vector>
#ifdef __cplusplus
#import <visp3/visp.h>
#endif

@interface imageToData : NSObject {
    NSMutableArray *_tags;
    std::vector<vpHomogeneousMatrix> _cMo;
}

struct AprilTags{
    int number;
    double poseData[16];
    double imagePoints[8];
};


@property (nonatomic, strong) NSMutableArray *tags;
@property struct AprilTags april;
@property struct vpCameraParameters cam;
@property (atomic, readonly) vpDetectorAprilTag* detector;
@property (atomic, readonly) std::vector<vpHomogeneousMatrix> cMo;

- (void)findTags:(UIImage *)img :(float)fx :(float)fy :(float)cx :(float)cy;
- (int)getNumberOfTags;
- (AprilTags)getTagAtIndex:(int)index;


@end


#endif /* TagFinder_h */
