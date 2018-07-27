//
//  TagFinder-Swift.h
//  AprilTagDetector
//
//  Created by djconnolly27 on 7/11/18.
//  Copyright © 2018 OccamLab. All rights reserved.
//

#ifndef TagFinder_Swift_h
#define TagFinder_Swift_h

#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>
#import <stdint.h>

@interface imageToData : NSObject

struct AprilTags{
    int number;
    double poseData[16];
};

@property struct AprilTags april;

- (void)findTags:(UIImage *)img :(float)fx :(float)fy :(float)cx :(float)cy;
- (int)getNumberOfTags;
- (struct AprilTags)getTagAtIndex:(int)index;

@end


#endif /* TagFinder_Swift_h */
