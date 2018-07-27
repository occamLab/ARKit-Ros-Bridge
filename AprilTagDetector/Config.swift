//
//  Config.swift
//  AprilTagDetector
//  Set up the ports to send data.

//
//  Created by Occam Lab on 7/20/18.
//  Copyright © 2018 MyOrganization. All rights reserved.
//

import Foundation

struct Config {
    
    struct Ports {
        static let broadcast = UInt16(35601)
        static let broadcastImages = UInt16(35602)
        static let broadcastAprilTags = UInt16(35603)
    }
    
}
