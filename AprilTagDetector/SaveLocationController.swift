//
//  SaveLocationController.swift
//  AprilTagDetector
//
//  Created by SeungU on 6/22/20.
//  Copyright © 2020 MyOrganization. All rights reserved.
//

import UIKit
import CoreLocation

class SaveLocationController: UIViewController, CLLocationManagerDelegate {
    
    var locationManager:CLLocationManager!
    @IBOutlet var longitudeLabel: UILabel!
    @IBOutlet var latitudeLabel: UILabel!
    @IBOutlet var locationAddress: UILabel!
    
    var autoLocationSet: Bool = false
    
    override func viewDidLoad() {
        super.viewDidLoad()
        //Do any additional setup after loading the view.
        locationManager = CLLocationManager()
        locationManager.delegate = self
        locationManager.requestWhenInUseAuthorization()
        locationManager.desiredAccuracy = kCLLocationAccuracyBest
        locationManager.startUpdatingLocation()
    }
    
    @IBAction func enterButton(_ sender: Any) {
        self.navigationController?.popViewController(animated: true)
    }
    @IBAction func getLocationButton(_ sender: Any) {
        getAddress()
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
    }
    
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        if let coor = manager.location?.coordinate {
            print("latitude" + String(coor.latitude) + "/ longitude" + String(coor.longitude))
            longitudeLabel.text = String(coor.longitude)
            latitudeLabel.text = String(coor.latitude)
            longitudeLabel.sizeToFit()
            latitudeLabel.sizeToFit()
        }
    }
    
    func getAddress() {
        if let location = locationManager.location {
            let geocoder = CLGeocoder()
            
            geocoder.reverseGeocodeLocation(location, completionHandler: {(placemarks, error) -> Void in
                guard error == nil else {
                    print("Reverse Geocode failed")
                    return
                }
                var placeMark: CLPlacemark!
                placeMark = placemarks?[0]
                
                var text: String?
                
                if let country = placeMark.country {
                    text = country
                    if let locality = placeMark.locality {
                        text = text! + "\n" + locality
                    }
                    if let throughfare = placeMark.thoroughfare {
                        text = text! + "\n" + throughfare
                    }
                    if let subThroughfare = placeMark.subThoroughfare {
                        text = text! + "\n" + subThroughfare
                    }
                    self.locationAddress.text = text
                }
            })
        }
    }
}

