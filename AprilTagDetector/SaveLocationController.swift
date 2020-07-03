//
//  SaveLocationController.swift
//  AprilTagDetector
//
//  Created by SeungU on 6/22/20.
//  Copyright © 2020 MyOrganization. All rights reserved.
//

import UIKit
import CoreLocation

// protocol needed to send the data back as dismissing this view
protocol writeValueBackDelegate: class {
    func writeValueBack(value: [String:String])
}

class SaveLocationController: UIViewController, CLLocationManagerDelegate, UITextFieldDelegate {
    
    var locationManager:CLLocationManager!
    @IBOutlet var longitudeLabel: UILabel!
    @IBOutlet var latitudeLabel: UILabel!
    @IBOutlet var locationAddress: UILabel!
    @IBOutlet var userInput: UITextField!
    
    // delegate to send data back
    weak var delegate: writeValueBackDelegate?
    
    // true when retrieved location info for the first time
    var autoLocationSet: Bool = false
    var locationName: String = ""
    
    override func viewDidLoad() {
        super.viewDidLoad()
        //Do any additional setup after loading the view.
        locationManager = CLLocationManager()
        locationManager.delegate = self
        locationManager.requestWhenInUseAuthorization()
        locationManager.desiredAccuracy = kCLLocationAccuracyBest
        locationManager.startUpdatingLocation()
        
        userInput.delegate = self
    }
    
    //
    @IBAction func enterButton(_ sender: Any) {
        var dataDic: [String:String] = [:]
        dataDic["LocationName"] = locationName
        dataDic["locationAddress"] = locationAddress.text
        dataDic["Latitude"] = latitudeLabel.text
        dataDic["Longitude"] = longitudeLabel.text
        
        if dataDic["LocationName"] != "" {
            self.delegate?.writeValueBack(value: dataDic)
            self.navigationController?.popViewController(animated: true)
        } else {
            let alert = UIAlertController(title: "Information Missing", message: "Please check if every required field is complete", preferredStyle: .alert)
            alert.addAction(UIAlertAction(title: "OK", style: .default, handler: nil))
            self.present(alert, animated: true)
        }
    }
    
    // calles getAddress function when pressed
    @IBAction func getLocationButton(_ sender: Any) {
        getAddress()
    }
    
    override func didReceiveMemoryWarning() {
        super.didReceiveMemoryWarning()
    }
    
    // function is called whenever location info changes.
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {
        if let coor = manager.location?.coordinate {
            //print("latitude" + String(coor.latitude) + "/ longitude" + String(coor.longitude))
            longitudeLabel.text = String(coor.longitude)
            latitudeLabel.text = String(coor.latitude)
            longitudeLabel.sizeToFit()
            latitudeLabel.sizeToFit()
        }
    }
    
    // function called when return key is pressed on the keyboard
    func textFieldShouldReturn(_ textField: UITextField) -> Bool {
        if let input = userInput.text {
            locationName = input
            //print(locationName)
        }
        userInput.resignFirstResponder()
        return true
    }
    
    // retrieve address info from current longitude and latitude
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

