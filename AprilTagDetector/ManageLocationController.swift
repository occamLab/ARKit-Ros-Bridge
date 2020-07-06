//
//  ManageLocationController.swift
//  AprilTagDetector
//
//  Created by SeungU on 7/1/20.
//  Copyright © 2020 MyOrganization. All rights reserved.
//

import UIKit
import ARKit

//protocol to send data back
protocol writeNodeBackDelegate: class {
    func writeNodeBack(nodes: [LocationData], deleteNodes: [LocationData])
}

//class for custom cell design with a imageView and a Label
class LocationTableViewCell: UITableViewCell {
    @IBOutlet weak var locationImageView: UIImageView!
    @IBOutlet weak var locationTextLabel: UILabel!
}

class ManageLocationController: UITableViewController {
    
    //List of locationData received from the main view
    var nodeList: [LocationData] = []
    //List of locationData that will be deleted after returning
    var deleteNodeList: [LocationData] = []
    
    //delegate used to send the data back
    weak var delegate: writeNodeBackDelegate?
    
    override func viewDidLoad() {
        super.viewDidLoad()
        tableView.rowHeight = 150
        //print(nodeList[0].name!)
    }
    
    //call the writeNodeBack function when dismissing the view.
    override func viewWillDisappear(_ animated: Bool) {
        self.delegate?.writeNodeBack(nodes: nodeList, deleteNodes: deleteNodeList)
    }
    
    //number of sections in the tableView
    override func numberOfSections(in tableView: UITableView) -> Int {
        return 1
    }
    
    //number of cells
    override func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int {
        return nodeList.count
    }
    
    //section header
    override func tableView(_ tableView: UITableView, titleForHeaderInSection section: Int) -> String? {
        return "Saved Location List"
    }
    
    //set how each row will create cell with the snapshot and name of the locationData
    override func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        let cell = tableView.dequeueReusableCell(withIdentifier: "LocationCell", for: indexPath) as! LocationTableViewCell

        cell.locationTextLabel?.text = nodeList[indexPath.row].node.name!
        cell.locationTextLabel?.sizeToFit()
        cell.locationImageView?.image = nodeList[indexPath.row].picture
        //cell.textLabel?.text = "test"
        return cell
    }
    
    //enable editting the cell
    override func tableView(_ tableView: UITableView, canEditRowAt indexPath: IndexPath) -> Bool {
        return true
    }
    
    //enable deleting the cell by swiping
    override func tableView(_ tableView: UITableView, commit editingStyle: UITableViewCellEditingStyle, forRowAt indexPath: IndexPath) {
        if editingStyle == .delete {
            deleteNodeList.append(nodeList[indexPath.row])
            nodeList.remove(at: indexPath.row)
            tableView.deleteRows(at: [indexPath], with: .fade)
        }
    }
}

