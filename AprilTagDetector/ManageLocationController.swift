//
//  ManageLocationController.swift
//  AprilTagDetector
//
//  Created by SeungU on 7/1/20.
//  Copyright © 2020 MyOrganization. All rights reserved.
//

import UIKit
import ARKit

protocol writeNodeBackDelegate: class {
    func writeNodeBack(nodes: [LocationData], deleteNodes: [LocationData])
}

class LocationTableViewCell: UITableViewCell {
    @IBOutlet weak var locationImageView: UIImageView!
    @IBOutlet weak var locationTextLabel: UILabel!
}

class ManageLocationController: UITableViewController {
    
    var nodeList: [LocationData] = []
    var deleteNodeList: [LocationData] = []
    
    weak var delegate: writeNodeBackDelegate?
    
    override func viewDidLoad() {
        super.viewDidLoad()
        tableView.rowHeight = 150
        //print(nodeList[0].name!)
    }
    
    override func viewWillDisappear(_ animated: Bool) {
        self.delegate?.writeNodeBack(nodes: nodeList, deleteNodes: deleteNodeList)
    }
    
    override func numberOfSections(in tableView: UITableView) -> Int {
        return 1
    }

    override func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int {
        return nodeList.count
    }

    override func tableView(_ tableView: UITableView, titleForHeaderInSection section: Int) -> String? {
        return "Saved Location List"
    }

    override func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        let cell = tableView.dequeueReusableCell(withIdentifier: "LocationCell", for: indexPath) as! LocationTableViewCell

        cell.locationTextLabel?.text = nodeList[indexPath.row].node.name!
        cell.locationTextLabel?.sizeToFit()
        cell.locationImageView?.image = nodeList[indexPath.row].picture
        //cell.textLabel?.text = "test"
        return cell
    }
    
    override func tableView(_ tableView: UITableView, canEditRowAt indexPath: IndexPath) -> Bool {
        return true
    }
    
    override func tableView(_ tableView: UITableView, commit editingStyle: UITableViewCellEditingStyle, forRowAt indexPath: IndexPath) {
        if editingStyle == .delete {
            deleteNodeList.append(nodeList[indexPath.row])
            nodeList.remove(at: indexPath.row)
            tableView.deleteRows(at: [indexPath], with: .fade)
        }
    }
}

