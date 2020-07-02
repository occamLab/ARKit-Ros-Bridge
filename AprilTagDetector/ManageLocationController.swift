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
    func writeNodeBack(nodes: [SCNNode], deleteNodes: [SCNNode])
}

class ManageLocationController: UITableViewController {
    
    var nodeList: [SCNNode] = []
    var deleteNodeList: [SCNNode] = []
    
    weak var delegate: writeNodeBackDelegate?
    
    override func viewDidLoad() {
        super.viewDidLoad() 
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
        let cell = tableView.dequeueReusableCell(withIdentifier: "CountryCell", for: indexPath)

        cell.textLabel?.text = nodeList[indexPath.row].name!
        
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

