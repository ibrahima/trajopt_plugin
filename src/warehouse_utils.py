#!/usr/bin/env python

import pymongo
import gridfs
from moveit_msgs.msg import RobotState

class MoveitWarehouseDatabase(object):
    def __init__(self, database_name, port=33829):
        self.client = pymongo.MongoClient('localhost', port)
        self.db = self.client[database_name]
        self.gridfs = gridfs.GridFS(self.db)

    def collection_for_message(self, klass):
        """
        Returns the MongoDB collection name for the message type klass
        """
        mod = klass.__module__
        module_name = mod[:mod.find('.')]
        message_name = "%s/%s" % (module_name, klass.__name__)
        result = self.db.ros_message_collections.find_one(type=message_name)
        return result["name"]

    def get_message(self, klass, **kwargs):
        """
        Returns a message of type klass specified by the query arguments in kwargs
        """
        collection_name = self.collection_for_message(klass)
        collection = self.db[collection_name]
        entry = collection.find_one(kwargs)
        blob_id = entry['blob_id']
        f = self.gridfs.get(blob_id)
        msg = klass()
        msg.deserialize(f.read())
        return msg
        
if __name__ == "__main__":
    mwdb = MoveitWarehouseDatabase('moveit_robot_states')
    msg = mwdb.get_message(RobotState, state_id="pr2_state_0000")
    print msg
