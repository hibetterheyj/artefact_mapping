#!/usr/bin/env python3

import rospy
import math
import time
import numpy
import pandas as pd
import os
from artefact_msgs.msg import Artefact
"""
Header header
geometry_msgs/PointStamped landmark
    Header header
    Point point
        float64 x
        float64 y
        float64 z
uint64 class_label
"""

class ArteListener:
    def __init__(self, export_name):
        # variables
        self.idx = -1
        self.artefact_x = 0
        self.artefact_y = 0
        self.artefact_z = 0
        self.artefact_class = ''

        # folder & files
        self.script_folder = os.path.abspath(os.path.dirname(__file__))
        self.coco_file = self.script_folder + "/coco.names"
        self.output_folder = "/artefact/"
        self.output_folder_path = self.script_folder + self.output_folder
        if not os.path.exists(self.output_folder_path):
            os.mkdir(self.output_folder_path)
        self.bag_name = export_name

        # Create coco class dictionary
        self.class_dictionary = {}
        a_file = open(self.coco_file, 'r')
        for key, line in enumerate(a_file):
            value = line.split()[0]
            self.class_dictionary[key] = value

        # Init node
        rospy.init_node('artefact_listener', anonymous=True)
        rospy.loginfo('Node "artefact_listener" created')

        # Create subscribers
        rospy.loginfo('Create artefact_sub')
        self.artefact_sub = rospy.Subscriber(
            '/W_artefact', Artefact, self.artefact_cb)

    def artefact_cb(self, artefact):
        """subscribe to the /W_artefact topic to get the artefact infomation"""
        self.idx = artefact.header.seq
        self.artefact_x = artefact.landmark.point.x
        self.artefact_y = artefact.landmark.point.y
        self.artefact_z = artefact.landmark.point.z
        self.artefact_class = self.index2class(artefact.class_label)
        rospy.loginfo('{}. Detected {} @ ({}, {}, {})'.format(
            self.idx, self.artefact_class, self.artefact_x, self.artefact_y, self.artefact_z))

    def index2class(self, class_label):
        """
        decode the corresponding class from label
        """
        # backpack, fake person, tv screen, banana, bottle
        return self.class_dictionary[class_label]

    def save_to_csv(self):
        artefact_list = []
        name = ['X', 'Y', 'Z', 'Class']
        loop_rate = rospy.Rate(10) # Hz
        current_idx = -1
        while not rospy.is_shutdown():
            if (self.idx <= current_idx):
                continue
            else:
                current_idx = self.idx
                new_artefact = [self.artefact_x, self.artefact_y,
                    self.artefact_z, self.artefact_class]
                artefact_list.append(new_artefact)
                artefact_pd = pd.DataFrame(columns=name, data=artefact_list)
                artefact_pd.to_csv(self.bag_name + ".csv",
                             encoding='utf-8', index=False)

if __name__ == "__main__":
    # wangen_a_a | zurich_garage | zurich_polystrasse
    export_name = 'final_challenge'  # final_challenge | zurich_garage
    artefact_listener = ArteListener(export_name)
    try:
        artefact_listener.save_to_csv()
    except rospy.ROSInterruptException:
        pass
