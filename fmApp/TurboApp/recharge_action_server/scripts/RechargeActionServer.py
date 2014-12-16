#!/usr/bin/env python
#/****************************************************************************
# FroboMind positionGoalActionServer.py
# Copyright (c) 2011-2013, author Leon Bonde Larsen <leon@bondelarsen.dk>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name FroboMind nor the
#      names of its contributors may be used to endorse or promote products
#      derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#****************************************************************************/
"""
    Recharge server.
"""
import rospy,actionlib
from recharge_action_server.msg import rechargeAction
from dynamic_reconfigure.server import Server

class RechargeActionServer():
    def __init__(self):
        # Init action server
        self._action_name = rospy.get_name()
        self._server = actionlib.SimpleActionServer(self._action_name, rechargeAction, auto_start=False, execute_cb=self.execute)
        
#        self.orig_footprint = "[[-0.25,-0.15],[-0.12,-0.15],[-0.12,-0.2],[0.13,-0.2],[0.13,-0.16],[0.13,-0.16],[0.22,-0.08],[0.22,-0.05],[0.22,0.05],[0.22,0.08],[0.13,0.16],[0.13,0.16],[0.13,0.2],[-0.12,0.2],[-0.12,0.15],[-0.25,0.15]]";
#        self_orig_footprintScale = 0.01;
#        self.l_footprint = "[[-0.25,-0.15],[-0.12,-0.15],[-0.12,-0.2],[1.13,-0.2],[1.13,-0.16],[1.22,-0.16],[1.31,-0.08],[1.31,-0.05],[1.35,-0.05],[1.35,0.05],[1.31,0.05],[1.31,0.08],[1.22,0.16],[1.13,0.16],[1.13,0.2],[-0.12,0.2],[-0.12,0.15],[-0.25,0.15]]";
#        self.g_footprint =" [[-0.25,-0.15],[-0.12,-0.15],[-0.12,-0.2],[0.13,-0.2],[0.13,-0.16],[0.13,-0.16],[0.22,-0.08],[0.22,-0.05],[0.22,0.05],[0.22,0.08],[0.13,0.16],[0.13,0.16],[0.13,0.2],[-0.12,0.2],[-0.12,0.15],[-0.25,0.15]]";
#        self.g_footprintScale = -0.1;

        # Start the action server
        self._server.start()

    def execute(self,goal):
        if(goal==1):
            if rospy.has_param('/move_base/local_costmap/footprint'):
                rospy.set_param('/move_base/local_costmap/footprint', orig_footprint)
            if rospy.has_param('/move_base/global_costmap/footprint'):
                rospy.set_param('/move_base/global_costmap/footprint', orig_footprint)
            if rospy.has_param('/move_base/global_costmap/footprint_padding'):
                rospy.set_param('/move_base/global_costmap/footprint_padding', self_orig_footprintScale)
            if(rospy.get_param('/move_base/local_costmap/footprint')==orig_footprint):
                if(rospy.get_param('/move_base/global_costmap/footprint')==orig_footprint):
                    if(rospy.get_param('/move_base/global_costmap/footprint_padding')==self_orig_footprintScale):
                        self.setSucceeded()
        else:
            if rospy.has_param('/move_base/local_costmap/footprint'):
                rospy.set_param('/move_base/local_costmap/footprint', self.l_footprint)
            if rospy.has_param('/move_base/global_costmap/footprint'):
                rospy.set_param('/move_base/global_costmap/footprint', self.g_footprint)
            if rospy.has_param('/move_base/global_costmap/footprint_padding'):
                rospy.set_param('/move_base/global_costmap/footprint_padding', self.g_footprintScale)
            if(rospy.get_param('/move_base/local_costmap/footprint')==self.l_footprint):
                if(rospy.get_param('/move_base/global_costmap/footprint')==self.g_footprint):
                    if(rospy.get_param('/move_base/global_costmap/footprint_padding')==self.g_footprintScale):
                        self.setSucceeded()

            
if __name__ == '__main__':
    rospy.init_node('rechargeAction')
    try:
        action_server = RechargeActionServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass        
