#!/usr/bin/env python
#// This program is free software: you can redistribute it and/or modify
#// it under the terms of the GNU Lesser General Public License as published by
#// the Free Software Foundation, either version 3 of the License, or
#// at your option) any later version.
#//
#// This program is distributed in the hope that it will be useful,
#// but WITHOUT ANY WARRANTY; without even the implied warranty of
#// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#// GNU Lesser General Public License for more details.
#//
#// You should have received a copy of the GNU Lesser General Public License
#// along with this program.  If not, see <http://www.gnu.org/licenses/>.

from __future__ import with_statement # for python 2.5
__author__ = 'Robert Ellenberg'
__author__ = 'Daniel M. Lofaro'
__license__ = 'GPLv3 license'

import hubo_ach as ha
import ach
import time
#from ctypes import *

from optparse import OptionParser
#import select

#from openravepy import *
from numpy import pi,inf
import numpy as _np
import openhubo

skip = 100
kipi = 0
skiptemp = 0.0


def make_maps(robot):
    ind=openhubo.make_name_to_index_converter(robot)

    ha_from_oh={}
    ha_from_oh.setdefault(ind('RSP'),ha.RSP)
    ha_from_oh.setdefault(ind('RSR'),ha.RSR)
    ha_from_oh.setdefault(ind('RSY'),ha.RSY)
    ha_from_oh.setdefault(ind('REP'),ha.REB)
    ha_from_oh.setdefault(ind('RWY'),ha.RWY)
    ha_from_oh.setdefault(ind('RWP'),ha.RWP)

    ha_from_oh.setdefault(ind('LSP'),ha.LSP)
    ha_from_oh.setdefault(ind('LSR'),ha.LSR)
    ha_from_oh.setdefault(ind('LSY'),ha.LSY)
    ha_from_oh.setdefault(ind('LEP'),ha.LEB)
    ha_from_oh.setdefault(ind('LWY'),ha.LWY)
    ha_from_oh.setdefault(ind('LWP'),ha.LWP)

    ha_from_oh.setdefault(ind('HPY'),ha.WST)

    ha_from_oh.setdefault(ind('RHY'),ha.RHY)
    ha_from_oh.setdefault(ind('RHR'),ha.RHR)
    ha_from_oh.setdefault(ind('RHP'),ha.RHP)
    ha_from_oh.setdefault(ind('RKP'),ha.RKN)
    ha_from_oh.setdefault(ind('RAP'),ha.RAP)
    ha_from_oh.setdefault(ind('RAR'),ha.RAR)

    ha_from_oh.setdefault(ind('LHY'),ha.LHY)
    ha_from_oh.setdefault(ind('LHR'),ha.LHR)
    ha_from_oh.setdefault(ind('LHP'),ha.LHP)
    ha_from_oh.setdefault(ind('LKP'),ha.LKN)
    ha_from_oh.setdefault(ind('LAP'),ha.LAP)
    ha_from_oh.setdefault(ind('LAR'),ha.LAR)

    oh_from_ha = {v:k for k, v in ha_from_oh.items()}
    return (ha_from_oh,oh_from_ha)

class StatusReporter:
    def __init__(self, name=None,waittime=2.0):
        self.name = name
        self.tstart = time.time()
        self.waittime=waittime
        self.count=0

    def update(self):
        self.count+=1
        t=time.time()
        dt=t - self.tstart
        if dt>=self.waittime:
            update_rate=ha.HUBO_LOOP_PERIOD*self.count/dt
            print 'Elapsed: {0} sec, {1:0.2f} percent of realtime'.format(
                dt,update_rate * 100.)
            self.count=0
            self.tstart=t


def sim2state(robot,state):
    """Iterate over all robot DOF and copy the values to the appropriate
    hubo-ach joint."""

    pose=robot.GetDOFValues() # gets the current state

    for k,v in ha_from_oh_map.items():
        state.joint[v].pos = pose[k]

    return pose

def pos2robot(robot, state):
    """Iterate over all hubo-ach joints and copy the "pos" field to a robot
    pose vector"""

    pose=robot.GetDOFValues() # gets the current state
    for k,v in oh_from_ha_map.items():
        pose[v]=state.joint[k].pos
    return pose

def ref2robot(robot, state):
    """Iterate over all refs and copy from the state to a robot pose vector"""

    pose=robot.GetDOFValues() # gets the current state
    for k,v in oh_from_ha_map.items():
        pose[v]=state.joint[k].ref

    return pose

if __name__=='__main__':
    import cProfile, openhubo.startup
    pr = cProfile.Profile()

    parser = OptionParser()
    parser.add_option("-s","--simtime", action="store_true",
                      dest="simetime", default=False,
                      help="Use Sim time instead of realtime")
    (env,options)=openhubo.setup('qtcoin',True,parser)
    env.SetDebugLevel(4)
    time.sleep(.25)
    options.ghost=True

    [robot,ctrl,ind,ghost,recorder]=openhubo.load_scene(env,options)

    if not options.stop:
        env.StartSimulation(openhubo.TIMESTEP)
        time.sleep(.5)

    # Hubo-Ach Start and setup:
    s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
    s.flush()
    state = ha.HUBO_STATE()

    ts = ach.Channel(ha.HUBO_CHAN_VIRTUAL_TO_SIM_NAME)
    ts.flush()
    sim = ha.HUBO_VIRTUAL()

    fs = ach.Channel(ha.HUBO_CHAN_VIRTUAL_FROM_SIM_NAME)
    fs.flush()

    print('Starting Sim')

    fs.put(sim)

    if options.profile:
        steps=1000
        pr.enable()
    else:
        #FIXME: swap to while true?
        steps=1000000

    # sloppy globals
    (ha_from_oh_map,oh_from_ha_map) = make_maps(robot)

    pr.enable()

    reporter=StatusReporter('Virtual Hubo',2.0)

    N = max(_np.ceil(ha.HUBO_LOOP_PERIOD/openhubo.TIMESTEP),1)

    for k in xrange(steps):
        #Use status reporter to print status messages
        reporter.update()

        #Update, waiting for simulation to complete
        [status, framesizes] = ts.get(sim, wait=True, last=False)
        [status, framesizes] = s.get(state, wait=False, last=True)

        ref_pose = ref2robot(robot, state)

        if openhubo.check_physics(env):
            ctrl.SetDesired(ref_pose)   # sends to robot
        else:
            ghost.SetDOFValues(ref_pose)
            state_pose = pos2robot(robot, state)
            ctrl.SetDesired(state_pose)   # Directly copy robot state over

        #Loop at openHubo timestep N times to get a total delta_t of HUBO_LOOP_PERIOD
        for x in xrange(int(N)):
            env.StepSimulation(openhubo.TIMESTEP)

        #Update ach channels with new time and pose data
        sim.time = sim.time + N*openhubo.TIMESTEP
        state_pose = sim2state(robot,state)
        s.put(state)
        fs.put(sim)
        #time.sleep(0.001)  # sleep to allow for keyboard input

    if options.profile:
        pr.disable()
        pr.print_stats('time')
    pr.disable()
    pr.print_stats('time')
