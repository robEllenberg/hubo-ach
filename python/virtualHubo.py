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
    def __init__(self, env,skip=10):
        self.env=env
        self.tinit = time.time()
        self.tstart = self.tinit
        self.tsim_start=env.GetSimulationTime()
        self.skip=skip
        self.count=0

    def update(self):
        self.count+=1
        if self.count>=self.skip:
            t=time.time()
            dt=t - self.tstart
            self.tstart=t
            dt_sim=(self.env.GetSimulationTime()-self.tsim_start)/1000000.0
            self.tsim_start=self.env.GetSimulationTime()

            update_rate=dt_sim/dt
            print 't: {0:0.3f} sec, dt_sim: {1:0.3f} sec, {2:0.2f}% of realtime'.format(t-self.tinit,
                dt_sim,update_rate * 100.)
            self.count=0


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

def init_message(env,options):
    msg=['Starting simulation']

    if options.simtime:
        msg.append('Sim-timed mode')
    else:
        msg.append('WARNING: Realtime mode')

    if openhubo.check_physics(env):
        msg.append('Physics enabled')
    else:
        msg.append('Physics disabled')

    print ', '.join(msg)

def step_simulation(env,dt):
    """Trick to get yappi to profile external functions."""
    env.StepSimulation(dt)
    #if not openhubo.check_physics(env):
        #time.sleep(dt)


if __name__=='__main__':
    #Import a few libraries that are helpful for profiling
    import yappi, openhubo.startup

    #Parse command line options, including physics and simtime flags.
    parser = OptionParser()
    parser.set_defaults(simtime=True)
    parser.add_option("-s","--realtime", action="store_false",
                      dest="simtime",
                      help="Run simulation in realtime mode (not recommended unless you have a FAST machine)")
    (env,options)=openhubo.setup('qtcoin',True,parser)

    #Force enable the ghost robot to show ref vs. pos
    env.SetDebugLevel(3)
    options.ghost=True

    #Profiling stuff to see python performance
    if options.profile:
        steps=1000
        yappi.start()
    else:
        steps=inf

    [robot,ctrl,ind,ghost,recorder]=openhubo.load_scene(env,options)

    # Hubo-Ach Start and setup:
    s = ach.Channel(ha.HUBO_CHAN_STATE_NAME)
    s.flush()
    state = ha.HUBO_STATE()

    ts = ach.Channel(ha.HUBO_CHAN_VIRTUAL_TO_SIM_NAME)
    ts.flush()
    sim = ha.HUBO_VIRTUAL()

    fs = ach.Channel(ha.HUBO_CHAN_VIRTUAL_FROM_SIM_NAME)
    fs.flush()

    init_message(env,options)

    # Make global mapping functions between hubo-ach (ha) and openhubo (oh)
    (ha_from_oh_map,oh_from_ha_map) = make_maps(robot)

    #Initialization
    k=0
    t=time.time()
    #Define a "reporter" that prints status messages every X updates
    reporter=StatusReporter(env,50)

    if options.simtime:
        fs.put(sim)
        while k<steps:
            [status, framesizes] = ts.get(sim, wait=True, last=False)
            [status, framesizes] = s.get(state, wait=False, last=True)
            #Update, waiting for simulation to complete

            ref_pose = ref2robot(robot, state)

            if openhubo.check_physics(env):
                #Ghost is updated internally
                ctrl.SetDesired(ref_pose)   # sends to robot
            else:
                #No physics, ghost is updated explicitly
                ghost.SetDOFValues(ref_pose)
                state_pose = pos2robot(robot, state)
                ctrl.SetDesired(state_pose)   # Directly copy state (idealcontroller)

            #Loop at openHubo timestep N times to get a total delta_t of HUBO_LOOP_PERIOD
            N = max(_np.ceil(ha.HUBO_LOOP_PERIOD/openhubo.TIMESTEP),1)
            for x in range(int(N)):
                #Profiling trick, below is equivalent to env.StartSimulation(...)
                step_simulation(env,openhubo.TIMESTEP)

            #Update ach channels with new time and pose data
            state_pose = sim2state(robot,state)
            s.put(state)
            sim.time += N*openhubo.TIMESTEP
            fs.put(sim)

            #Update misc stuff
            reporter.update()
            k+=1
    else:
        #WARNING: "Realtime" mode is only as realtime as the code below, that is to say, not very!
        env.StartSimulation(openhubo.TIMESTEP,True)
        while k<steps:
            k+=1
            reporter.update()

            [status, framesizes] = s.get(state, wait=False, last=False)

            ref_pose = ref2robot(robot, state)

            if openhubo.check_physics(env):
                #Ghost is updated internally
                ctrl.SetDesired(ref_pose)   # sends to robot
            else:
                #No physics, ghost is updated explicitly
                ghost.SetDOFValues(ref_pose)
                state_pose = pos2robot(robot, state)
                ctrl.SetDesired(state_pose)   # Directly copy state (idealcontroller)

            state_pose = sim2state(robot,state)
            s.put(state)

            #Pause loop to wait for next
            t1=time.time()
            pause_time=max(ha.HUBO_LOOP_PERIOD-(t1-t),0)
            time.sleep(pause_time)
            t=t1

    if options.profile:
        yappi.print_stats()
