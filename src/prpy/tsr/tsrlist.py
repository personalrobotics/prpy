#!/usr/bin/env python
import logging, numpy, os, yaml
from prpy.tsr.tsr import TSR, TSRChain
from rospkg import RosPack

class TSRList:

    def __init__(self):
        # TODO: Read this from param server instead of file. 
        rp = RosPack()
        tsr_yaml_file = os.path.join(rp.get_path('prpy'), 'config', 'tsrlist.yaml')
        with open(tsr_yaml_file, 'r') as f:
            tsr_data = yaml.load(f)

        self.tsrlist = {}
        for ctsr_name in tsr_data:
            # grab the yaml for this tsr
            ctsr_yaml = tsr_data[ctsr_name]

            # metadata about the chain
            sample_start = False
            sample_goal = False
            constrain = False
            if 'sample_start' in ctsr_yaml:
                sample_start = ctsr_yaml['sample_start']
            if 'sample_goal' in ctsr_yaml:
                sample_goal = ctsr_yaml['sample_goal']
            if 'constrain' in ctsr_yaml:
                constrain = ctsr_yaml['constrain']

            # Validate
            if not sample_start and not sample_goal and not constrain:
                logging.error('Invalid TSR chain specified for %s. Must specify either sample_goal, sample_start or constrain as True. Skipping.' % ctsr_name)
                continue

            # Now read the TSRs in the chain
            if not 'TSRs' in ctsr_yaml:
                logging.error('No TSRs specified for %s. Skipping.' % ctsr_name)
                continue

            tsrs = []
            valid = True
            for idx in range(len(ctsr_yaml['TSRs'])):
                tsr_yaml = ctsr_yaml['TSRs'][idx]
                if 'Tw_e' not in tsr_yaml or 'Bw' not in tsr_yaml:
                    logging.error('Invalid TSR specified at index %d. Must specify Tw_e and Bw matrix. Skipping TSR chain.' % idx)
                    valid = False
                    break

                tsr = TSR(Tw_e = numpy.array(tsr_yaml['Tw_e']),
                          Bw = numpy.array(tsr_yaml['Bw']))
                tsrs.append(tsr)

            if not valid:
                continue

            # Finally, add the chain
            tsrchain = TSRChain(sample_start = ctsr_yaml['sample_start'],
                                sample_goal = ctsr_yaml['sample_goal'],
                                TSRs = tsrs)
            self.tsrlist[ctsr_name] = tsrchain

    def GetTSRChain(self, object_name, manip, T0_w=numpy.eye(4)):
        
        # Get the index of the manipulator
        with manip.GetRobot():
            manip.SetActive()
            manip_idx = manip.GetRobot().GetActiveManipulatorIndex()

        tsrchain = None
        # Read the tsr from the dictionary
        if object_name in self.tsrlist:
            tsrchain = self.tsrlist[object_name]
            for tsr in tsrchain.TSRs:
                tsr.manipindex = manip_idx
                tsr.T0_w = T0_w

        return tsrchain
