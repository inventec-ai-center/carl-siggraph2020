import numpy as np
import agent_builder as AgentBuilder
import learning.tf_util as TFUtil
from learning.rl_agent import RLAgent
from util.logger import Logger
import learning.rl_world as DeepMimicRLWorld


class RLWorld(DeepMimicRLWorld.RLWorld):

    def __init__(self, env, arg_parser):
        super().__init__(env, arg_parser)
        return

    def build_agents(self):
        num_agents = self.env.get_num_agents()
        self.agents = []

        Logger.print('')
        Logger.print('Num Agents: {:d}'.format(num_agents))

        agent_files = self.arg_parser.parse_strings('agent_files')
        assert(len(agent_files) == num_agents or len(agent_files) == 0)

        model_files = self.arg_parser.parse_strings('model_files')
        assert(len(model_files) == num_agents or len(model_files) == 0)

        control_adapter_model_file = self.arg_parser.parse_string('control_adapter_model_file')

        output_path = self.arg_parser.parse_string('output_path')
        int_output_path = self.arg_parser.parse_string('int_output_path')

        global_rand_seed = self.arg_parser.parse_int('rand_seed')

        for i in range(num_agents):
            curr_file = agent_files[i]
            curr_agent = self._build_agent(i, curr_file, global_rand_seed)

            if curr_agent is not None:
                curr_agent.output_dir = output_path
                curr_agent.int_output_dir = int_output_path
                Logger.print(str(curr_agent))

                if (len(model_files) > 0):
                    curr_model_file = model_files[i]
                    if curr_model_file != 'none':
                        curr_agent.load_model(policy_model_path=curr_model_file,
                                              control_adapter_model_path=control_adapter_model_file)

            self.agents.append(curr_agent)
            Logger.print('')

        self.set_enable_training(self.enable_training)
        return

    def _build_agent(self, id, agent_file, seed):
        Logger.print('Agent {:d}: {}'.format(id, agent_file))
        if (agent_file == 'none'):
            agent = None
        else:
            agent = AgentBuilder.build_agent(self, id, agent_file, seed)
            assert (agent != None), 'Failed to build agent {:d} from: {}'.format(id, agent_file)

        return agent


    def isDone(self):
        isDone = False
        for agent in self.agents:
            if (agent is not None):
                isDone |= agent.isDone()
        return isDone
