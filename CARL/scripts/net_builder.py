import learning.nets.fc_2layers_1024units as fc_2layers_1024units
import fc_2layers_512units as fc_2layers_512units
import fc_3layers_512units_branch_inputs as fc_3layers_512units_branch_inputs
import fc_2layers_16units as fc_2layers_16units

def build_net(net_name, input_tfs, reuse=False):
    net = None

    if (net_name == fc_2layers_1024units.NAME):
        net = fc_2layers_1024units.build_net(input_tfs, reuse)
    elif (net_name == fc_3layers_512units_branch_inputs.NAME):
        net = fc_3layers_512units_branch_inputs.build_net(input_tfs, reuse)
    elif (net_name == fc_2layers_512units.NAME):
        net = fc_2layers_512units.build_net(input_tfs, reuse)
    elif (net_name == fc_2layers_16units.NAME):
        net = fc_2layers_16units.build_net(input_tfs, reuse)
    else:
        assert False, 'Unsupported net: ' + net_name
    
    return net
