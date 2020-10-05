import tensorflow as tf
import tf_util_extend as TFUtilExtend

NAME = "fc_3layers_512units_branch_inputs"

def build_net(input_tfs, reuse=False):
    layers = [512, 256]
    combined_layer = 256
    activation = tf.nn.relu
    net_count = 0
    
    h_list = []
    for i in range(0, len(input_tfs)):
        if input_tfs[i].shape[1] > 0:
            h, net_count = TFUtilExtend.fc_net(input_tfs[i], net_count, layers, activation=activation, reuse=reuse)
            h = activation(h)
            h_list.append(h)
        else:
            assert False
    
    h = tf.concat(axis=-1, values=h_list)
    h, _ = TFUtilExtend.fc_net(h, net_count, [combined_layer], activation=activation, reuse=reuse)
    h = activation(h)
    return h