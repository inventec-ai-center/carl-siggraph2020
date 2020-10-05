import tensorflow as tf
import tf_util_extend as TFUtilExtend

NAME = "fc_2layers_512units"

def build_net(input_tfs, reuse=False):
    layers = [512, 256]
    activation = tf.nn.relu

    input_tf = tf.concat(axis=-1, values=input_tfs)          
    h, _ = TFUtilExtend.fc_net(input_tf, 0, layers, activation=activation, reuse=reuse)
    h = activation(h)
    return h