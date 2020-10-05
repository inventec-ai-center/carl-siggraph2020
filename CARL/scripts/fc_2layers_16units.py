import tensorflow as tf
import tf_util_extend as TFUtilExtend

NAME = "fc_2layers_16units"

def build_net(input_tf, reuse=False):
    layers = [16, 8]
    activation = tf.nn.leaky_relu
    net_count = 0

    h, net_count = TFUtilExtend.fc_net(input_tf, net_count, layers, activation=activation, reuse=reuse)
    h = activation(h)
    return h