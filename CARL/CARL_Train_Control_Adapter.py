import os
import sys
import json
import time
import numpy as np
import tensorflow as tf
from tensorflow.python import pywrap_tensorflow

sys.path.append('../DeepMimic/')
sys.path.append('scripts/')

import tf_util_extend as TFUtilExtend
import net_builder as NetBuilder
from util.arg_parser import ArgParser
from util.logger import Logger
import util.mpi_util as MPIUtil
import util.util as Util


class ControlAdapter:
    LEARNING_RATE_KEY = 'LearningRate'
    EPOCHS_KEY = 'Epochs'
    BATCH_SIZE = 'BatchSize'
    GENERATOR_NET_KEY = 'GeneratorNet'
    DISCRIMINATOR_NET_KEY = 'DiscriminatorNet'
    GENERATOR_VALUE_LOSS_FACTOR_KEY = 'GeneratorValueLossFactor'
    DISCRIMINATOR_VALUE_LOSS_FACTOR_KEY = 'DiscriminatorValueLossFactor'
    NORMALIZER_MODEL_FILE_KEY = 'NormalizerModelFile'
    STATES_FILE_KEY = 'StatesFile'
    GOALS_FILE_KEY = 'GoalsFile'
    WEIGHTS_FILE_KEY = 'WeightsFile'
    VALIDATION_RATIO_KEY = 'ValidationRatio'
    OUTPUT_PATH_KEY = 'OutputPath'
    OUTPUT_ITERS_KEY = "OutputIters"
    INT_OUTPUT_PATH_KEY = 'IntOutputPath'
    INT_OUTPUT_ITERS_KEY = 'IntOutputIters'

    def __init__(self, json_file, n_epochs=None):
        self.tf_scope = 'control_adapter'
        self.graph = tf.Graph()
        self.sess = tf.Session(graph=self.graph)
        self.logger = Logger()
        self.output_dir = ''
        self.int_output_dir = ''
        self.iter = 0
        self.wall_time_begin = 0

        assert(json_file != '')
        with open(json_file, 'r') as fin:
             self.json_data = json.loads(fin.read())

        self._parse_params(self.json_data)
        self._load_data(self.json_data)
        self._build_graph(self.json_data)

        if n_epochs: self.epochs = n_epochs

        return

    def __del__(self):
        self.shutdown()
        return

    def shutdown(self):
        Logger.print('Shutting down...')
        self.sess.close()
        return

    def _parse_params(self, json_data):
        assert self.EPOCHS_KEY in json_data
        assert self.BATCH_SIZE in json_data
        assert self.VALIDATION_RATIO_KEY in json_data
        assert self.OUTPUT_PATH_KEY in json_data
        assert self.OUTPUT_ITERS_KEY in json_data
        assert self.INT_OUTPUT_PATH_KEY in json_data
        assert self.INT_OUTPUT_ITERS_KEY in json_data

        self.epochs = json_data[self.EPOCHS_KEY]
        self.batch_size = json_data[self.BATCH_SIZE]
        self.validation_ratio = json_data[self.VALIDATION_RATIO_KEY]
        self.output_dir = json_data[self.OUTPUT_PATH_KEY]
        self.output_iters = json_data[self.OUTPUT_ITERS_KEY]
        self.int_output_dir = json_data[self.INT_OUTPUT_PATH_KEY]
        self.int_output_iters = json_data[self.INT_OUTPUT_ITERS_KEY]
        self.logger.configure_output_file(self.output_dir + "/control_adapter_log.txt")
        return

    def _build_graph(self, json_data):
        with self.sess.as_default(), self.graph.as_default():
            with tf.variable_scope(self.tf_scope):
                self._build_nets(json_data)

                with tf.variable_scope('solvers'):
                    self._build_losses(json_data)
                    self._build_solvers(json_data)

                self._initialize_vars()
                self._build_saver(json_data)
        return

    def _build_net_gating(self, net_name, s_tf, g_tf, w_dim, reuse):
        input_tfs = [s_tf, g_tf]
        h = NetBuilder.build_net(net_name, input_tfs, reuse)
        with tf.variable_scope('output', reuse=reuse):
            w_tf = tf.layers.dense(inputs=h, units=w_dim, activation=tf.nn.sigmoid, kernel_initializer=TFUtilExtend.xavier_initializer, reuse=reuse)
        return w_tf

    def _build_net_discriminator(self, net_name, w_tf, g_dim, reuse):
        h = NetBuilder.build_net(net_name, w_tf, reuse)
        with tf.variable_scope('output_1', reuse=reuse):
            D_tf = tf.layers.dense(inputs=h, units=1, activation=None, kernel_initializer=TFUtilExtend.xavier_initializer, reuse=reuse)
        with tf.variable_scope('output_2', reuse=reuse):
            D_v  = tf.layers.dense(inputs=h, units=g_dim, activation=None, kernel_initializer=TFUtilExtend.xavier_initializer, reuse=reuse)
        return D_tf, h , D_v

    def _build_nets(self, json_data):
        assert self.GENERATOR_NET_KEY in json_data
        assert self.DISCRIMINATOR_NET_KEY in json_data

        generator_net = json_data[self.GENERATOR_NET_KEY]
        discriminator_net = json_data[self.DISCRIMINATOR_NET_KEY]

        s_size = self.states.shape[1]
        g_size = self.goals.shape[1]
        w_size = self.weights.shape[1]

        s_norm_mean, s_norm_std, _, _ = self._build_normalizer(json_data)

        # setup input tensors
        self.s_tf = tf.placeholder(tf.float32, shape=[None, s_size], name="s") # states
        self.g_tf = tf.placeholder(tf.float32, shape=[None, g_size], name="g") # high-level goals
        self.w_real_tf = tf.placeholder(tf.float32, shape=[None, w_size], name="w_real") # weights drawn from real distribution

        with tf.variable_scope('generator'):
            self.s_norm_tf = (self.s_tf - s_norm_mean) / s_norm_std
            self.g_norm_tf = self.g_tf
            self.w_gene_tf = self._build_net_gating(generator_net, self.s_norm_tf, self.g_norm_tf, w_size, reuse=False) # weights drawn from fake distribution

        with tf.variable_scope('discriminator'):
            self.D_real_tf, self.h_real, self.D_real_v = self._build_net_discriminator(discriminator_net, self.w_real_tf, g_size, reuse=False)
            self.D_gene_tf, self.h_gene, self.D_gene_v = self._build_net_discriminator(discriminator_net, self.w_gene_tf, g_size, reuse=True)

        return

    def _initialize_vars(self):
        self.sess.run(tf.global_variables_initializer())
        return

    def _load_data(self, json_data):
        assert self.STATES_FILE_KEY in json_data
        assert self.GOALS_FILE_KEY in json_data
        assert self.WEIGHTS_FILE_KEY in json_data

        states_file = json_data[self.STATES_FILE_KEY]
        goals_file = json_data[self.GOALS_FILE_KEY]
        weights_file = json_data[self.WEIGHTS_FILE_KEY]

        self.states = np.load(states_file)
        self.goals = np.load(goals_file)
        self.weights = np.load(weights_file)
        assert(self.states.shape[0] == self.goals.shape[0] == self.weights.shape[0])

        Logger.print('Loaded training data with %d samples' % (self.states.shape[0]))
        return

    def _build_normalizer(self, json_data):
        assert(self.NORMALIZER_MODEL_FILE_KEY in self.json_data)
        model_file = self.json_data[self.NORMALIZER_MODEL_FILE_KEY]
        reader = pywrap_tensorflow.NewCheckpointReader(model_file)
        s_mean = reader.get_tensor('agent/resource/s_norm/mean')
        s_std  = reader.get_tensor('agent/resource/s_norm/std')
        g_mean = reader.get_tensor('agent/resource/g_norm/mean')
        g_std  = reader.get_tensor('agent/resource/g_norm/std')
        return s_mean, s_std, g_mean, g_std

    def _build_losses(self, json_data):
        assert self.GENERATOR_VALUE_LOSS_FACTOR_KEY in json_data
        assert self.DISCRIMINATOR_VALUE_LOSS_FACTOR_KEY in json_data

        generator_value_loss_factor = json_data[self.GENERATOR_VALUE_LOSS_FACTOR_KEY]
        discriminator_value_loss_factor = json_data[self.DISCRIMINATOR_VALUE_LOSS_FACTOR_KEY]

        # discriminator loss
        self.loss_D_real_tf = tf.reduce_mean(tf.nn.sigmoid_cross_entropy_with_logits(logits=self.D_real_tf, labels=tf.ones_like(self.D_real_tf)))  # Probability of real being real
        self.loss_D_gene_tf = tf.reduce_mean(tf.nn.sigmoid_cross_entropy_with_logits(logits=self.D_gene_tf, labels=tf.zeros_like(self.D_gene_tf))) # Probability of fake being fake
        self.loss_D_val_tf = tf.reduce_mean(tf.keras.losses.MSE(self.g_norm_tf, self.D_real_v))
        self.loss_D_tf = (self.loss_D_real_tf + self.loss_D_gene_tf) / 2 + self.loss_D_val_tf * discriminator_value_loss_factor

        # generator loss
        self.loss_G_l1_tf = tf.reduce_mean(tf.abs(self.w_real_tf - self.w_gene_tf))
        # self.loss_G_val_tf = tf.reduce_mean(tf.keras.losses.MSE(self.g_norm_tf, self.D_gene_v))
        self.loss_G_gan_tf = tf.reduce_mean(tf.nn.sigmoid_cross_entropy_with_logits(logits=self.D_gene_tf, labels=tf.ones_like(self.D_gene_tf)))
        self.loss_G_tf = self.loss_G_gan_tf + self.loss_G_l1_tf * generator_value_loss_factor
        return

    def _build_saver(self, json_data):
        vars = self._get_saver_vars()
        vars_restore = []
        for var in vars:
            vars_restore.append(var)
        self.saver = tf.train.Saver(vars_restore, max_to_keep=0)
        return

    def _get_saver_vars(self):
        with self.sess.as_default(), self.graph.as_default():
            vars = tf.get_collection(tf.GraphKeys.GLOBAL_VARIABLES, scope=self.tf_scope)
            vars = [v for v in vars if '/solver/' not in v.name]
            assert len(vars) > 0
        return vars

    def _build_solvers(self, json_data):
        assert self.LEARNING_RATE_KEY in json_data

        learning_rate = json_data[self.LEARNING_RATE_KEY]

        discriminator_vars = self._tf_vars('discriminator')
        self.discriminator_solver = tf.train.AdamOptimizer(learning_rate).minimize(self.loss_D_tf, var_list=discriminator_vars)

        generator_vars = self._tf_vars('generator')
        self.generator_solver = tf.train.AdamOptimizer(learning_rate).minimize(self.loss_G_tf, var_list=generator_vars)

        Logger.print('-'*80)
        for var in discriminator_vars:
            Logger.print(var)

        Logger.print('-'*80)
        for var in generator_vars:
            Logger.print(var)

        return

    def _tf_vars(self, scope=''):
        with self.sess.as_default(), self.graph.as_default():
            res = tf.get_collection(tf.GraphKeys.TRAINABLE_VARIABLES, scope=self.tf_scope + '/' + scope)
            assert len(res) > 0
        return res

    def train(self):
        num_samples = self.states.shape[0]
        num_valid_samples = np.floor(num_samples * self.validation_ratio).astype("int")
        num_train_samples = num_samples - num_valid_samples
        val_range = np.arange(0, num_samples)
        np.random.shuffle(val_range)
        train_range = val_range[0:num_train_samples]
        valid_range = val_range[num_train_samples:num_samples]

        num_batches = np.ceil(num_train_samples // self.batch_size).astype("int")
        train_states = self.states[train_range]
        train_goals = self.goals[train_range]
        train_weights = self.weights[train_range]

        self.wall_time_begin = time.time()

        for e in range(self.epochs):
            loss_D_list = []
            loss_D_real_list = []
            loss_D_gene_list = []
            loss_D_val_list = []
            loss_G_list = []
            loss_G_l1_list = []
            loss_G_gan_list = []

            for b in range(num_batches):
                start = b * self.batch_size
                end = start + self.batch_size
                batch_states = train_states[start:end]
                batch_goals = train_goals[start:end]
                batch_weights = train_weights[start:end]

                # discriminator step
                feed_dict = {
                    self.s_tf: batch_states,
                    self.g_tf: batch_goals,
                    self.w_real_tf: batch_weights
                }
                _, loss_D, loss_D_gene, loss_D_real, loss_D_val = self.sess.run([self.discriminator_solver, self.loss_D_tf, self.loss_D_gene_tf, self.loss_D_real_tf, self.loss_D_val_tf], \
                                                                                feed_dict=feed_dict)
                loss_D_list.append(loss_D)
                loss_D_gene_list.append(loss_D_gene)
                loss_D_real_list.append(loss_D_real)
                loss_D_val_list.append(loss_D_val)

                # generator step
                feed_dict = {
                    self.s_tf: batch_states,
                    self.g_tf: batch_goals,
                    self.w_real_tf: batch_weights
                }
                _, loss_G, loss_G_l1, loss_G_gan = self.sess.run([self.generator_solver, self.loss_G_tf, self.loss_G_l1_tf, self.loss_G_gan_tf], feed_dict=feed_dict)
                loss_G_list.append(loss_G)
                loss_G_l1_list.append(loss_G_l1)
                loss_G_gan_list.append(loss_G_gan)

            avg_loss_D = np.mean(loss_D_list)
            avg_loss_D_real = np.mean(loss_D_real_list)
            avg_loss_D_gene = np.mean(loss_D_gene_list)
            avg_loss_D_val = np.mean(loss_D_val_list)
            avg_loss_G = np.mean(loss_G_list)
            avg_loss_G_l1 = np.mean(loss_G_l1_list)
            avg_loss_G_gan = np.mean(loss_G_gan_list)

            wall_time = time.time() - self.wall_time_begin
            wall_time /= 60 * 60 # hours

            self.logger.log_tabular('Epoch', e)
            self.logger.log_tabular("Wall_Time (hr)", wall_time)
            self.logger.log_tabular('G/Loss', avg_loss_G)
            self.logger.log_tabular('G/Loss_L1', avg_loss_G_l1)
            self.logger.log_tabular('G/Loss_GAN', avg_loss_G_gan)
            self.logger.log_tabular('D/Loss', avg_loss_D)
            self.logger.log_tabular('D/Loss_Real', avg_loss_D_real)
            self.logger.log_tabular('D/Loss_Gene', avg_loss_D_gene)
            self.logger.log_tabular('D/Loss_Value', avg_loss_D_val)
            self.logger.print_tabular()
            Logger.print('')

            self._update_iter(e)
        return

    def _update_iter(self, iter):
        self.iter = iter

        if (self._enable_output() and self.iter % self.output_iters == 0):
            output_path = self._get_output_path()
            output_dir = os.path.dirname(output_path)
            if not os.path.exists(output_dir):
                os.makedirs(output_dir)
            self.save_model(output_path)

        if (self._enable_int_output() and self.iter % self.int_output_iters == 0):
            int_output_path = self._get_int_output_path()
            int_output_dir = os.path.dirname(int_output_path)
            if not os.path.exists(int_output_dir):
                os.makedirs(int_output_dir)
            self.save_model(int_output_path)

        if (self._enable_output() and self.iter % self.int_output_iters == 0):
            self.logger.dump_tabular()

        return

    def save_model(self, output_path):
        with self.sess.as_default(), self.graph.as_default():
            try:
                save_path = self.saver.save(self.sess, output_path, write_meta_graph=False, write_state=False)
                Logger.print('Model saved to: ' + save_path)
            except:
                Logger.print("Failed to save model to: " + save_path)
        return

    def _enable_output(self):
        return self.output_dir != ""

    def _enable_int_output(self):
        return self.int_output_dir != ""

    def _get_output_path(self):
        assert(self.output_dir != '')
        file_path = self.output_dir + '/model_control_adapter.ckpt'
        return file_path

    def _get_int_output_path(self):
        assert(self.int_output_dir != '')
        file_path = self.int_output_dir + ('/control_adapter_models/control_adapter_int_model_{:010d}.ckpt').format(self.iter)
        return file_path


def main():
    json_file = sys.argv[2]

    try: n_epochs = int(sys.argv[3])
    except: n_epochs = None

    control_adapter = ControlAdapter(json_file, n_epochs)
    control_adapter.train()
    control_adapter.shutdown()
    return


if __name__ == '__main__':
    main()
