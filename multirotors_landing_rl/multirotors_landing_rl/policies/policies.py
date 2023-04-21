import numpy as np
import tensorflow as tf

from stable_baselines.common.tf_util import batch_to_seq, seq_to_batch
from stable_baselines.common.tf_layers import conv, linear, conv_to_fc, lstm
from stable_baselines.common.policies import ActorCriticPolicy, \
    RecurrentActorCriticPolicy, mlp_extractor


def compressed_nature_cnn(scaled_images, **kwargs):
    """
    CNN from Nature paper (compressed version)

    :param scaled_images: (TensorFlow Tensor) Image input placeholder
    :param kwargs: (dict) Extra keywords parameters for the convolutional layers of the CNN
    :return: (TensorFlow Tensor) The CNN output layer
    """
    activ = tf.nn.relu
    layer_1 = activ(
        conv(scaled_images, 'c1', n_filters=8, filter_size=8, stride=4,
             init_scale=np.sqrt(2), **kwargs))
    layer_2 = activ(conv(layer_1, 'c2', n_filters=16, filter_size=4, stride=2,
                         init_scale=np.sqrt(2), **kwargs))
    layer_3 = activ(conv(layer_2, 'c3', n_filters=16, filter_size=3, stride=1,
                         init_scale=np.sqrt(2), **kwargs))
    layer_3 = conv_to_fc(layer_3)
    return activ(linear(layer_3, 'fc1', n_hidden=128, init_scale=np.sqrt(2)))


def double_cnn_observation(obs, activ, image_height, image_width, **kwargs):
    # Get the features for depth
    depth_image = tf.reshape(
        obs[:, :, :, 0], [tf.shape(obs)[0], image_height, image_width, 1]
    )

    layer_1_depth = activ(
        conv(depth_image, 'c1_0', n_filters=8, filter_size=8, stride=4,
             init_scale=np.sqrt(2), **kwargs))
    layer_2_depth = activ(
        conv(layer_1_depth, 'c2_0', n_filters=16, filter_size=4, stride=2,
             init_scale=np.sqrt(2), **kwargs))
    layer_3_depth = activ(
        conv(layer_2_depth, 'c3_0', n_filters=16, filter_size=3, stride=1,
             init_scale=np.sqrt(2), **kwargs))
    layer_3_depth = conv_to_fc(layer_3_depth)
    depth_features = tf.layers.flatten(activ(
        linear(layer_3_depth, 'fc1_0', n_hidden=128, init_scale=np.sqrt(2))))

    # Get the features for semantics
    semantic_image = tf.reshape(
        obs[:, :, :, 1], [tf.shape(obs)[0], image_height, image_width, 1]
    )

    layer_1_semantic = activ(
        conv(semantic_image, 'c1_1', n_filters=8, filter_size=8, stride=4,
             init_scale=np.sqrt(2), **kwargs))
    layer_2_semantic = activ(
        conv(layer_1_semantic, 'c2_1', n_filters=16, filter_size=4, stride=2,
             init_scale=np.sqrt(2), **kwargs))
    layer_3_semantic = activ(
        conv(layer_2_semantic, 'c3_1', n_filters=16, filter_size=3, stride=1,
             init_scale=np.sqrt(2), **kwargs))
    layer_3_semantic = conv_to_fc(layer_3_semantic)
    semantic_features = tf.layers.flatten(activ(
        linear(layer_3_semantic, 'fc1_1', n_hidden=128, init_scale=np.sqrt(2))))
    #
    return depth_features, semantic_features


def confidence_aware_cnn_observation(obs, activ, image_height, image_width, **kwargs):
    # Get the features for depth
    depth_and_conf_image = tf.reshape(
        obs[:, :, :, :2], [tf.shape(obs)[0], image_height, image_width, 2]
    )

    layer_1_depth = activ(
        conv(depth_and_conf_image, 'c1_0', n_filters=8, filter_size=8, stride=4,
             init_scale=np.sqrt(2), **kwargs))
    layer_2_depth = activ(
        conv(layer_1_depth, 'c2_0', n_filters=16, filter_size=4, stride=2,
             init_scale=np.sqrt(2), **kwargs))
    layer_3_depth = activ(
        conv(layer_2_depth, 'c3_0', n_filters=16, filter_size=3, stride=1,
             init_scale=np.sqrt(2), **kwargs))
    layer_3_depth = conv_to_fc(layer_3_depth)
    depth_features = tf.layers.flatten(activ(
        linear(layer_3_depth, 'fc1_0', n_hidden=128, init_scale=np.sqrt(2))))

    # Get the features for semantics
    semantic_image = tf.reshape(
        obs[:, :, :, 2], [tf.shape(obs)[0], image_height, image_width, 1]
    )

    layer_1_semantic = activ(
        conv(semantic_image, 'c1_1', n_filters=8, filter_size=8, stride=4,
             init_scale=np.sqrt(2), **kwargs))
    layer_2_semantic = activ(
        conv(layer_1_semantic, 'c2_1', n_filters=16, filter_size=4, stride=2,
             init_scale=np.sqrt(2), **kwargs))
    layer_3_semantic = activ(
        conv(layer_2_semantic, 'c3_1', n_filters=16, filter_size=3, stride=1,
             init_scale=np.sqrt(2), **kwargs))
    layer_3_semantic = conv_to_fc(layer_3_semantic)
    semantic_features = tf.layers.flatten(activ(
        linear(layer_3_semantic, 'fc1_1', n_hidden=128, init_scale=np.sqrt(2))))
    #
    return depth_features, semantic_features


class CnnAsymmetricalPolicy(ActorCriticPolicy):
    """
    Custom MLP policy of three layers of size 128 each for the actor and
     2 layers of 64 for the critic, with a nature_cnn feature extractor
    """

    def __init__(self, sess, ob_space, ac_space, n_env, n_steps, n_batch,
                 reuse=False, **kwargs):
        super(CnnAsymmetricalPolicy, self).__init__(sess, ob_space, ac_space,
                                                    n_env, n_steps, n_batch,
                                                    reuse=reuse, scale=True)
        # Read parameters (kind of a hack...)
        try:
            use_dropout = kwargs["use_dropout"]
            del (kwargs["use_dropout"])

            dropout_rate = kwargs["dropout_rate"]
            del (kwargs["dropout_rate"])
        except KeyError:
            use_dropout = False

        with tf.variable_scope("model", reuse=reuse):
            # First, pass through a CNN feature extractor
            activ = tf.nn.relu
            extracted_features = compressed_nature_cnn(self.processed_obs,
                                                       **kwargs)
            extracted_features = tf.layers.flatten(extracted_features)
            if use_dropout:
                extracted_features = tf.nn.dropout(extracted_features, rate=dropout_rate)

            pi_h = extracted_features
            for i, layer_size in enumerate([64, 64, 64]):
                pi_h = activ(
                    tf.layers.dense(pi_h, layer_size, name='pi_fc' + str(i)))
            pi_latent = pi_h

            vf_h = extracted_features
            for i, layer_size in enumerate([32, 32]):
                vf_h = activ(
                    tf.layers.dense(vf_h, layer_size, name='vf_fc' + str(i)))
            self._value_fn = tf.layers.dense(vf_h, 1, name='vf')
            vf_latent = vf_h

            self._proba_distribution, self._policy, self.q_value = \
                self.pdtype.proba_distribution_from_latent(pi_latent, vf_latent,
                                                           init_scale=0.01)

        self._setup_init()

    def step(self, obs, state=None, mask=None, deterministic=False):
        if deterministic:
            action, value, neglogp = self.sess.run(
                [self.deterministic_action, self.value_flat, self.neglogp],
                {self.obs_ph: obs})
        else:
            action, value, neglogp = self.sess.run(
                [self.action, self.value_flat, self.neglogp],
                {self.obs_ph: obs})
        return action, value, self.initial_state, neglogp

    def proba_step(self, obs, state=None, mask=None):
        return self.sess.run(self.policy_proba, {self.obs_ph: obs})

    def value(self, obs, state=None, mask=None):
        return self.sess.run(self.value_flat, {self.obs_ph: obs})


class LstmCnnAsymmetricalPolicy(RecurrentActorCriticPolicy):
    """
    Custom MLP policy of three layers of size 128 each for the actor and
     2 layers of 64 for the critic, with a nature_cnn feature extractor and
     a LSTM layer in between
    """

    recurrent = True

    def __init__(self, sess, ob_space, ac_space, n_env, n_steps, n_batch,
                 n_lstm=256, layer_norm=False, reuse=False, **kwargs):
        super(LstmCnnAsymmetricalPolicy, self).__init__(sess, ob_space,
                                                        ac_space, n_env,
                                                        n_steps, n_batch,
                                                        state_shape=(
                                                            2 * n_lstm,),
                                                        reuse=reuse, scale=True)
        # Read parameters (kind of a hack...)
        use_dropout = kwargs["use_dropout"]
        del (kwargs["use_dropout"])

        dropout_rate = kwargs["dropout_rate"]
        del (kwargs["dropout_rate"])

        with tf.variable_scope("model", reuse=reuse):
            # First, pass through a CNN feature extractor
            activ = tf.nn.relu
            extracted_features = compressed_nature_cnn(self.processed_obs,
                                                       **kwargs)
            if use_dropout:
                extracted_features = tf.nn.dropout(extracted_features, rate=dropout_rate)

            # Then go through a LSTM NN
            input_sequence = batch_to_seq(extracted_features, self.n_env,
                                          n_steps)
            masks = batch_to_seq(self.dones_ph, self.n_env, n_steps)
            rnn_output, self.snew = lstm(input_sequence, masks, self.states_ph,
                                         'lstm1', n_hidden=n_lstm,
                                         layer_norm=layer_norm)
            rnn_features = tf.layers.flatten(seq_to_batch(rnn_output))

            pi_h = rnn_features
            for i, layer_size in enumerate([64, 64, 64]):
                pi_h = activ(
                    tf.layers.dense(pi_h, layer_size, name='pi_fc' + str(i)))
            pi_latent = pi_h

            vf_h = rnn_features
            for i, layer_size in enumerate([32, 32]):
                vf_h = activ(
                    tf.layers.dense(vf_h, layer_size, name='vf_fc' + str(i)))
            self._value_fn = tf.layers.dense(vf_h, 1, name='vf')
            vf_latent = vf_h

            self._proba_distribution, self._policy, self.q_value = \
                self.pdtype.proba_distribution_from_latent(pi_latent, vf_latent,
                                                           init_scale=0.01)

        self._setup_init()

    def step(self, obs, state=None, mask=None, deterministic=False):
        if deterministic:
            return self.sess.run(
                [self.deterministic_action, self.value_flat, self.snew,
                 self.neglogp],
                {self.obs_ph: obs, self.states_ph: state, self.dones_ph: mask})
        else:
            return self.sess.run(
                [self.action, self.value_flat, self.snew, self.neglogp],
                {self.obs_ph: obs, self.states_ph: state, self.dones_ph: mask})

    def proba_step(self, obs, state=None, mask=None):
        return self.sess.run(self.policy_proba,
                             {self.obs_ph: obs, self.states_ph: state,
                              self.dones_ph: mask})

    def value(self, obs, state=None, mask=None):
        return self.sess.run(self.value_flat,
                             {self.obs_ph: obs, self.states_ph: state,
                              self.dones_ph: mask})


class DepthAndStatePolicy(ActorCriticPolicy):
    """
    Custom policy taking as input both a depth image and a state vector (
    position and goal). It passes the depth image throught a cnn feature
    extractor and the state vector through MLP. Then, the outputs are
    concatenated and passed through an action and a value networks.
    """

    def __init__(self, sess, ob_space, ac_space, n_env, n_steps, n_batch,
                 reuse=False, **kwargs):
        super(DepthAndStatePolicy, self).__init__(sess, ob_space, ac_space,
                                                  n_env, n_steps, n_batch,
                                                  reuse=reuse, scale=True)

        # Get size information about observation vector/matrix
        image_width = kwargs["image_width"]
        del (kwargs["image_width"])

        image_height = kwargs["image_height"]
        del (kwargs["image_height"])

        image_channels = kwargs["image_channels"]
        del (kwargs["image_channels"])

        state_size = kwargs["state_size"]
        del (kwargs["state_size"])

        use_dropout = kwargs["use_dropout"]
        del (kwargs["use_dropout"])

        dropout_rate = kwargs["dropout_rate"]
        del (kwargs["dropout_rate"])

        # Actual policy
        with tf.variable_scope("model", reuse=reuse):
            # Get the observation vector and divide it into image and state vec
            depth_image = tf.reshape(
                self.processed_obs[:, :image_width * image_height * image_channels],
                [tf.shape(self.processed_obs)[0], image_height, image_width, image_channels]
            )
            state_vector = tf.reshape(
                self.processed_obs[:, image_width * image_height * image_channels:],
                [tf.shape(self.processed_obs)[0], state_size, 1]
            )

            # First, pass through a CNN feature extractor with the image
            activ = tf.nn.relu
            extracted_features = compressed_nature_cnn(depth_image, **kwargs)
            extracted_features = tf.layers.flatten(extracted_features)

            if use_dropout:
                extracted_features = tf.nn.dropout(extracted_features, rate=dropout_rate)

            # Simultaneously send the state vector throught a MLP
            act_fun = tf.tanh
            net_arch = [dict(vf=[32, 32], pi=[64, 64, 64])]
            pi_latent, vf_latent = mlp_extractor(
                tf.layers.flatten(state_vector), net_arch, act_fun)

            # Now concatenate the outputs of the two feature extractors
            # and pass them to the action and value sub-networks
            pi_h = tf.concat([extracted_features, pi_latent], 1)
            vf_h = tf.concat([extracted_features, vf_latent], 1)

            for i, layer_size in enumerate([64, 64, 64]):
                pi_h = activ(
                    tf.layers.dense(pi_h, layer_size, name='pi_fc' + str(i)))
            pi_latent = pi_h

            for i, layer_size in enumerate([32, 32]):
                vf_h = activ(
                    tf.layers.dense(vf_h, layer_size, name='vf_fc' + str(i)))
            self._value_fn = tf.layers.dense(vf_h, 1, name='vf')
            vf_latent = vf_h

            self._proba_distribution, self._policy, self.q_value = \
                self.pdtype.proba_distribution_from_latent(pi_latent, vf_latent,
                                                           init_scale=0.01)

        self._setup_init()

    def step(self, obs, state=None, mask=None, deterministic=False):
        if deterministic:
            action, value, neglogp = self.sess.run(
                [self.deterministic_action, self.value_flat, self.neglogp],
                {self.obs_ph: obs})
        else:
            action, value, neglogp = self.sess.run(
                [self.action, self.value_flat, self.neglogp],
                {self.obs_ph: obs})
        return action, value, self.initial_state, neglogp

    def proba_step(self, obs, state=None, mask=None):
        return self.sess.run(self.policy_proba, {self.obs_ph: obs})

    def value(self, obs, state=None, mask=None):
        return self.sess.run(self.value_flat, {self.obs_ph: obs})


class DoubleCnnAsymmetricalPolicy(ActorCriticPolicy):
    """
    Custom MLP policy of three layers of size 128 each for the actor and
     2 layers of 64 for the critic, with feature extraction done using two
     different CNNs
    """

    def __init__(self, sess, ob_space, ac_space, n_env, n_steps, n_batch,
                 reuse=False, **kwargs):
        super(DoubleCnnAsymmetricalPolicy, self).__init__(sess, ob_space, ac_space,
                                                          n_env, n_steps, n_batch,
                                                          reuse=reuse, scale=True)

        with tf.variable_scope("model", reuse=reuse):
            # First, pass through a CNN feature extractor
            activ = tf.nn.relu

            # Get size information about observation vector/matrix
            image_width = kwargs["image_width"]
            del (kwargs["image_width"])

            image_height = kwargs["image_height"]
            del (kwargs["image_height"])

            try:
                use_dropout = kwargs["use_dropout"]
                del (kwargs["use_dropout"])

                dropout_rate = kwargs["dropout_rate"]
                del (kwargs["dropout_rate"])
            except KeyError:
                use_dropout = False

            depth_features, semantic_features = \
                double_cnn_observation(self.processed_obs, activ, image_height,
                                       image_width, **kwargs)
            # Combine features
            extracted_features = tf.concat([depth_features, semantic_features], 1)
            if use_dropout:
                extracted_features = tf.nn.dropout(extracted_features, rate=dropout_rate)

            pi_h = extracted_features
            for i, layer_size in enumerate([64, 64, 64]):
                pi_h = activ(
                    tf.layers.dense(pi_h, layer_size, name='pi_fc' + str(i)))
            pi_latent = pi_h

            vf_h = extracted_features
            for i, layer_size in enumerate([32, 32]):
                vf_h = activ(
                    tf.layers.dense(vf_h, layer_size, name='vf_fc' + str(i)))
            self._value_fn = tf.layers.dense(vf_h, 1, name='vf')
            vf_latent = vf_h

            self._proba_distribution, self._policy, self.q_value = \
                self.pdtype.proba_distribution_from_latent(pi_latent, vf_latent,
                                                           init_scale=0.01)

        self._setup_init()

    def step(self, obs, state=None, mask=None, deterministic=False):
        if deterministic:
            action, value, neglogp = self.sess.run(
                [self.deterministic_action, self.value_flat, self.neglogp],
                {self.obs_ph: obs})
        else:
            action, value, neglogp = self.sess.run(
                [self.action, self.value_flat, self.neglogp],
                {self.obs_ph: obs})
        return action, value, self.initial_state, neglogp

    def proba_step(self, obs, state=None, mask=None):
        return self.sess.run(self.policy_proba, {self.obs_ph: obs})

    def value(self, obs, state=None, mask=None):
        return self.sess.run(self.value_flat, {self.obs_ph: obs})


class LstmDoubleCnnAsymmetricalPolicy(RecurrentActorCriticPolicy):
    """
    Custom MLP policy of three layers of size 128 each for the actor and
     2 layers of 64 for the critic, with two CNNs as feature extractor and
     a LSTM layer in between
    """

    recurrent = True

    def __init__(self, sess, ob_space, ac_space, n_env, n_steps, n_batch,
                 n_lstm=256, layer_norm=False, reuse=False, **kwargs):
        super(LstmDoubleCnnAsymmetricalPolicy, self).__init__(sess, ob_space,
                                                              ac_space, n_env,
                                                              n_steps, n_batch,
                                                              state_shape=(
                                                                  2 * n_lstm,),
                                                              reuse=reuse, scale=True)

        with tf.variable_scope("model", reuse=reuse):
            # First, pass through a CNN feature extractor
            activ = tf.nn.relu
            # Get size information about observation vector/matrix
            image_width = kwargs["image_width"]
            del (kwargs["image_width"])

            image_height = kwargs["image_height"]
            del (kwargs["image_height"])

            use_dropout = kwargs["use_dropout"]
            del (kwargs["use_dropout"])

            dropout_rate = kwargs["dropout_rate"]
            del (kwargs["dropout_rate"])

            depth_features, semantic_features = \
                double_cnn_observation(self.processed_obs, activ, image_height,
                                       image_width, **kwargs)
            # Combine features
            extracted_features = tf.concat([depth_features, semantic_features],
                                           1)
            if use_dropout:
                extracted_features = tf.nn.dropout(extracted_features, rate=dropout_rate)

            # Then go through a LSTM NN
            input_sequence = batch_to_seq(extracted_features, self.n_env,
                                          n_steps)
            masks = batch_to_seq(self.dones_ph, self.n_env, n_steps)
            rnn_output, self.snew = lstm(input_sequence, masks, self.states_ph,
                                         'lstm1', n_hidden=n_lstm,
                                         layer_norm=layer_norm)
            rnn_features = tf.layers.flatten(seq_to_batch(rnn_output))

            pi_h = rnn_features
            for i, layer_size in enumerate([64, 64, 64]):
                pi_h = activ(
                    tf.layers.dense(pi_h, layer_size, name='pi_fc' + str(i)))
            pi_latent = pi_h

            vf_h = rnn_features
            for i, layer_size in enumerate([32, 32]):
                vf_h = activ(
                    tf.layers.dense(vf_h, layer_size, name='vf_fc' + str(i)))
            self._value_fn = tf.layers.dense(vf_h, 1, name='vf')
            vf_latent = vf_h

            self._proba_distribution, self._policy, self.q_value = \
                self.pdtype.proba_distribution_from_latent(pi_latent, vf_latent,
                                                           init_scale=0.01)

        self._setup_init()

    def step(self, obs, state=None, mask=None, deterministic=False):
        if deterministic:
            return self.sess.run(
                [self.deterministic_action, self.value_flat, self.snew,
                 self.neglogp],
                {self.obs_ph: obs, self.states_ph: state, self.dones_ph: mask})
        else:
            return self.sess.run(
                [self.action, self.value_flat, self.snew, self.neglogp],
                {self.obs_ph: obs, self.states_ph: state, self.dones_ph: mask})

    def proba_step(self, obs, state=None, mask=None):
        return self.sess.run(self.policy_proba,
                             {self.obs_ph: obs, self.states_ph: state,
                              self.dones_ph: mask})

    def value(self, obs, state=None, mask=None):
        return self.sess.run(self.value_flat,
                             {self.obs_ph: obs, self.states_ph: state,
                              self.dones_ph: mask})


class ConfidenceAwareAsymmetricalPolicy(ActorCriticPolicy):
    """
    Custom MLP policy of three layers of size 128 each for the actor and
     2 layers of 64 for the critic, with feature extraction done using two
     different CNNs
    """

    def __init__(self, sess, ob_space, ac_space, n_env, n_steps, n_batch,
                 reuse=False, **kwargs):
        super(ConfidenceAwareAsymmetricalPolicy, self).__init__(sess, ob_space, ac_space,
                                                                n_env, n_steps, n_batch,
                                                                reuse=reuse, scale=True)

        with tf.variable_scope("model", reuse=reuse):
            # First, pass through a CNN feature extractor
            activ = tf.nn.relu

            # Get size information about observation vector/matrix
            image_width = kwargs["image_width"]
            del (kwargs["image_width"])

            image_height = kwargs["image_height"]
            del (kwargs["image_height"])

            use_dropout = kwargs["use_dropout"]
            del (kwargs["use_dropout"])

            dropout_rate = kwargs["dropout_rate"]
            del (kwargs["dropout_rate"])

            depth_features, semantic_features = \
                confidence_aware_cnn_observation(self.processed_obs, activ, image_height,
                                                 image_width, **kwargs)
            # Combine features
            extracted_features = tf.concat([depth_features, semantic_features], 1)
            if use_dropout:
                extracted_features = tf.nn.dropout(extracted_features, rate=dropout_rate)

            pi_h = extracted_features
            for i, layer_size in enumerate([64, 64, 64]):
                pi_h = activ(
                    tf.layers.dense(pi_h, layer_size, name='pi_fc' + str(i)))
            pi_latent = pi_h

            vf_h = extracted_features
            for i, layer_size in enumerate([32, 32]):
                vf_h = activ(
                    tf.layers.dense(vf_h, layer_size, name='vf_fc' + str(i)))
            self._value_fn = tf.layers.dense(vf_h, 1, name='vf')
            vf_latent = vf_h

            self._proba_distribution, self._policy, self.q_value = \
                self.pdtype.proba_distribution_from_latent(pi_latent, vf_latent,
                                                           init_scale=0.01)

        self._setup_init()

    def step(self, obs, state=None, mask=None, deterministic=False):
        if deterministic:
            action, value, neglogp = self.sess.run(
                [self.deterministic_action, self.value_flat, self.neglogp],
                {self.obs_ph: obs})
        else:
            action, value, neglogp = self.sess.run(
                [self.action, self.value_flat, self.neglogp],
                {self.obs_ph: obs})
        return action, value, self.initial_state, neglogp

    def proba_step(self, obs, state=None, mask=None):
        return self.sess.run(self.policy_proba, {self.obs_ph: obs})

    def value(self, obs, state=None, mask=None):
        return self.sess.run(self.value_flat, {self.obs_ph: obs})
