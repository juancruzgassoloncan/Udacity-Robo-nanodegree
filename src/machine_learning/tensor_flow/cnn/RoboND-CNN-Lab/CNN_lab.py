
# coding: utf-8

# # Image Classification
# In this lab, you'll classify images from the [Fashion-MNIST dataset](https://github.com/zalandoresearch/fashion-mnist#get-the-data).  The dataset consists of different types of clothing items such as shirts, trousers, sneakers etc. You'll preprocess the images, then train a convolutional neural network on all the samples. The images need to be normalized and the labels need to be one-hot encoded.  You'll get to apply what you learned and build a model with convolutional, max pooling, dropout, and fully connected layers.  At the end, you'll get to see your neural network's predictions on the sample images.
# ## Get the Data
# We have provided you with a pickle file for the dataset available in the GitHub repo. We have provided with a script - helper.py, which extracts the dataset for you when the corresponding functions are called.

# ## Explore the Data
# The Fashion-MNIST dataset consists of a training set of 60,000 examples and a test set of 10,000 examples. Each example is a 28x28 grayscale image, associated with a label from the following 10 classes:
#
# * T-shirt/top
# * Trouser
# * Pullover
# * Dress
# * Coat
# * Sandal
# * Shirt
# * Sneaker
# * Bag
# * Ankle boot
#
# Understanding a dataset is part of making predictions on the data.  Play around with the code cell below by changing the `sample_id`. The `sample_id` is the id for a image and label pair in the dataset.
#
# Ask yourself "What are all possible labels?", "What is the range of values for the image data?", "Are the labels in order or random?".  Answers to questions like these will help you preprocess the data and end up with better predictions.

# In[3]:


get_ipython().magic('matplotlib inline')
get_ipython().magic("config InlineBackend.figure_format = 'retina'")

import helper
import numpy as np

import pickle

filename = "fashion-mnist.p"

# Explore the dataset
sample_id = 8
helper.display_stats(filename, sample_id)


# ## Implement Preprocess Functions
# ### Normalize
# In the cell below, implement the `normalize` function to take in image data, `x`, and return it as a normalized Numpy array. The values should be in the range of 0 to 1, inclusive.  The return object should be the same shape as `x`.

# In[4]:


import problem_unittests as tests
def normalize(x):
    """
    Normalize a list of sample image data in the range of 0 to 1
    : x: List of image data.  The image shape is (28, 28, 1)
    : return: Numpy array of normalize data
    """
    # TODO: Implement Function
    max_x = x.max()
    min_x = x.min()
    rng = (0,1)
    out = rng[0] + ((x - min_x)*(rng[1]-rng[0]))/(max_x - min_x)
    return out


"""
DON'T MODIFY ANYTHING IN THIS CELL THAT IS BELOW THIS LINE
"""
tests.test_normalize(normalize)


# ### One-hot encode
# Just like the previous code cell, you'll be implementing a function for preprocessing.  This time, you'll implement the `one_hot_encode` function. The input, `x`, are a list of labels.  Implement the function to return the list of labels as One-Hot encoded Numpy array.  The possible values for labels are 0 to 9. The one-hot encoding function should return the same encoding for each value between each call to `one_hot_encode`.  Make sure to save the map of encodings outside the function.
#
# Hint: Don't reinvent the wheel. You have multiple ways to attempt this: Numpy, TF, or even sklearn's preprocessing package.

# In[5]:


import tensorflow as tf

def one_hot_encode(x):
    """
    One hot encode a list of sample labels. Return a one-hot encoded vector for each label.
    : x: List of sample Labels
    : return: Numpy array of one-hot encoded labels
    """
    # TODO: Implement Function
    encode = tf.one_hot(x,10)
    with tf.Session() as sess:
        return(sess.run(encode))




"""
DON'T MODIFY ANYTHING IN THIS CELL THAT IS BELOW THIS LINE
"""
tests.test_one_hot_encode(one_hot_encode)


# ### Randomize Data
# As you saw from exploring the data above, the order of the samples are randomized.  It doesn't hurt to randomize it again, but you don't need to for this dataset.

# ## Preprocess all the data and save it
# Running the code cell below will preprocess all the Fashion-MNIST data and save it to file. The code below also uses 10% of the training data for validation.

# In[6]:


"""
DON'T MODIFY ANYTHING IN THIS CELL
"""
# Preprocess Training, Validation, and Testing Data
helper.preprocess_and_save_data(filename, normalize, one_hot_encode)


# # Check Point
# This is your first checkpoint.  If you ever decide to come back to this notebook or have to restart the notebook, you can start from here.  The preprocessed data has been saved to disk.

# In[7]:


"""
DON'T MODIFY ANYTHING IN THIS CELL
"""
import pickle
import problem_unittests as tests
import helper

# Load the Preprocessed Validation data
valid_features, valid_labels = pickle.load(open('preprocess_validation.p', mode='rb'))


# ## Build the network
# For the neural network, you'll build each layer into a function.  Most of the code you've seen has been outside of functions. To test your code more thoroughly, we require that you put each layer in a function.  This allows us to give you better feedback and test for simple mistakes using our unittests.
#
# Let's begin!
#
# ### Input
# The neural network needs to read the image data, one-hot encoded labels, and dropout keep probability. Implement the following functions
# * Implement `neural_net_image_input`
#  * Return a [TF Placeholder](https://www.tensorflow.org/api_docs/python/tf/placeholder)
#  * Set the shape using `image_shape` with batch size set to `None`.
#  * Name the TensorFlow placeholder "x" using the TensorFlow `name` parameter in the [TF Placeholder](https://www.tensorflow.org/api_docs/python/tf/placeholder).
# * Implement `neural_net_label_input`
#  * Return a [TF Placeholder](https://www.tensorflow.org/api_docs/python/tf/placeholder)
#  * Set the shape using `n_classes` with batch size set to `None`.
#  * Name the TensorFlow placeholder "y" using the TensorFlow `name` parameter in the [TF Placeholder](https://www.tensorflow.org/api_docs/python/tf/placeholder).
# * Implement `neural_net_keep_prob_input`
#  * Return a [TF Placeholder](https://www.tensorflow.org/api_docs/python/tf/placeholder) for dropout keep probability.
#  * Name the TensorFlow placeholder "keep_prob" using the TensorFlow `name` parameter in the [TF Placeholder](https://www.tensorflow.org/api_docs/python/tf/placeholder).
#
# These names will be used at the end of the lab to load your saved model.
#
# Note: `None` for shapes in TensorFlow allow for a dynamic size.

# In[9]:


import tensorflow as tf

def neural_net_image_input(image_shape):
    """
    Return a Tensor for a batch of image input
    : image_shape: Shape of the images
    : return: Tensor for image input.
    """
    h = image_shape[0]
    w = image_shape[1]
    c = image_shape[2]
    img_input = tf.placeholder(tf.float32,shape=[None, h, w, c],name='x')
    return img_input



def neural_net_label_input(n_classes):
    """
    Return a Tensor for a batch of label input
    : n_classes: Number of classes
    : return: Tensor for label input.
    """
    return tf.placeholder(tf.float32,shape=[None,n_classes],name='y')


def neural_net_keep_prob_input():
    """
    Return a Tensor for keep probability
    : return: Tensor for keep probability.
    """

    return tf.placeholder(tf.float32,name='keep_prob')


"""
DON'T MODIFY ANYTHING IN THIS CELL THAT IS BELOW THIS LINE
"""
tf.reset_default_graph()
tests.test_nn_image_inputs(neural_net_image_input)
tests.test_nn_label_inputs(neural_net_label_input)
tests.test_nn_keep_prob_inputs(neural_net_keep_prob_input)


# ### Convolution and Max Pooling Layer
# Convolution layers have a lot of success with images. For this code cell, you should implement the function `conv2d_maxpool` to apply convolution then max pooling:
# * Create the weight and bias using `conv_ksize`, `conv_num_outputs` and the shape of `x_tensor`.
# * Apply a convolution to `x_tensor` using weight and `conv_strides`.
#  * We recommend you use same padding, but you're welcome to use any padding.
# * Add bias
# * Add a nonlinear activation to the convolution.
# * Apply Max Pooling using `pool_ksize` and `pool_strides`.
#  * We recommend you use same padding, but you're welcome to use any padding.

# In[34]:


def conv2d_maxpool(x_tensor, conv_num_outputs, conv_ksize, conv_strides, pool_ksize, pool_strides):
    """
    Apply convolution then max pooling to x_tensor
    :param x_tensor: TensorFlow Tensor
    :param conv_num_outputs: Number of outputs for the convolutional layer
    :param conv_ksize: kernal size 2-D Tuple for the convolutional layer
    :param conv_strides: Stride 2-D Tuple for convolution
    :param pool_ksize: kernal size 2-D Tuple for pool
    :param pool_strides: Stride 2-D Tuple for pool
    : return: A tensor that represents convolution and max pooling of x_tensor
    """
    # The shape of the filter weight is (height, width, input_depth, output_depth)
    kernel = [conv_ksize[0], conv_ksize[1], x_tensor.get_shape().as_list()[-1], conv_num_outputs]
    w = tf.Variable(tf.truncated_normal(kernel, stddev=0.01))
    b = tf.Variable(tf.zeros([conv_num_outputs]))

    x = tf.nn.conv2d(x_tensor,w,[1, conv_strides[0], conv_strides[1], 1],padding='SAME')
    x = tf.nn.bias_add(x,b)
    x = tf.nn.relu(x)

    x = tf.nn.max_pool(x,
                       ksize = [1, pool_ksize[0], pool_ksize[1], 1],
                       strides = [1,pool_strides[0],pool_strides[1],1],
                       padding='SAME')
    return x


"""
DON'T MODIFY ANYTHING IN THIS CELL THAT IS BELOW THIS LINE
"""
tests.test_con_pool(conv2d_maxpool)


# ### Flatten Layer
# Implement the `flatten` function to change the dimension of `x_tensor` from a 4-D tensor to a 2-D tensor.  The output should be the shape (*Batch Size*, *Flattened Image Size*).
#
# Shortcut Option: you can use classes from the [TensorFlow Layers](https://www.tensorflow.org/api_docs/python/tf/layers) or [TensorFlow Layers (contrib)](https://www.tensorflow.org/api_guides/python/contrib.layers) packages for this layer which help with some high-level features. For more of a challenge, only use other TensorFlow packages.

# In[128]:


x_tensor = tf.random_normal([3,2,2,3])
shape = x_tensor.get_shape().as_list()
new_shape = np.prod(shape[1:])
new = tf.reshape(x_tensor,(-1,new_shape))

# new.set_shape(tf.cast(new_shape,tf.int32))
# new.set_shape([None])
# for img in t:



with tf.Session()as sess:
    print(sess.run(new))
    print(new.get_shape())



# In[131]:


def flatten(x_tensor):
    """
    Flatten x_tensor to (Batch Size, Flattened Image Size)
    : x_tensor: A tensor of size (Batch Size, ...), where ... are the image dimensions.
    : return: A tensor of size (Batch Size, Flattened Image Size).
    """
    shape = x_tensor.get_shape().as_list()
    new_shape = np.prod(shape[1:])
    new = tf.reshape(x_tensor,(-1,new_shape))
    return new
#     return tf.layers.flatten(x_tensor)

"""
DON'T MODIFY ANYTHING IN THIS CELL THAT IS BELOW THIS LINE
"""
tests.test_flatten(flatten)


# ### Fully-Connected Layer
# Implement the `fully_conn` function to apply a fully connected layer to `x_tensor` with the shape (*Batch Size*, *num_outputs*).
#
# Shortcut option: you can use classes from the [TensorFlow Layers](https://www.tensorflow.org/api_docs/python/tf/layers) or [TensorFlow Layers (contrib)](https://www.tensorflow.org/api_guides/python/contrib.layers) packages for this layer. For more of a challenge, only use other TensorFlow packages.

# In[138]:


def fully_conn(x_tensor, num_outputs):
    """
    Apply a fully connected layer to x_tensor using weight and bias
    : x_tensor: A 2-D tensor where the first dimension is batch size.
    : num_outputs: The number of output that the new tensor should be.
    : return: A 2-D tensor where the second dimension is num_outputs.
    """
    w = tf.Variable(tf.truncated_normal([x_tensor.get_shape().as_list()[-1], num_outputs], stddev=0.01))
    b = tf.Variable(tf.zeros([num_outputs]))

    layer = tf.add(tf.matmul(x_tensor,w),b)
    tf.nn.relu(layer)

    return layer
#     return tf.layers.dense(x_tensor,num_outputs,activation=tf.nn.relu)


"""
DON'T MODIFY ANYTHING IN THIS CELL THAT IS BELOW THIS LINE
"""
tests.test_fully_conn(fully_conn)


# ### Output Layer
# Implement the `output` function to apply a fully connected layer to `x_tensor` with the shape (*Batch Size*, *num_outputs*).
#
# Shortcut option: you can use classes from the [TensorFlow Layers](https://www.tensorflow.org/api_docs/python/tf/layers) or [TensorFlow Layers (contrib)](https://www.tensorflow.org/api_guides/python/contrib.layers) packages for this layer. For more of a challenge, only use other TensorFlow packages.
#
# **Note:** Activation, softmax, or cross entropy should **not** be applied to this.

# In[140]:



def output(x_tensor, num_outputs):
    """
    Apply a output layer to x_tensor using weight and bias
    : x_tensor: A 2-D tensor where the first dimension is batch size.
    : num_outputs: The number of output that the new tensor should be.
    : return: A 2-D tensor where the second dimension is num_outputs.
    """
    # TODO: Implement Function
    w = tf.Variable(tf.truncated_normal([x_tensor.get_shape().as_list()[-1], num_outputs], stddev=0.01))
    b = tf.Variable(tf.zeros([num_outputs]))
    layer = tf.add(tf.matmul(x_tensor,w),b)
    return layer


"""
DON'T MODIFY ANYTHING IN THIS CELL THAT IS BELOW THIS LINE
"""
tests.test_output(output)


# ### Create Convolutional Model
# Implement the function `conv_net` to create a convolutional neural network model. The function takes in a batch of images, `x`, and outputs logits.  Use the layers you created above to create this model:
#
# * Apply 1, 2, or 3 Convolution and Max Pool layers
# * Apply a Flatten Layer
# * Apply 1, 2, or 3 Fully Connected Layers
# * Apply an Output Layer
# * Return the output
# * Apply [TensorFlow's Dropout](https://www.tensorflow.org/api_docs/python/tf/nn/dropout) to one or more layers in the model using `keep_prob`.

# In[153]:


def conv_net(x, keep_prob):
    """
    Create a convolutional neural network model
    : x: Placeholder tensor that holds image data.
    : keep_prob: Placeholder tensor that hold dropout keep probability.
    : return: Tensor that represents logits
    """
    # TODO: Apply 1, 2, or 3 Convolution and Max Pool layers
    #    Play around with different number of outputs, kernel size and stride
    # Function Definition from Above:
    conv = conv2d_maxpool(x,
                   conv_num_outputs=32,
                   conv_ksize=(5,5),
                   conv_strides=(2,2),
                   pool_ksize=(2,2),
                   pool_strides=(1,1))
    conv = conv2d_maxpool(conv,
                   conv_num_outputs=64,
                   conv_ksize=(3,3),
                   conv_strides=(1,1),
                   pool_ksize=(2,2),
                   pool_strides=(1,1))
    conv = tf.nn.dropout(conv, keep_prob)
    conv = conv2d_maxpool(conv,
                   conv_num_outputs=128,
                   conv_ksize=(3,3),
                   conv_strides=(1,1),
                   pool_ksize=(2,2),
                   pool_strides=(1,1))
    conv = tf.nn.dropout(conv,keep_prob)

    # TODO: Apply a Flatten Layer
    # Function Definition from Above:
    flat = flatten(conv)

    # TODO: Apply 1, 2, or 3 Fully Connected Layers
    #    Play around with different number of outputs
    # Function Definition from Above:
    fc = fully_conn(flat, 256)
    fc = fully_conn(fc, 128)
    fc = fully_conn(fc, 64)
    # TODO: Apply an Output Layer
    #    Set this to the number of classes
    # Function Definition from Above:
    last_layer = output(fc, 10)
#     last_later = tf.nn.softmax(last_layer)
    # TODO: return output
    return last_layer


"""
DON'T MODIFY ANYTHING IN THIS CELL THAT IS BELOW THIS LINE
"""

##############################
## Build the Neural Network ##
##############################

# Remove previous weights, bias, inputs, etc..
tf.reset_default_graph()

# Inputs
x = neural_net_image_input((28, 28, 1))
y = neural_net_label_input(10)
keep_prob = neural_net_keep_prob_input()

# Model
logits = conv_net(x, keep_prob)

# Name logits Tensor, so that is can be loaded from disk after training
logits = tf.identity(logits, name='logits')

# Loss and Optimizer
cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=logits, labels=y))
optimizer = tf.train.AdamOptimizer().minimize(cost)

# Accuracy
correct_pred = tf.equal(tf.argmax(logits, 1), tf.argmax(y, 1))
accuracy = tf.reduce_mean(tf.cast(correct_pred, tf.float32), name='accuracy')

tests.test_conv_net(conv_net)


# ## Train the Neural Network
# ### Single Optimization
# Implement the function `train_neural_network` to do a single optimization.  The optimization should use `optimizer` to optimize in `session` with a `feed_dict` of the following:
# * `x` for image input
# * `y` for labels
# * `keep_prob` for keep probability for dropout
#
# This function will be called for each batch, so `tf.global_variables_initializer()` has already been called.
#
# Hint: You can refer to the "Convolutional Network in TensorFlow" section in the lesson.
#
# Note: Nothing needs to be returned. This function is only optimizing the neural network.

# In[154]:



def train_neural_network(session, optimizer, keep_probability, feature_batch, label_batch):
    """
    Optimize the session on a batch of images and labels
    : session: Current TensorFlow session
    : optimizer: TensorFlow optimizer function
    : keep_probability: keep probability
    : feature_batch: Batch of Numpy image data
    : label_batch: Batch of Numpy label data
    """
    session.run(optimizer, feed_dict={x: feature_batch,
                                       y: label_batch,
                                       keep_prob: keep_probability})



"""
DON'T MODIFY ANYTHING IN THIS CELL THAT IS BELOW THIS LINE
"""
tests.test_train_nn(train_neural_network)


# ### Show Stats
# Implement the function `print_stats` to print loss and validation accuracy.  Use the global variables `valid_features` and `valid_labels` to calculate validation accuracy.  Use a keep probability of `1.0` to calculate the loss and validation accuracy.
#
# Hint: You can refer to the "Convolutional Network in TensorFlow" section in the lesson.

# In[149]:


def print_stats(session, feature_batch, label_batch, cost, accuracy):
    """
    Print information about loss and validation accuracy
    : session: Current TensorFlow session
    : feature_batch: Batch of Numpy image data
    : label_batch: Batch of Numpy label data
    : cost: TensorFlow cost function
    : accuracy: TensorFlow accuracy function
    """
    # TODO: Calculate loss and validation accuracy
    loss = sess.run(cost,feed_dict={
                                     x: feature_batch,
                                     y: label_batch,
                                     keep_prob: 1.})
    validation_accuracy = sess.run(accuracy, feed_dict={
                                              x: valid_features,
                                              y: valid_labels,
                                              keep_prob: 1.})

    # TODO: Print loss and validation accuracy
    print("Loss: {0}; Valid_acc: {1}".format(loss,validation_accuracy))


# ### Hyperparameters
# Tune the following parameters:
# * Set `epochs` to the number of iterations until the network stops learning or start overfitting
# * Set `batch_size` to the highest number that your machine has memory for.  Most people set them to common sizes of memory:
#  * 64
#  * 128
#  * 256
#  * ...
# * Set `keep_probability` to the probability of keeping a node using dropout

# In[151]:


# TODO: Tune Parameters
epochs = 20
batch_size = 64
keep_probability = 0.4


# ### Train the Model
# Now that you have your model built and your hyperparameters defined, let's train it!

# In[152]:


"""
DON'T MODIFY ANYTHING IN THIS CELL
"""
save_model_path = './image_classification'

with tf.Session() as sess:
    # Initializing the variables
    sess.run(tf.global_variables_initializer())

    # Training cycle
    for epoch in range(epochs):
        for batch_features, batch_labels in helper.load_preprocess_training_batch(batch_size):
            train_neural_network(sess, optimizer, keep_probability, batch_features, batch_labels)
        print('Epoch {:>2}:  '.format(epoch + 1), end='')
        print_stats(sess, batch_features, batch_labels, cost, accuracy)

    # Save Model
    saver = tf.train.Saver()
    save_path = saver.save(sess, save_model_path)


# # Checkpoint
# The model has been saved to disk.
# ## Test Model
# Test your model against the test dataset.  This will be your final accuracy. You should have an accuracy greater than 50%. If you don't, keep tweaking the model architecture and parameters.

# In[ ]:


"""
DON'T MODIFY ANYTHING IN THIS CELL
"""
get_ipython().magic('matplotlib inline')
get_ipython().magic("config InlineBackend.figure_format = 'retina'")

import tensorflow as tf
import pickle
import helper
import random


save_model_path = './image_classification'
n_samples = 4
top_n_predictions = 3

def test_model():
    """
    Test the saved model against the test dataset
    """

    test_features, test_labels = pickle.load(open('preprocess_test.p', mode='rb'))
    loaded_graph = tf.Graph()

    config = tf.ConfigProto(device_count = {'GPU': 0})

    with tf.Session(config=config, graph=loaded_graph) as sess:
        # Load model
        loader = tf.train.import_meta_graph(save_model_path + '.meta')
        loader.restore(sess, save_model_path)

        # Get Tensors from loaded model
        loaded_x = loaded_graph.get_tensor_by_name('x:0')
        loaded_y = loaded_graph.get_tensor_by_name('y:0')
        loaded_keep_prob = loaded_graph.get_tensor_by_name('keep_prob:0')
        loaded_logits = loaded_graph.get_tensor_by_name('logits:0')
        loaded_acc = loaded_graph.get_tensor_by_name('accuracy:0')

        # Get accuracy in batches for memory limitations
        test_batch_acc_total = 0
        test_batch_count = 0

        for test_feature_batch, test_label_batch in helper.batch_features_labels(test_features, test_labels, batch_size):
            test_batch_acc_total += sess.run(
                loaded_acc,
                feed_dict={loaded_x: test_feature_batch, loaded_y: test_label_batch, loaded_keep_prob: 1.0})
            test_batch_count += 1

        print('Testing Accuracy: {}\n'.format(test_batch_acc_total/test_batch_count))

        # Print Random Samples
        random_test_features, random_test_labels = tuple(zip(*random.sample(list(zip(test_features, test_labels)), n_samples)))
        random_test_predictions = sess.run(
            tf.nn.top_k(tf.nn.softmax(loaded_logits), top_n_predictions),
            feed_dict={loaded_x: random_test_features, loaded_y: random_test_labels, loaded_keep_prob: 1.0})
        helper.display_image_predictions(random_test_features, random_test_labels, random_test_predictions)


test_model()
