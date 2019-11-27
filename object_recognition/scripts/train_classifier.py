from __future__ import absolute_import, division, print_function, unicode_literals

import os

import numpy as np

import cv2
import matplotlib.pyplot as plt
import tensorflow as tf
from load_dataset import split_train_test
from sklearn.model_selection import StratifiedKFold


def load_data():
    x_train, y_train, x_test, y_test = split_train_test(.9, '/home/group9/catkin_ws/src/object_recognition/scripts/')
    print(x_train.shape, y_train.shape, x_test.shape, y_test.shape)

    # Normalize the data
    x_train = x_train / 255
    x_test = x_test / 255

    # Shuffle training data
    permutation = np.random.permutation(x_train.shape[0])
    x_train = x_train[permutation]
    y_train = y_train[permutation]

    return x_train, y_train, x_test, y_test


def create_model():
    # Create a NN, just for testing
    # model = tf.keras.models.Sequential([
    #   tf.keras.layers.Flatten(input_shape=(64, 64, 3)),
    #   tf.keras.layers.Dense(128, activation='relu'),
    #   tf.keras.layers.Dropout(0.33),
    #   tf.keras.layers.Dense(7, activation='softmax')
    # ])

    model = tf.keras.models.Sequential([
        tf.keras.layers.Conv2D(16, kernel_size=3, activation='relu', input_shape=(64, 64, 3)),
        tf.keras.layers.Conv2D(8, kernel_size=3, activation='relu'),
        tf.keras.layers.Flatten(),
        tf.keras.layers.Dense(128, activation='relu'),
        tf.keras.layers.Dropout(0.4),
        tf.keras.layers.Dense(7, activation='softmax')
    ])

    model.compile(optimizer='adam',
                  loss='sparse_categorical_crossentropy',
                  metrics=['accuracy'])

    return model


def plot_one_split():
    x_train, y_train, x_test, y_test = load_data()
    model = create_model()
    # When calling the fit function, it returns a history object
    # We give it a specific validation set (in this case the mnist test data)
    history = model.fit(x_train, y_train, batch_size=128, validation_data=(x_test, y_test), epochs=3,  verbose=1,
                        shuffle=True, workers=0)

    path = os.path.join(os.environ['HOME'], 'network_model')

    model.save(os.path.join(path, "16_8_128_7_convnet.h5"))

    # plot the training accuracy
    plt.plot(history.history['accuracy'])
    # plot the validation accuracy
    plt.plot(history.history['val_accuracy'])

    # Force the y axis to show between 0 and 1, else it will auto scale
    plt.ylim(0, 1)

    plt.title('Model accuracy')
    plt.ylabel('Accuracy')
    plt.xlabel('Epoch')
    plt.legend(['Train', 'Test'], loc='upper left')
    plt.show()

    # plot the training loss
    plt.plot(history.history['loss'])
    # plot the validation loss
    plt.plot(history.history['val_loss'])
    plt.title('Model loss')
    plt.ylabel('Los')
    plt.xlabel('Epoch')
    plt.legend(['Train', 'Test'], loc='upper left')
    plt.show()


def cross_validate(n_splits=4):
    x_train, y_train, x_test, y_test = load_data()
    kfold = StratifiedKFold(n_splits=n_splits, shuffle=True)
    test_scores = []
    cvscores = []
    for train_idxs, test_idxs in kfold.split(x_train, y_train):
        model = create_model()
        history = model.fit(x_train[train_idxs], y_train[train_idxs], batch_size=128, validation_data=(x_test, y_test),
                            epochs=5, verbose=0, shuffle=True, workers=0)
        scores = model.evaluate(x_train[test_idxs], y_train[test_idxs], verbose=0, workers=0)
        print("%s: %.2f%%" % (model.metrics_names[1], scores[1] * 100))
        cvscores.append(scores[1] * 100)

        test_score = model.evaluate(x_test, y_test, verbose=0, workers=0)
        print("test_%s: %.2f%%" % (model.metrics_names[1], test_score[1] * 100))
        test_scores.append(test_score[1] * 100)
        del model

    print(cvscores)
    print(np.mean(cvscores))
    print(np.std(cvscores))
    print("val %.2f" % (sum(cvscores) / len(cvscores)))

    print(test_scores)
    print("test %.2f" % (sum(test_scores) / len(test_scores)))
    print(np.mean(test_scores))
    print(np.std(test_scores))

    # print("%.2f%% (+/- %.2f%%)" % (np.mean(cvscores), np.std(cvscores)))

# cross_validate()
plot_one_split()

# Cross validation
# cross_validate()