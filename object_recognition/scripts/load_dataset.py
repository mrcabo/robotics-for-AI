import random

import numpy as np
from os.path import join, isfile
from os import listdir
import cv2
classes = ["crazyflie", "erazer_box", "evergreen", "jetson", "powerbank", "raspicam", "whitebox"]


def load_category(class_name, dataset_path):
    category_path = join(dataset_path, class_name)
    filenames = [file_name[:-4] for file_name in listdir(category_path) if file_name[-4:] == ".npy"]
    print(class_name, len(filenames))
    ret = []
    rois = []
    for file_name in filenames:
        image = cv2.imread(join(dataset_path, class_name, file_name + ".jpg"))
        ret.append(image)
        rois.append(np.load(join(dataset_path, class_name, file_name + ".npy")).astype('int'))
    return np.array(ret), np.array(rois)


def load_dataset(path='/home/group9/dataset'):
    all_data = ()
    all_rois = ()
    labels = []

    for label, class_name in enumerate(classes):
        data, rois = load_category(class_name, dataset_path=path)
        all_data += (data,)
        all_rois += (rois,)
        labels.extend([label] * data.shape[0])

    return np.concatenate(all_data), np.array(labels), np.concatenate(all_rois)


def load_from_npy(path='.'):
    return np.load(join(path, 'data.npy')), np.load(join(path, 'labels.npy')), np.load(join(path, 'rois.npy'))


def random_part():
    return random.randint(0, 20) - 15


def random_brightness_change(image):
    hsvImg = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # decreasing the V channel by a factor from the original
    values = hsvImg[:, :, 2] * (1 + ((random.random() - 0.5) / 2))
    hsvImg[:, :, 2] = values

    hsvImg = np.clip(hsvImg, 0, 255)

    return cv2.cvtColor(hsvImg, cv2.COLOR_HSV2BGR)


def salt_and_pepper(image):
    row, col, ch = image.shape
    s_vs_p = 0.5
    amount = 0.004
    out = np.copy(image)
    # Salt mode
    num_salt = np.ceil(amount * image.size * s_vs_p)
    coords = [np.random.randint(0, i - 1, int(num_salt))
              for i in image.shape]
    out[coords] = 255

    # Pepper mode
    num_pepper = np.ceil(amount * image.size * (1. - s_vs_p))
    coords = [np.random.randint(0, i - 1, int(num_pepper))
              for i in image.shape]
    out[coords] = 0

    return out


def cut_and_scale(image, rois, size=64):
    # cut
    image_crop = image[rois[0]:rois[1], rois[2]:rois[3]]
    # scale
    scaled = cv2.resize(image_crop, (size, size))

    return scaled


def augment(image, rois, size=64, show_img=False):
    scaled = cut_and_scale(image, rois, size)
    # random hsv value change
    brightness_adjusted = random_brightness_change(scaled)
    # add some pepper noise
    pepper_noised = salt_and_pepper(brightness_adjusted)
    # Rotation

    if show_img:
        cv2.imshow('test', pepper_noised)
        cv2.waitKey(0)
    return pepper_noised


def create_augmented_dataset(data, labels, rois, offset=20):
    augmented_data = []
    augmented_labels = []

    diffs = [-offset, 0, offset]
    for image, label, roi in zip(data, labels, rois):
        for d_x in diffs:
            for d_y in diffs:
                temp_roi = [roi[0] + d_y + random_part(), roi[1] + d_y + random_part(), roi[2] + d_x + random_part(),
                            roi[3] + d_x + random_part()]
                augmented_data.append(augment(image, temp_roi))
                augmented_labels.append(label)

    return np.array(augmented_data), np.array(augmented_labels)


# def split_train_test(data, labels, split=0.9):
#     length = data.shape[0]
#     permutation = np.random.permutation(length)
#     data = data[permutation]
#     labels = labels[permutation]
#
#     split_idx = int(length*split)
#     return data[:split_idx], labels[:split_idx], data[split_idx:], labels[split_idx:]


def split_train_test(split=0.8, path='.'):
    data, labels, rois = load_from_npy(path=path)
    print(data.shape, labels.shape, rois.shape)

    x_train = []
    y_train = []
    x_test = []
    y_test = []

    for class_label in range(len(classes)):
        class_idxs = np.where(labels == class_label)
        class_data = data[class_idxs]
        class_rois = rois[class_idxs]
        class_labels = labels[class_idxs]
        split_idx = int(split * class_data.shape[0])

        # Augment training data
        a_data, a_labels = create_augmented_dataset(class_data[:split_idx], class_labels[:split_idx], class_rois[:split_idx])
        x_train.append(a_data)
        y_train.append(a_labels)
        # Don't augment testing data
        x_test.append(np.array([cut_and_scale(image, roi) for image, roi in zip(class_data[split_idx:], class_rois[split_idx:])]))
        y_test.append(class_labels[split_idx:])

    return np.concatenate(x_train), np.concatenate(y_train), np.concatenate(x_test), np.concatenate(y_test)


# x_train, y_train, x_test, y_test = split_train_test(.8)
# print(x_train.shape, y_train.shape, x_test.shape, y_test.shape)
#
# cv2.namedWindow('test', cv2.WINDOW_NORMAL)
# for i in range(30, 40):
#     print('test')
#     print(y_test[i])
#     cv2.imshow('test', x_test[i])
#     cv2.waitKey(0)
#     print('train')
#     print(y_train[i*100])
#     cv2.imshow('test', x_train[i*100])
#     cv2.waitKey(0)

# data, labels, rois = load_from_npy()
# print(data.shape, labels.shape, rois.shape)
# augmented_data, augmented_labels = create_augmented_dataset(data, labels, rois)
# print(augmented_data.shape, augmented_labels.shape)
#
# np.save("augmented_data.npy", augmented_data)
# np.save("augmented_labels.npy", augmented_labels)

# idx = 250
# cv2.namedWindow('test', cv2.WINDOW_NORMAL)
# print('label: %d' % labels[idx])
# offset = 20
# diffs = [-offset, 0, offset]
# for d_x in diffs:
#     for d_y in diffs:
#         x_1 = rois[idx][2] + d_x + random_part()
#         y_1 = rois[idx][0] + d_y + random_part()
#         x_2 = rois[idx][3] + d_x + random_part()
#         y_2 = rois[idx][1] + d_y + random_part()
#         augment(data[idx], [y_1, y_2, x_1, x_2], show_img=True)


