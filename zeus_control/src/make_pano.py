#!/usr/bin/env python 

# -*- coding: utf-8 -*-

# Created on Jul 30 2021
# @author: Simon Chamorro       simon.chamorro@usherbrooke.ca


"""
@package zeus_control

------------------------------------

Python script to make panorama using hugin panorama

"""
import os
import argparse
import subprocess
from os import listdir
from os.path import isfile, join


def parse_args():
    '''
    Parser for the arguments
    ----------
    Returns
    ----------
    args : obj
        The arguments
    '''
    parser = argparse.ArgumentParser('''Make a panoram with jpg images from a given folder.''')
    parser.add_argument('--path',
                        type=str,
                        default='/home/simon/test',
                        help='path to folder containing images')
    args = parser.parse_args()
    return args


def make_panorama(path, images):
    '''
    Make panorama
    '''
    img_string = ''
    for im in images:
        img_string += (' ' + im)
    subprocess.call(os.getcwd() + "/make_pano.sh " + path + img_string, shell=True)


def get_images(path):
    '''
    Get images
    ----------
    Params
    ----------
    path : String
        Absolute path to images
    ----------
    Return
    ----------
    images: List
        List of image paths
    '''
    files = [f for f in listdir(path) if isfile(join(path, f))]
    images = [path + '/' + f for f in files if f.split('.')[-1] == 'jpg']
    return images


def main():
    # Load the arguments
    args = parse_args()

    # Get images
    images = get_images(args.path)

    # Make panorama
    make_panorama(args.path, images)



if __name__ == '__main__':
    main()
