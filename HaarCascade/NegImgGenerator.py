# Name: NegImgGenerator
# Author: Peter Mai and Harrison Kinsley
# Reference: https://pythonprogramming.net/haar-cascade-object-detection-python-opencv-tutorial/
#
# This program is used as preparation for haar cascade image recognition training
# and is composed of three primary features. It's first feature is to gather
# various images online and modifying them to be used as negative image. The
# second feature is it'll remove user-specified pictures that are bad picture.
# The third feature is it will generate a pos and neg file called bg.txt
#
# This program runs on Python 3

"""Modules used for downloading images online and converting them"""
import urllib.request
import argparse
import sys
import os
import cv2
import numpy as np

width = 0
height = 0

def generate_raw_images():
    """ Asks users for a link to a website that holds various other links to
    images that would be downloaded to generate the negative images. You can
    get this link by going to site such as http://image-net.org
    """

    global width
    global height

    # Ask user to specify how many images to collect along with width and height
    width = int(input("Please enter negative image width:    "))
    height = int(input("Please enter negative image height:   "))
    max_neg = int(input("How many negative image to generate:  "))

    print("\nGreat! You chose to generate " + str(max_neg) + " " + str(width) +
          "x" + str(height) + " negative images.\n")

    print("You can get a list of links from http://image-net.org after ")
    print("searching under Downloads > URLS\n")
    sources = int(input("How many of these sources do you want to provide: "))

    # Collect specified source url from user
    source_url = []
    for source_idx in range(0, sources):
        source_url.append(input("Please enter source link " + str(source_idx+1) +": "))

    print("\nStoring negative images to " + os.path.dirname(os.path.abspath(__file__))+"/neg...\n")

    # Create a 'neg' directory if it has not been created
    if not os.path.exists('neg'):
        os.makedirs('neg')

    # Keep track of current image
    pic_num = 0
    max_neg_per_source = max_neg / sources

    # For each link, gather max_neg / sources negative images
    for source_idx in range(0, sources):
        neg_images_link = source_url[source_idx]
        neg_images_urls = urllib.request.urlopen(neg_images_link).read().decode()

        # Attempt to download image and convert it to specified scaled image
        for url in neg_images_urls.split('\n'):
            try:
                print(url)
                urllib.request.urlretrieve(url, "neg/"+str(pic_num)+'.jpg')
                img = cv2.imread("neg/"+str(pic_num)+'.jpg', cv2.IMREAD_GRAYSCALE)
                resized_image = cv2.resize(img, (width, height))
                cv2.imwrite("neg/"+str(pic_num)+'.jpg', resized_image)
                pic_num += 1
                if pic_num % max_neg_per_source == 0:
                    break

            except Exception as ex:
                print(str(ex))

    print("Done.\n")


def remove_bad_images():
    """ Remove bad pictures that user specified. These images are often images
    retrieved when link no longer works and the host website returns an
    invalid image rather than a 404 error.
    """

    print("Please store any images you want to remove in ")
    print("  " + os.path.dirname(os.path.abspath(__file__)) + "/bad_images/...\n")
    input("Enter 'y' to continue: ")

    # Scan each image in 'neg' against all images in 'bad_images'
    print("Begin searching for files to remove...")
    for file_type in ['neg']:
        for img in os.listdir(file_type):
            for bad_img in os.listdir('bad_images'):
                try:
                    current_image_path = str(file_type) + '/' + str(img)
                    bad_img = cv2.imread('bad_images/' + str(bad_img))
                    question = cv2.imread(current_image_path)

                    # Remove 'neg' image if it matches 'bad_image'
                    if bad_img.shape == question.shape and not(np.bitwise_xor(bad_img,question).any()):
                        print("removing " + current_image_path)
                        os.remove(current_image_path)

                except Exception as ex:
                    print(str(ex))

    print("Done.\n")


def create_pos_n_neg():
    """ This module will generate a positive and negative file from the negative
    images gathered. It will produce a bg.txt background file
    """

    global width
    global height

    if width == 0:
        width = int(input("Please enter negative image width:    "))
    if height == 0:
        height = int(input("Please enter negative image height:    "))

    print("Begin generating background file.")
    for file_type in ['neg']:

        # Generate background file bg.txt
        for img in os.listdir(file_type):
            if file_type == 'neg':
                line = file_type+'/'+img+'\n'
                with open('bg.txt', 'a') as f:
                    f.write(line)

            elif file_type == 'pos':
                line = file_type+'/'+img+' 1 0 0 ' + str(width) + ' ' + str(height) + '\n'
                with open('info.dat', 'a') as f:
                    f.write(line)

    print("Done.\n")



# Ask for command-line arguments on how to run program
parser = argparse.ArgumentParser()
parser.add_argument("method",help="\"all\" Perform all three steps.\n" +
                                  "\"generate_raw_images\" Only generate neg images.\n" +
                                  "\"remove_bad_images\" Remove bad images.\n" +
                                  "\"create_pos_n_neg\" Generate bg.txt.\n")
args = parser.parse_args()

# Run script based on what user specified
if args.method == "all":
    generate_raw_images()
    remove_bad_images()
    create_pos_n_neg()
elif args.method == "generate_raw_images":
    generate_raw_images()
elif args.method == "remove_bad_images":
    remove_bad_images()
elif args.method == "create_pos_n_neg":
    create_pos_n_neg()
else:
    print("error: " + args.method + " is not a valid argument")
    parser.print_help()
    sys.exit()
