#!/bin/sh
# $1=dir
# $2=pic1
# $3=pic2
# $4=pic3

# Assembles a Hugin .pto project file using equirectangular projection.
pto_gen -o $1'/panorama.pto' -p 1 -f 60 $2 $3 $4 $5 $6 $7 $8 $9 

# Control point detector for hugin.
cpfind --multirow --celeste -o $1'/panorama.pto' $1'/panorama.pto'
# Remove all non-credible control points.
cpclean -o $1'/panorama.pto' $1'/panorama.pto'
# Find vertical lines and assigns vertical control points to them.
linefind -o $1'/panorama.pto' $1'/panorama.pto'
# Control point optimization.
autooptimiser -a -m -l -s -o $1'/panorama.pto' $1'/panorama.pto'
# Change some output options of the project file
pano_modify --canvas=AUTO --crop=AUTO -o $1'/panorama.pto' $1'/panorama.pto'
# Stitching panorama
hugin_executor --stitching --prefix=$1'/panorama.pto' $1'/panorama.pto'
# Convert .tif to .jpg
convert $1'/panorama.tif' $1'/panorama.jpg'

# Remove unwanted format (.tif and .pto) 
rm $1'/panorama.pto'
rm $1'/panorama.tif'

