#!/bin/bash

input_filename=$1
config_file="config.json"
out_folder="default"


rgb_output=$(python3 get_red_rgb.py "$input_filename" "$config_file")
echo "RGB output file: $rgb_output"

hsv_output=$(python3 get_red_hsv.py "$input_filename" "$config_file")
echo "HSV output file: $hsv_output"

lab_output=$(python3 get_red_lab.py "$input_filename" "$config_file")
echo "Lab output file: $lab_output"


# STATISTICAL OUTLIER REMOVAL
rgb_output_SO=$(./statistical_removal "$rgb_output")
echo "RGB after statistical removal: $rgb_output_SO"

hsv_output_SO=$(./statistical_removal "$hsv_output")
echo "HSV after statistical removal: $hsv_output_SO"

lab_output_SO=$(./statistical_removal "$lab_output")
echo "Lab after statistical removal: $lab_output_SO"


# COUNTING 

count_rgb=$(./counting "$rgb_output_SO" "$config_file")
echo "Final count for RGB: $count_rgb"

count_hsv=$(./counting "$hsv_output_SO" "$config_file")
echo "Final count for HSV: $count_hsv"

count_lab=$(./counting "$lab_output_SO" "$config_file")
echo "Final count for Lab: $count_lab"




