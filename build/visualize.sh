#!/bin/bash

input_filename=$1
color_space=$2

# Call Python script
python3 overlay_cloud.py "$input_filename" "red_${color_space}.pcd"
python3 overlay_cloud.py "$input_filename" "red_${color_space}_inliers.pcd"
python3 overlay_cloud.py "$input_filename" "red_${color_space}_inliers_super.pcd"
python3 overlay_cloud.py "$input_filename" "red_${color_space}_inliers_lccp0.pcd"
python3 overlay_cloud.py "$input_filename" "red_${color_space}_inliers_final.pcd"
