#!/bin/bash

input_filename=$1
color_space=$2
overlay=true
gray=true

# Parse optional arguments
for arg in "$@"; do
    case $arg in
        overlay=*)
            overlay="${arg#*=}"
            ;;
        gray=*)
            gray="${arg#*=}"
            ;;
    esac
done

# Call Python script
python3 overlay_cloud.py "$input_filename" "red_${color_space}.pcd" "$overlay" "$gray"
python3 overlay_cloud.py "$input_filename" "red_${color_space}_inliers.pcd" "$overlay" "$gray"
python3 overlay_cloud.py "$input_filename" "red_${color_space}_inliers_super.pcd" "$overlay" "$gray"
python3 overlay_cloud.py "$input_filename" "red_${color_space}_inliers_lccp0.pcd" "$overlay" "$gray"
python3 overlay_cloud.py "$input_filename" "red_${color_space}_inliers_final.pcd" "$overlay" "$gray"
