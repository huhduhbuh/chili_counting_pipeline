#!/bin/bash

# usage: bash visualize.sh [input_filename] [config used] [color space] [overlay=true] [gray=true]

input_filename=$1
config_used=$2
color_space=$3
overlay=true
gray=true

path="results/$1/$2/"

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
python3 overlay_cloud.py "input_clouds/$input_filename" "${path}/red_${color_space}.pcd" "$overlay" "$gray"
python3 overlay_cloud.py "input_clouds/$input_filename" "${path}/red_${color_space}_inliers.pcd" "$overlay" "$gray"
python3 overlay_cloud.py "input_clouds/$input_filename" "${path}/red_${color_space}_inliers_super.pcd" "$overlay" "$gray"
python3 overlay_cloud.py "input_clouds/$input_filename" "${path}/red_${color_space}_inliers_lccp0.pcd" "$overlay" "$gray"
python3 overlay_cloud.py "input_clouds/$input_filename" "${path}/red_${color_space}_inliers_final.pcd" "$overlay" "$gray"
