#!/bin/bash

in_dir=../datasets/traffic_count_data/xls_data
out_dir=../datasets/traffic_count_data/csv_data

for fname in $in_dir/*.xls; do
    echo $fname
    prefix=${fname%*.xls}
    prefix=${prefix#$in_dir/}
    ssconvert -S $fname $out_dir/$prefix.csv
done
