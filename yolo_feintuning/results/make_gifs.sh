#!/bin/bash

epochs=(10 25 50 75 100 150 200)

plots=("BoxF1_curve.png" "confusion_matrix_normalized.png" "results.png")

outdir="gifs"
mkdir -p "$outdir"

for plot in "${plots[@]}"; do
    inputs=()
    filter=""
    idx=0
    for epoch in "${epochs[@]}"; do
        file="feinTuned_yolov8n_${epoch}_epochs/${plot}"
        inputs+=("-loop" "1" "-t" "2" "-i" "$file")
        filter="$filter [$idx:v]drawtext=text='${epoch} epochs':x=(w-text_w)/2:y=20:fontsize=60:fontcolor=blue:box=1:boxcolor=black@0.5[frame$idx];"
        idx=$((idx+1))
    done

    concat_inputs=$(for i in $(seq 0 $((idx-1))); do echo -n "[frame$i]"; done)
    filter="$filter ${concat_inputs}concat=n=${idx}:v=1:a=0,format=yuv420p[out]"

    ffmpeg "${inputs[@]}" -filter_complex "$filter" -map "[out]" -loop 0 "$outdir/${plot%.png}.gif"
done
