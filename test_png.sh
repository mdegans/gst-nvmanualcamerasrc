#!/bin/bash

export BUILDDIR="$PWD/builddir"
export GST_PLUGIN_PATH="$BUILDDIR/src"
export GST_DEBUG=4
export GST_DEBUG_DUMP_DOT_DIR="$PWD"

set -ex

ninja -C "$BUILDDIR"

gst-launch-1.0 nvmanualcamerasrc num_buffers=1 \
  gain=1 \
  digitalgain=1 \
  exposuretime=1 \
  ! "video/x-raw(memory:NVMM),width=4032,height=3040" \
  ! nvvidconv ! "video/x-raw, format=RGBA" \
  ! pngenc ! filesink location=out.png || true

rm -f *.pdf

for dotfile in *.dot ; do
  dot -Tpdf "$dotfile" > "$(basename "${dotfile/.dot}")".pdf || true
  rm "$dotfile"
done
