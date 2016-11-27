@echo off
ffmpeg.exe -y -r 30 -f image2 -s 1920x1080 -i frame%%04d.png -vcodec libx264 -crf 25  -pix_fmt yuv420p "prrt.mp4"
