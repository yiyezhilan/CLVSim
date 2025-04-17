"""
Author: Qingning Lan
This script is used to generate the video from the rendered images.

Input:
    - The rendered images.
    - The video name.
"""
import subprocess
import time


class Video:
    def __init__(self, img_folder, video_path):
        self.img_path = "./" + img_folder + "/%d.png"
        self.video_path = video_path

    def make_video(self):
        subprocess.run(
            [
                "ffmpeg",
                "-r",
                "25",
                "-i",
                self.img_path,
                "-preset",
                "medium",
                "-pix_fmt",
                "yuv420p",
                "-crf",
                "20",
                "-y",
                self.video_path,
            ]
        )
        print(self.video_path + " finished")
        time.sleep(1)


folders = ["Rendered_IMG_fix_cam", "Rendered_IMG_rot_cam"]
video_names = ["video_fix_cam.mp4", "video_rot_cam.mp4"]

for i in range(len(folders)):
    video = Video(folders[i], video_names[i])
    video.make_video()
