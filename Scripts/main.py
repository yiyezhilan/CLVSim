"""
Author: Qingning Lan
This script is used to render all the images of the rover and the SPH fluid using Blender.

Input:
    - The total number of frames.
"""
import subprocess
import time
import logging
from datetime import datetime

# 设置日志记录
logging.basicConfig(filename='blender_render.log', level=logging.INFO)

# 记录脚本开始的时间
start_time = datetime.now()
logging.info(f"Script started at {start_time}")

# Blender 的路径
blender_path = "D:\\Blender Foundation\\blender-3.6.7-windows-x64\\blender.exe"
# blender_path = "D:\\Blender Foundation\\K-Cycles_2023\\blender.exe"

# Python 脚本的路径
script_path = "./render.py"

# 循环参数和超时时间（以秒为单位）
outer_loops = 1
inner_loops = 526   # change this to the total number of frames
timeout_seconds = 360  # 例如，设置为6min

try:
    for j in range(1, outer_loops + 1):
        logging.info(f"Starting outer loop number: {j}")
        print(f"Starting outer loop number: {j}")

        for i in range(inner_loops):
            logging.info(f"Starting render of loop {j}: {i}")
            print(f"Starting render of loop {j}: {i}")
            
            try:
                # 调用 Blender 进行渲染，并设置超时时间
                subprocess.run([blender_path, "--background", "--python", script_path, str(i)], check=True, timeout=timeout_seconds)
                logging.info(f"Finished render: {i}")
                print(f"Finished render: {i}")
            except subprocess.TimeoutExpired:
                logging.error(f"Render timeout for loop {j}: {i}")
                print(f"Render timeout for loop {j}: {i}")
                current_time = datetime.now()
                print(f"Render timeout in: {current_time}")
                # 在这里添加超时后的处理逻辑
            except subprocess.CalledProcessError as e:
                logging.error(f"Error in render: {i}, with error: {e}")
                print(f"Error in render: {i}, with error: {e}")
                current_time = datetime.now()
                print(f"Error in render in: {current_time}")
                # 可以在这里添加额外的错误处理逻辑

            # time.sleep(1)

        logging.info(f"Finished outer loop number: {j}")
        print(f"Finished outer loop number: {j}")

    logging.info("All loops finished")
    print("All loops finished")
except Exception as e:
    logging.error(f"Unexpected error: {e}")
    print(f"Unexpected error: {e}")
    current_time = datetime.now()
    print(f"Unexpected error in: {current_time}")

# 记录脚本结束的时间
end_time = datetime.now()
logging.info(f"Script ended at {end_time}")
