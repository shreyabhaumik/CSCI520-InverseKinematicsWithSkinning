"""
python3 -m pip install opencv-python
"""

import os
import cv2

project_dir = '/Users/shreya/Documents/MSinCSatUSC/Spring-2020/CSCI-520-Computer-Animation-and-Simulation/Assignment 3/SHREYA_BHAUMIK_6796453186_HW3/Animations'
target_dir = project_dir #os.path.join(project_dir, 'screenshots') # A folder where you saved ppm files
save_dir = os.path.join(project_dir, 'FinalAnimations')          # A folder where you wish to store jpg files - first make the folder before running this file

for filename in os.listdir(target_dir):
	if '.ppm' not in filename: continue

	print('Converting', filename)
	img_src_path = os.path.join(target_dir, filename)
	img = cv2.imread(img_src_path)

	current_frame = str(int(filename[3:-4])).zfill(3) # 3 to remove "pic", -4 to remove extension, 3 to format XXX
	img_save_path = os.path.join(save_dir, current_frame + '.jpg')
	cv2.imwrite(img_save_path, img)

	# Delete the file
	os.remove(img_src_path)
