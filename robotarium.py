import os
import cv2
from datetime import datetime
import math
import moviepy.video.io.ImageSequenceClip
import numpy as np
from PIL import Image
import re
import shutil
import time

from robotarium_abc import *


class Robotarium(RobotariumABC):

        def __init__(self, number_of_robots=-1, show_figure=True, sim_in_real_time = True, initial_conditions=np.array([])):
            super().__init__(number_of_robots, show_figure, sim_in_real_time, initial_conditions)

            #Initialize some rendering variables
            self.previous_render_time = time.time()
            self.sim_in_real_time = sim_in_real_time

            #Initialize checks for step and get poses calls
            self._called_step_already = True
            self._checked_poses_already = False

            #Initialization of error collection.
            self._errors = {}

            #Initialize steps
            self._iterations = 0 

            # set frames per seconds for the generated video from the simulation
            self.fps = 30

            # create a folder to store generated images from the simulation
            self.timestamp = int(time.time())
            self.image_folder = 'static/images/' + str(self.timestamp)
            os.mkdir(self.image_folder)

            # specify which method to use to generate video: moviepy or opencv
            self.video_method = "opencv" # default, could make it an input parameter
            self.video_root_path = "static/uploads"
            self.video_path = "static/uploads/" + str(self.timestamp)

        def get_poses(self):
            """
            Returns the states of the agents.

            -> 3xN numpy array (of robot poses)
            """

            assert(not self._checked_poses_already), "Can only call get_poses() once per call of step()."
            # Allow step() to be called again.
            self._called_step_already = False
            self._checked_poses_already = True 

            return self.poses

        def generate_video_moviepy(self):
            """
            Create the simulation video using moviepy from the images generated.

            """

            images = [os.path.join(self.image_folder, img)
               for img in os.listdir(self.image_folder)
               if img.endswith(".png")]
            
            try:
                images = sorted(images, key=lambda x: (int(re.sub('\D', '', x)), x))
                clip = moviepy.video.io.ImageSequenceClip.ImageSequenceClip(images, fps=self.fps)
                clip.write_videofile(self.video_path + '.mp4')
            except:
                error = "ERROR: Failed to create simulation video from images at " + self.image_folder
                print(error)
                print("Exit the simulation process now")
                
                return error

            return "Done"

        def generate_video_opencv(self):
            """
            Create the simulation video using opencv from the images generated.

            """

            data = [os.path.join(self.image_folder, img)
               for img in os.listdir(self.image_folder)
               if img.endswith(".png")]
            
            try:
                data = sorted(data, key=lambda x: (int(re.sub('\D', '', x)), x))
                videodims = (Image.open(data[0]).size)
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                video = cv2.VideoWriter(self.video_path + '.mp4', fourcc, self.fps, videodims)

                for fidx, f in enumerate(data):
                    print( f'Done: {round(fidx * 100 / len(data), 1)} % - {f}', end="\r")
                    img = Image.open(f)
                    video.write(cv2.cvtColor(np.array(img), cv2.COLOR_RGB2BGR))

                video.release()
            except:
                error = "ERROR: Failed to create simulation video from images at " + self.image_folder
                print(error)
                print("Exit the simulation process now")
                
                return error

            return "Done"

        def call_at_scripts_end(self):
            """
            Call this function at the end of scripts to display potentail errors.  
            Even if you don't want to print the errors, calling this function at the
            end of your script will enable execution on the Robotarium testbed.

            """

            return_message = "Done"

            # generate the simulation video
            if self.video_method == "moviepy":
                status = self.generate_video_moviepy()
            else:
                status = self.generate_video_opencv()

            # exit if video generation status has error
            if "ERROR" in status:
                return_message = status

                return return_message
            
            # delete the image folder after the video is generated
            shutil.rmtree(self.image_folder)
            
            # store timestamp to text for a record
            with open('static/text/timestamp.txt', 'w') as f:
                f.truncate(0)
                f.write(str(self.timestamp))
            
            # delete the oldest video being generated 
            #count = 0
            
            # TODO: delete the old files by dates instead say older than 30 days
            # TODO: run the code as a separate function by crontab nightly say
            #for path in os.listdir(self.video_root_path):
            #    # check if current path is a file
            #    if os.path.isfile(os.path.join(self.video_root_path, path)):
            #        count += 1

            #smallestVideoInt = float('inf')
            #if(count > 25):
            #    videoFileList = os.listdir(self.video_root_path)
            #    for video in videoFileList:
            #        if(video != '.keep'):
            #            intStr = video[0:-4]
            #            videoInt = int(intStr)
            #            if videoInt < smallestVideoInt:
            #                smallestVideoInt = videoInt
            #   
            #    os.remove(self.video_root_path + '/' + str(smallestVideoInt) + '.mp4')
            
            video_filepath = self.video_path + '.mp4'
            print('\n' + 'No errors in your simulation! Acceptance of your experiment is likely!')
            print('The generated video from the simulation is located at:', video_filepath)
            print('Your simulation will take approximately {0} real seconds when deployed on the Robotarium. \n'.format(math.ceil(self._iterations*0.033)))

            return return_message

        def step(self):
            """
            Increments the simulation by updating the dynamics.

            """
            assert(not self._called_step_already), "Make sure to call get_poses before calling step() again."

            # Allow get_poses function to be called again.
            self._called_step_already = True
            self._checked_poses_already = False

            # Validate before thresholding velocities
            self._errors = self._validate()

            # skip rest of the code if there's error found
            if self._errors:
                return self._errors

            self._iterations += 1

            # Update dynamics of agents
            self.poses[0, :] = self.poses[0, :] + self.time_step*np.cos(self.poses[2,:])*self.velocities[0, :]
            self.poses[1, :] = self.poses[1, :] + self.time_step*np.sin(self.poses[2,:])*self.velocities[0, :]
            self.poses[2, :] = self.poses[2, :] + self.time_step*self.velocities[1, :]
            # Ensure angles are wrapped
            self.poses[2, :] = np.arctan2(np.sin(self.poses[2, :]), np.cos(self.poses[2, :]))

            # Update graphics
            if(self.show_figure):
                if(self.sim_in_real_time):
                    t = time.time()
                    while(t - self.previous_render_time < self.time_step):
                        t=time.time()
                    self.previous_render_time = t

                for i in range(self.number_of_robots):
                    self.chassis_patches[i].center = self.poses[:2, i]
                    self.chassis_patches[i].orientation = self.poses[2, i] + math.pi/4

                    self.right_wheel_patches[i].center = self.poses[:2, i]+self.robot_radius*np.array((np.cos(self.poses[2, i]+math.pi/2), np.sin(self.poses[2, i]+math.pi/2)))+\
                                            0.04*np.array((-np.sin(self.poses[2, i]+math.pi/2), np.cos(self.poses[2, i]+math.pi/2)))
                    self.right_wheel_patches[i].orientation = self.poses[2, i] + math.pi/4

                    self.left_wheel_patches[i].center = self.poses[:2, i]+self.robot_radius*np.array((np.cos(self.poses[2, i]-math.pi/2), np.sin(self.poses[2, i]-math.pi/2)))+\
                                            0.04*np.array((-np.sin(self.poses[2, i]+math.pi/2), np.cos(self.poses[2, i]+math.pi/2)))
                    self.left_wheel_patches[i].orientation = self.poses[2,i] + math.pi/4
                    
                    self.right_led_patches[i].center = self.poses[:2, i]+0.75*self.robot_radius*np.array((np.cos(self.poses[2,i]), np.sin(self.poses[2,i])))-\
                                    0.04*np.array((-np.sin(self.poses[2, i]), np.cos(self.poses[2, i])))
                    self.left_led_patches[i].center = self.poses[:2, i]+0.75*self.robot_radius*np.array((np.cos(self.poses[2,i]), np.sin(self.poses[2,i])))-\
                                    0.015*np.array((-np.sin(self.poses[2, i]), np.cos(self.poses[2, i])))


                self.figure.savefig("static/images/{0}/{1}".format(self.timestamp, self._iterations))
                #self.figure.canvas.flush_events()

            return self._errors

