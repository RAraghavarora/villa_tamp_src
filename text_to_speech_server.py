#!/usr/bin/python
from kokoro import KPipeline
import rospy
import actionlib
from io import BytesIO
import os
import pygame
from threading import Lock
import soundfile as sf
import numpy as np

from tmc_msgs.msg import Voice, TalkRequestAction, TalkRequestGoal
from std_msgs.msg import String, UInt32


class KokoroTTSServer:
    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.ActionServer(self._action_name, TalkRequestAction,
                                          goal_cb=self.goal_cb, cancel_cb=self.cancel_cb, auto_start=False)
        self.goal_handle_list = []
        self.active_goal_id = None
        self.preempted_id = None

        self.lock = Lock()
        
        # Initialize Kokoro TTS pipeline
        # 'a' for American English, 'b' for British English
        self.pipeline = KPipeline(lang_code='a')
        
        pygame.init()
        pygame.mixer.init(frequency=24000)  # Kokoro outputs at 24kHz
        self._as.start()
        print("Started Kokoro TTS server")

    def goal_cb(self, gh):
        with self.lock:
            self.goal_handle_list.append(gh)

    def cancel_cb(self, gh):
        with self.lock:
            for g in self.goal_handle_list:
                if gh.get_goal_id() == g.get_goal_id():
                    self.goal_handle_list.remove(g)
            if gh.get_goal_id() == self.active_goal_id:
                self.preempted_id = self.active_goal_id

    def execute(self):
        with self.lock:
            if len(self.goal_handle_list) > 0:
                gh = self.goal_handle_list.pop(0)
                gh.set_accepted()
                goal = gh.get_goal()
                self.active_goal_id = gh.get_goal_id()
            else:
                return

        say = goal.data.sentence
        rospy.loginfo('%s: %s' % (self._action_name, say))
        
        # Process text through Kokoro TTS
        audio_data = None
        temp_file = "/tmp/kokoro_speech.wav"
        
        # Generate speech using Kokoro
        generator = self.pipeline(say, voice='af_heart')  # Using the default voice
        
        # Process the generator (it yields tuples like (gs, ps, audio))
        for _, _, audio in generator:
            if audio_data is None:
                audio_data = audio
            else:
                # Concatenate audio segments
                audio_data = np.concatenate((audio_data, audio))
        
        # Save to temporary file
        sf.write(temp_file, audio_data, 24000)
        
        # Play using pygame
        pygame.mixer.music.load(temp_file)
        pygame.mixer.music.play()

        while pygame.mixer.music.get_busy():
            if self.preempted_id == self.active_goal_id:
                rospy.loginfo('%s: Preempted' % self._action_name)
                pygame.mixer.music.stop()
                gh.set_canceled()
                return
            rospy.sleep(0.1)

        gh.set_succeeded()


if __name__ == '__main__':
    rospy.init_node('kokoro_tts_action')
    server = KokoroTTSServer(rospy.get_name())
    while not rospy.is_shutdown():
        server.execute()
        rospy.sleep(0.1)
