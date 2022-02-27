#!/usr/bin/env python3

import speech_recognition as sr
import rospy
from std_msgs.msg import String, Float32


r = sr.Recognizer()
m = sr.Microphone()

def record_audio(pub_final, pub_confidence):
	with m as source: 
		r.adjust_for_ambient_noise(source) 
		r.dynamic_energy_threshold = True

		
		r.non_speaking_duration = 0.1
		#Must be equal or larger than non_speaking_duration
		r.pause_threshold = 0.1
		#Getting a faster reaction by considering 0.2s a phrase
		r.phrase_threshold = 0.2
		


		while not rospy.is_shutdown():
			audio = r.listen(source)
			try:
				voice_data = r.recognize_google(audio, language="fr-FR", show_all=False)
				pub_final.publish(voice_data)
				print(voice_data)

				'''
				transcript = voice_data['alternative'][0]['transcript']
				confidence = voice_data['alternative'][0]['confidence']
				print(transcript, confidence)
				pub_final.publish(transcript)
				pub_confidence.publish(confidence)
				'''


			except sr.UnknownValueError :
				print("noise")

			except sr.RequestError :
				print("Request error. Are you connected to internet?")
			
            
       

			



def main():
	rospy.init_node('Verity', anonymous=True)
	pub_final = rospy.Publisher('speech_recognition/final_result',String, queue_size=10)
	pub_confidence = rospy.Publisher('speech_recognition/confidence',Float32, queue_size=10)

	record_audio(pub_final, pub_confidence)

	'''
	while not rospy.is_shutdown():
		record_audio(pub_final, pub_confidence)
	'''


if __name__ == '__main__':
    main()


		



