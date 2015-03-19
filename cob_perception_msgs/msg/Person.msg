#This message is used to describe a person
 
string              name				#id of a person
string              detector			#used device for detection (scan, camera)
geometry_msgs/Pose  position			#position in space
geometry_msgs/Point velocity			#speed

cob_perception_msgs/Skeleton skeleton	#assigned skeleton to a tracked person