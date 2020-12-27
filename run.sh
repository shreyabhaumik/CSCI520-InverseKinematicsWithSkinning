# cd armadillo
# cd dragon
# cd hand
cd star
# after filename-'skin.config', next argument is for choice in IK algorithm, and the one after that is for choice in skinning method to be used
# IK algorithm		- 0(Tikhonov IK method),	1(Pseudoinverse method),	2(Transpose method)
# Skinning method	- 0(Linear Blend Skinning),	1(Dual Quaternion Skinning)
# by default, both are set to 0
# Have tweaked IKjointIDs and made a 'changedskin.config' file for each of the three models that came with the startup code.
../driver skin.config 0 1
#../driver changedskin.config 0 0