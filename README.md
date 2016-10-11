# MarsArmTouchLocalization
Cpp code running on the Mars arm for tactile localization


Note: The kinematics config file does not appear to be working. This should be debugged, as a kinematics config file is the right way to change the kinematics. Temporarily, you can edit
boeing/software/src/Components/Controllers/CoordinatedController/arm/
config_TRACLABS.cc 
to change the dh parameters
