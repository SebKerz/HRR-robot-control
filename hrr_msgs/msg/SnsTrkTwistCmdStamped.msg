# as the Sensor tracking interface may not clearly state the reference frame
# that is used for commands, this message circumvents by allowing to 
# add a rotation center to the command, relative to the current TCP-center
# This removes TF-broadcaster to be available and allows to purely rely on the internal readouts
Header header
hrr_msgs/SnsTrkTwistCmd sns_trk_twist