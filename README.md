# OpenJoint
OpenJoint for ignition-gazebo.

To enable the attachment, use ign topic -t "/model/vehicle_blue/open_joint/attach" -m ignition.msgs.Empty -p "unused: true"
To disable the attachment, use ign topic -t "/model/vehicle_blue/open_joint/detach" -m ignition.msgs.Empty -p "unused: true"
The contactTopic should be the same as the contact sensor's topic, find it in your SDF.
