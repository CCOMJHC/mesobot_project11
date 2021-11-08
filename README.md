# Mesobot with Project11

This package provides status information from WHOI's Mesobot to the Project11 framework. It was developed using the `sonardyne_usbl` package to provide positioning and acoustic communications.

## mesobot_node

The `mesobot_node.py` script listens for position and status updates from USBL messages and publishes Project11 `Heartbeat` messages as well as positions updated with depth and heading.

### Status

Mesobot sends updates that include depth and heading using acoustic SMS messages. The node listens for these messages over the `sms` topic expecting messages of the type `sonardyne_msgs/SMS`. A `Heartbeat` messages is constructed from the components of the status message and published on the `project11/heartbeat` topic.

### Position

The node listens for `geographic_msgs/GeoPointStamped` messages over the `position` topic as well as the `backup_position` topic. The positions are augmented with heading and accurate depth information if such has been recieved from Mesobot within 5 minutes and replublished as `geographic_msgs/GeoPoseStamped` to the `nav/position` topic. Positions from the `backup_position` topic gets replublished if a position hasn't been recieved within 30 seconds from the `position` topic.
