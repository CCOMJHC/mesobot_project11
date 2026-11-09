# Mesobot with Project11

This package provides status information from WHOI's Mesobot to the Project11 framework. It was developed using the `sonardyne_usbl` package to provide positioning and acoustic communications.

## mesobot_node

The `mesobot_node.py` script listens for position and status updates from USBL messages and publishes Project11 `Heartbeat` messages as well as positions updated with depth and heading.

### Status

Mesobot sends updates that include depth and heading using acoustic SMS messages. The node listens for these messages over the `sms` topic expecting messages of the type `sonardyne_msgs/SMS`. A `Heartbeat` messages is constructed from the components of the status message and published on the `project11/heartbeat` topic.

### Position

The node listens for `geographic_msgs/GeoPointStamped` messages over the `position` topic as well as the `backup_position` topic. The positions are augmented with heading and accurate depth information if such has been recieved from Mesobot within 5 minutes and replublished as `geographic_msgs/GeoPoseStamped` to the `nav/position` topic. Positions from the `backup_position` topic gets replublished if a position hasn't been recieved within 30 seconds from the `position` topic.

## follow_layer

The `follow_layer.py` script closes the loop between a sound scattering layer
tracked in DriX's echosounder data and Mesobot's depth. It was written for the
NA155 expedition (E/V Nautilus, October 2023) and used on Mesobot deployment
MB088 to keep Mesobot inside a scattering layer while it filtered eDNA
samples, with DriX loitering above and the EK80 data processed in real time
by the
[`watercolumn_layer_tracker`](https://github.com/rolker/watercolumn_layer_tracker)
package.

| | Topic | Type |
|---|---|---|
| Subscribes | `.../es70_18cd_tracker/layer` | `project11_msgs/AcousticLayer` (tracked layer from `single_layer_tracker`) |
| Subscribes | `/project11/mesobot/depth` | `std_msgs/Float32` (Mesobot's last reported depth) |
| Publishes | `/project11/drix_8/usbl_modem/send_raw` | `std_msgs/String` (raw acoustic SMS to Mesobot) |

Behavior:

- At most once every 55 s, the latest tracked layer depth is sent to Mesobot
  as a depth goal using Mesobot's mission-executive directive format,
  `GOAL1 2 <depth> <speed>; WAIT 1800` (degree of freedom 2 is depth).
- The commanded vertical speed is proportional to the difference between the
  new target and Mesobot's last reported depth: 0.25 m/s for an offset of
  20 m or more, scaling down linearly to a floor of 0.005 m/s as the vehicle
  settles on the layer. If no depth report has arrived in the last 90 s the
  floor speed is used.
- Topic names are hard-coded for the NA155 configuration (DriX 8, EK80
  channel `es70_18cd`).

The tag `na155-mb088` marks the revision used on deployment MB088
(2023-10-16). The method is described in section 2.7 of Govindarajan et al.
(NA155 midwater eDNA paper, in preparation; citation to be updated on
publication).

## License

BSD 3-Clause; see [LICENSE](LICENSE).
