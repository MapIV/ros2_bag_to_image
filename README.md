# ROS2 BAG to image

## Topic to Image

Subscribes to a ROS2 Image topic and stores received messages to PNG files.

How to launch:
```
ros2 launch ros2_bag_to_image topic_to_image.xml \
    input/topic:=/camera/image_rect compressed:=True
```

Set `compressed` to `True` to subscribe to a compressed image_transport.

### Parameters
| Parameter       | Default      | Description                        |
|-----------------|--------------|------------------------------------|
| `input/topic`   | `/image_raw` | ROS2 Topic to subscribe            |
| `output/path`   | `/tmp/`      | Path where to store the PCD files. |
| `output/prefix` |              | Text to prepend to the output file |

## BAG To Image