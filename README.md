# ROS2 BAG to image

## Topic to Image

Subscribes to a ROS2 Image topic and stores received messages to PNG files.

How to launch:
```
ros2 launch ros2_bag_to_image topic_to_image.xml \
    input/topic:=/camera/image_rect compressed:=True
```

The images are exported in lossless PNG format.

Set `compressed` to `True` to subscribe to a compressed image_transport.

### Parameters
| Parameter       | Default      | Description                        |
|-----------------|--------------|------------------------------------|
| `input/topics`  | `/image_raw` | ROS2 Topic to subscribe            |
| `output/path`   | `/tmp/`      | Path where to store the PNG files. |
| `output/prefix` |              | Text to prepend to the output file |

## BAG To Image

Opens, reads, extracts and saves `sensor_msgs/msg/Image` or `sensor_msgs/msg/CompressedImages` from Ros2 bag.

Stores all the topics in the defined output path, names the images as:
`topic_name_stampsecs_stampnsecs.png`

The images are exported in lossless PNG format.

How to launch:
```bash
$ ros2 launch ros2_bag_to_image bag_to_image.xml \
        input/path:=/PATH_TO/input_bag/ \
        input/topics:="['camera1/image_rawcompressed', 'camera2/image_raw']"
```

### Parameters
| Parameter              | Default   | Description                                                         |
|------------------------|-----------|---------------------------------------------------------------------|
| `input/path`           |           | Input Rosbag Path                                                   |
| `input/bag_format`     | `cdr`     | Path where to store the PNG files.                                  |
| `input/bag_storage_id` | `sqlite3` | Text to prepend to the output file                                  |
| `input/topics`         |           | List of Input Topics in the form `"['topic1', 'topic2', 'topicN']"` |
| `output/path`          | `/tmp/`   | Output path PNG files                                               |
