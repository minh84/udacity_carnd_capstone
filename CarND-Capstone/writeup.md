[//]: # (Image References)
[reduce_speed]: ./media/reduce_speed.png

This is the final project of the Udacity Self-Dirving Car Nanodegree, we fork the starter code from [here](https://github.com/udacity/CarND-Capstone). The goal of the project is to implement a real self-driving car which consists three main parts:
* **Perception**: detect obstacle and traffic light
* **Planning**: plan aheah trajectory (including position, speed and direction)
* **Control**: implement a PID control to archive speed/direction

# Individual Submission
Name    | Udacity Account Email
------  | -------------------
Minh VU | vungocminh84@gmail.com

## Perception
This node (implemented in `tl_detector`) is used to detect the traffic light state (i.e Red/Yellow/Green/NoTrafficLight). We use [Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection) for this task.

We feel that this is the most time consuming task. Moreover, since Udacity reviewers only support Tensorflow 1.3.0, we decided to adopt an ad-hoc solution (taken from Slack's Forum)

* we train *Object Detection model*  using [Object Detection API](https://github.com/tensorflow/models/tree/master/research/object_detection) with Tensorflow 1.8.0: this task is done in this [repository](https://github.com/minh84/udacity_carnd_traffic_light)
* we export model for inference using Tensorflow 1.3.0: this task is done in this [repository](https://github.com/minh84/carnd_capstone_traffic_light)

We experiment with the three following models (specified in `sim_traffic_light_config.yaml` and `site_traffic_light_config.yaml` in field `frozen_model_graph`)

* `ssd_mobilenet_v1_coco`
* `ssd_inception_v2_coco`
* `faster_rcnn_resnet101_coco`

Few notes on training for the traffic-light detection

* the form of the traffic-light has the aspect ratio width/heigh ~ 1/3, so in the pipeline-config we only consider the `aspect-ratio=0.3333`
* to increse the inference speed we reduce the number of proposing boxes to 10

Note that, we expect that this node is run in less 500ms per cycle (since it uses `rospy.spin`), so the inference time must be less than 500ms to be used in real-time traffic light detection. The inference and accuracy of the three models are


* **With CPU only**
For sim dataset

|  metric    | ssd_mobilenet_v1_coco | ssd_inception_v2_coco | faster_rcnn_resnet101_coco |
| ---: | :---: | :---: | :---: |
| accuracy | 97.47% | 97.11%  | 98.92% |
| min runtime | 64  | 100     | 1554 |
| max runtime | 90  | 149     | 1932 |
| mean runtime | 69 | 110     | 1598 |

And for real dataset

|  metric    | ssd_mobilenet_v1_coco | ssd_inception_v2_coco | faster_rcnn_resnet101_coco |
| ---: | :---: | :---: | :---: |
| accuracy | 98.11% | 96.86%  | 100% |
| min runtime | 71  | 107     | 4566 |
| max runtime | 93  | 135     | 5200 |
| mean runtime | 82 | 118     | 4660 |

* **With GPU**

For sim dataset

|  metric    | ssd_mobilenet_v1_coco | ssd_inception_v2_coco | faster_rcnn_resnet101_coco |
| ---: | :---: | :---: | :---: |
| accuracy | 97.47% | 97.11%  | 98.92% |
| min runtime | 10  | 14     | 51 |
| max runtime | 25  | 29     | 70 |
| mean runtime | 12 | 16     | 56 |

And for real dataset

|  metric    | ssd_mobilenet_v1_coco | ssd_inception_v2_coco | faster_rcnn_resnet101_coco |
| ---: | :---: | :---: | :---: |
| accuracy | 98.11% | 96.86%  | 100% |
| min runtime | 11  | 15     | 108 |
| max runtime | 20  | 30     | 124 |
| mean runtime | 13 | 17     | 114 | 

So look at above result  

* without GPU: we should use `ssd_mobilenet_v1_coco`
* using GPU: both three models can be used and `faster_rcnn_resnet101_coco` is the most stable.

Note that to reduce the CPU/GPU usage, we only turn on the traffic-line detector when the car is close enough to the traffic-line (see `tl_detector.process_traffic_lights` for more detail).

## Planning
This node (implemented in `waypoint_updater`) receive data from

* `/current_pose`: which gives the car's current position
* `/traffic_waypoint`: which gives the car's next red-line
* `/base_waypoints`: which gives the car's global waypoints.

This node need to produce a planned trajectory which is a list of waypoints with corresponding speed in each point.

The main logic is follow:
* if there is no red-line: we try to drive a the maximum-speed (defined in `~/waypoint_loader/velocity`)
* if there is a red-line ahead: we try to reduce the speed as shown in the lecture
 ![alt-text][reduce_speed]
 this is implemented in `waypoint_update.decelerate_waypoints`. The implementation can be briefly explained as following, suppose dv/dt = a, loop back in time (t means time to present so that we have v(0)=0)
 * v(t) = a * t
 * s(t) = a * t^2/2 
 So we have v(t) = sqrt(2 * a * s(t)).
 
The walk-through for `waypoint_updater` is well explained the lecture so its implementation is straight forward. However, one crucial point is to control the rate of publishing planned trajectory, we find that the suggested value **50Hz (in the lecture) is too high** that makes the system becomes too slow to run. After testing we find that 10Hz is more reasonable.

## Control
This node (implemented in `twist_controller`) is used to compute `throttle, brake, steering` to be sent to the drive-by-wire node. It receives data from

* `/twist_cmd`: which publishes target linear/angular velocities 
* `/current_velocity`: which publishes current linear velocity

We need to compute `throttle, brake, steering` to achive target linear/angular velocities. This is done using a **PID** control (implemented in `twist_controller.py`).

## Few notes on ROS
ROS is great tool in Robotic R&D, here is a few notes that we learnt from doing this project

* when we subcribed to a channel, the callback might be triggered before the constructor finished to run: so in the constructor, it's better to define all member-variables before subcribing to any channel.
* there are two ways to do the spin in rospy
    * [rospy.spin](https://github.com/ros/ros_comm/blob/kinetic-devel/clients/rospy/src/rospy/client.py#L118): is a loop with 0.5 second sleep.
    
    * [rospy.Rate(RATE)](https://github.com/ros/ros_comm/blob/kinetic-devel/clients/rospy/src/rospy/timer.py): is a custom loop where we can control the frequency via input `RATE` (50 means 50 times per second).  

When testing the real ROS-bag, we need to install `jsk_visualization` via
```bash
sudo apt install ros-kinetic-jsk-visualization
``` 

## Conclusion
Going through this project we learn how to implement a real self-driving car which applied what we learnt from Term 1 through to Term 3. This is a great experience to start into the Self-Car-Driving field. There is so much more to do after this for example one can train model on Bosch traffic light data or experiment it with a car-kit.
 

## Attributions
We learnt a great deal from the following blogs
- [becominghuman blog](https://becominghuman.ai/traffic-light-detection-tensorflow-api-c75fdbadac62)
- [anthony_sarkis blog](https://medium.com/@anthony_sarkis/self-driving-cars-implementing-real-time-traffic-light-detection-and-classification-in-2017-7d9ae8df1c58)