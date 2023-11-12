# Decaying Cloud Aggregator

## Description

This package contains a node that aggregates pointclouds from multiple topics into a single pointcloud. The pointclouds
are aggregated by transforming them into a common frame and then merging them together. The pointclouds can be
downsampled using a voxel filter and cropped to a certain distance from the sensor. The pointclouds are aggregated using
a circular buffer, which means that the pointclouds are only kept for a certain amount of time. Finally, the aggregated
pointcloud is published in the specified target frame.

An example of how to launch the decaying cloud aggregator is shown in `launch/decaying_aggregator.launch`.

## Parameters

In `config/decaying_aggregator_config.yaml` the following parameters can be set:

* **`in_topics`** A list of topics of the pointclouds to aggregate.
* **`crop_distances`** A list of distances to crop the pointclouds to (negative values mean no cropping).
* **`subscriber_queue_sizes`** A list of queue sizes of the subscribers.
* **`out_topic`** The topic where to publish the aggregated pointcloud.
* **`target_frame`** The target frame in which to aggregate the pointclouds.
* **`publish_frame`** The frame to publish the aggregated pointcloud in.
* **`max_queue_size`** The maximum queue size of the aggregator.
* **`voxel_filter_size`** The voxel filter size to downsample the aggregated pointcloud to.
