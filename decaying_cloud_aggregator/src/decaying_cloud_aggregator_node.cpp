//=================================================================================================
// MIT License
//
// Copyright (c) 2023 Simon Giegerich, TU Darmstadt
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//=================================================================================================
// based on:
//=================================================================================================
// Copyright (c) 2023, Stefan Kohlbrecher, TU Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Simulation, Systems Optimization and Robotics
//       group, TU Darmstadt nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

#include <ros/ros.h>
#include <laser_geometry/laser_geometry.h>

#include <tf/transform_listener.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl_ros/transforms.h>


class DecayingCloudAggregator
{
public:
  DecayingCloudAggregator()
  {
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_( "decaying_aggregator" );

    if ( !pnh_.getParam( "in_topics", in_topics_xml_ ))
    {
      ROS_ERROR( "[DecayingCloudAggregator] Could not get param \"in_topics\" from param server" );
      return;
    }
    if ( !pnh_.getParam( "crop_distances", crop_distances_xml_ ))
    {
      ROS_ERROR( "[DecayingCloudAggregator] Could not get param \"crop_distances\" from param server" );
      return;
    }
    if ( !pnh_.getParam( "subscriber_queue_sizes", subscriber_queue_sizes_ ))
    {
      ROS_ERROR( "[DecayingCloudAggregator] Could not get param \"subscriber_queue_sizes\" from param server" );
      return;
    }

    if ( in_topics_xml_.getType() != XmlRpc::XmlRpcValue::TypeArray ||
         crop_distances_xml_.getType() != XmlRpc::XmlRpcValue::TypeArray ||
         subscriber_queue_sizes_.getType() != XmlRpc::XmlRpcValue::TypeArray ||
         in_topics_xml_.size() != crop_distances_xml_.size() ||
         in_topics_xml_.size() != subscriber_queue_sizes_.size() ||
         in_topics_xml_.size() == 0 )
    {
      ROS_ERROR(
        "[DecayingCloudAggregator] \"in_topics\", \"crop_distances\" and \"subscriber_queue_sizes\" must be non-empty arrays of equal size!" );
      return;
    }

    for ( int i = 0; i < in_topics_xml_.size(); ++i )
    {
      if ( in_topics_xml_[i].getType() != XmlRpc::XmlRpcValue::TypeString )
      {
        ROS_ERROR( "[DecayingCloudAggregator] \"in_topics\" entry %d is not a string", i );
        return;
      }
      if ( crop_distances_xml_[i].getType() != XmlRpc::XmlRpcValue::TypeDouble )
      {
        ROS_ERROR( "[DecayingCloudAggregator] \"crop_distances\" entry %d is not a double", i );
        return;
      }
      if ( subscriber_queue_sizes_[i].getType() != XmlRpc::XmlRpcValue::TypeInt )
      {
        ROS_ERROR( "[DecayingCloudAggregator] \"subscriber_queue_sizes\" entry %d is not an int", i );
        return;
      }
      in_topics_.emplace_back( in_topics_xml_[i] );
      auto crop_distance = static_cast<float>( static_cast<double>( crop_distances_xml_[i] ));
      // If the crop distance is negative, we don't crop => we set the crop distance to -1.0f as otherwise we would lose the sign
      squared_crop_distances_.push_back( crop_distance < 0.0f ? -1.0f : crop_distance * crop_distance );
    }

    if ( !pnh_.getParam( "out_topic", out_topic_ ))
    {
      ROS_ERROR( "[DecayingCloudAggregator] Could not get param \"out_topic\" from param server" );
      return;
    }

    if ( !pnh_.getParam( "target_frame", target_frame_ ))
    {
      ROS_ERROR( "[DecayingCloudAggregator] Could not get param \"target_frame\" from param server" );
      return;
    }

    pnh_.param( "publish_frame", publish_frame_, std::string( "" ));

    if ( !pnh_.getParam( "max_queue_size", max_queue_size_ ))
    {
      ROS_ERROR( "[DecayingCloudAggregator] Could not get param \"max_queue_size\" from param server" );
      return;
    }

    if ( !pnh_.getParam( "voxel_filter_size", voxel_filter_size_ ))
    {
      ROS_ERROR( "[DecayingCloudAggregator] Could not get param \"voxel_filter_size\" from param server" );
      return;
    }

    for ( int i = 0; i < in_topics_.size(); ++i )
    {
      cloud_subs_.push_back(
        nh_.subscribe<sensor_msgs::PointCloud2>( in_topics_[i], static_cast<int>(subscriber_queue_sizes_[i]),
                                                 boost::bind( &DecayingCloudAggregator::cloudCallback, this, _1,
                                                              squared_crop_distances_[i] )));
    }
    out_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>( out_topic_, 1, false );

    tfl_.reset( new tf::TransformListener());
    wait_duration_ = ros::Duration( 0.5 );
    cloud_concatenator_ = CloudConcatenator( max_queue_size_ );
  }

private:
  /**
   * @brief Callback for the input pointclouds
   * @param cloud_in The input pointcloud
   * @param squared_crop_distance The squared crop distance
   */
  void cloudCallback( const sensor_msgs::PointCloud2::ConstPtr &cloud_in, const float squared_crop_distance )
  {
    if ( tfl_->waitForTransform( target_frame_, cloud_in->header.frame_id, cloud_in->header.stamp, wait_duration_ ))
    {
      tf::StampedTransform transform;
      tfl_->lookupTransform( target_frame_, cloud_in->header.frame_id, cloud_in->header.stamp, transform );

      pcl::PointCloud<pcl::PointXYZ> pc_tmp;

      pcl::fromROSMsg( *cloud_in, pc_tmp );

      // Crop the pointcloud if the crop distance is non-negative
      if ( squared_crop_distance >= 0.0f )
        cropCloud( pc_tmp, squared_crop_distance );

      Eigen::Matrix4f sensorToWorld;
      pcl_ros::transformAsMatrix( transform, sensorToWorld );

      boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > pc;
      pc.reset( new pcl::PointCloud<pcl::PointXYZ>());

      pcl::transformPointCloud( pc_tmp, *pc, sensorToWorld );

      cloud_concatenator_.push( pc );

      pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_concatenated_cloud = cloud_concatenator_.concatenate();

      if ( voxel_filter_size_ > 0.0f )
        voxelFilter( *tmp_concatenated_cloud, voxel_filter_size_ );

      std::string cloud_frame_id = target_frame_;

      if ( !publish_frame_.empty())
      {
        if ( tfl_->waitForTransform( publish_frame_, target_frame_, cloud_in->header.stamp, wait_duration_ ))
        {
          tf::StampedTransform publish_transform;
          tfl_->lookupTransform( publish_frame_, target_frame_, cloud_in->header.stamp, publish_transform );

          Eigen::Matrix4f publish_transform_eigen;
          pcl_ros::transformAsMatrix( publish_transform, publish_transform_eigen );

          pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_transformed_cloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ> >();

          pcl::transformPointCloud( *tmp_concatenated_cloud, *tmp_transformed_cloud, publish_transform_eigen );

          tmp_concatenated_cloud = tmp_transformed_cloud;

          cloud_frame_id = publish_frame_;
        }
        else
        {
          ROS_ERROR_THROTTLE( 5.0,
                              "Cannot transform from cloud frame %s to publish_frame %s after waiting %f seconds. Not publishing cloud. This message is throttled.",
                              target_frame_.c_str(),
                              publish_frame_.c_str(),
                              wait_duration_.toSec());
          return;
        }
      }


      pcl::toROSMsg( *tmp_concatenated_cloud, final_cloud_ );

      final_cloud_.header.frame_id = cloud_frame_id;
      final_cloud_.header.stamp = cloud_in->header.stamp;
      out_cloud_pub_.publish( final_cloud_ );
    }
    else
    {
      ROS_ERROR_THROTTLE( 5.0,
                          "Cannot transform from cloud frame %s to target %s after waiting %f seconds. Not publishing cloud. This message is throttled.",
                          cloud_in->header.frame_id.c_str(),
                          target_frame_.c_str(),
                          wait_duration_.toSec());
    }
  }

  /**
   * Apply a voxel filter to a pointcloud
   * @param cloud The pointcloud
   * @param filter_size The filter leaf size (same for x,y,z)
   *
   * @author Jonathan Lichtenfeld
   */
  template<typename T>
  void voxelFilter( pcl::PointCloud<T> &cloud, float filter_size )
  {
    // Container for original & filtered data
    auto *input_cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr input_cloud_ptr( input_cloud );
    pcl::PCLPointCloud2 tmp_cloud_filtered;

    // Convert to PCL data type
    pcl::toPCLPointCloud2( cloud, *input_cloud );

    voxel_filter_.setInputCloud( input_cloud_ptr );
    voxel_filter_.setLeafSize( filter_size, filter_size, filter_size );
    voxel_filter_.filter( tmp_cloud_filtered );

    // Convert back to ROS data type
    pcl::fromPCLPointCloud2( tmp_cloud_filtered, cloud );
  }

  /**
   * Crop pointcloud to a radius specified by the squared crop distance
   * @param cloud The pointcloud
   * @param squared_crop_distance The squared crop distance
   *
   * @author Jonathan Lichtenfeld
   * @author Simon Giegerich
   */
  template<typename T>
  void cropCloud( pcl::PointCloud<T> &cloud, const float squared_crop_distance ) const
  {
    // crop everything outside certain radius
    typename pcl::PointCloud<T>::Ptr cloud_ptr = cloud.makeShared();
    pcl::PointIndices::Ptr inliers( new pcl::PointIndices());
    pcl::ExtractIndices<T> extract;
    for ( int i = 0; i < (*cloud_ptr).size(); i++ )
    {
      T pt( cloud_ptr->points[i].x, cloud_ptr->points[i].y, cloud_ptr->points[i].z );
      if ( pt.x * pt.x + pt.y * pt.y + pt.z * pt.z <= squared_crop_distance )
        inliers->indices.push_back( i );
    }
    extract.setInputCloud( cloud_ptr );
    extract.setIndices( inliers );
    extract.filter( cloud );
  }

  /**
   * @brief A class that stores a fixed number of pointclouds in a circular buffer and concatenates them.
   */
  class CloudConcatenator
  {
  public:
    /**
     * @brief Creates a new CloudConcatenator with a maximum size of 0.
     */
    CloudConcatenator() : CloudConcatenator( 0 ) { }

    /**
     * @brief Creates a new CloudConcatenator with the specified maximum size.
     * @param max_size The maximum size of the CloudConcatenator.
     */
    CloudConcatenator( int max_size ) : max_size( max_size ), current_size( 0 ), front( 0 ), rear( 0 )
    {
      buffer.resize( max_size );
    }

    /**
     * @brief Pushes a new pointcloud to the CloudConcatenator. If the maximum size is reached, the oldest pointcloud is popped.
     * @param item The item to push.
     */
    void push( const boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> &item )
    {
      // If the queue has reached its maximum size, pop the oldest item
      if ( current_size == max_size )
        pop();
      buffer[rear] = item;
      rear = ++rear % max_size;
      current_size++;
    }

    /**
     * @brief Pops the oldest item from the CloudConcatenator.
     */
    void pop()
    {
      if ( current_size > 0 )
      {
        front = ++front % max_size;
        current_size--;
      }
    }

    /**
     * @brief Returns whether the CloudConcatenator is empty.
     * @return True if the CloudConcatenator is empty, false otherwise.
     */
    bool isEmpty() const { return current_size == 0; }

    /**
     * @brief Returns the current size of the CloudConcatenator.
     * @return The current size of the CloudConcatenator.
     */
    int size() const { return current_size; }

    /**
     * @brief Concatenates all pointclouds in the CloudConcatenator.
     * @return The concatenated pointcloud.
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr concatenate()
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ> );
      for ( int i = front; i < front + current_size; i++ )
        *cloud += *buffer[i % max_size];
      return cloud;
    }

  private:
    // The buffer (implemented as a circular buffer) that stores the pointclouds
    std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> buffer;

    // The maximum size of the buffer
    int max_size;

    // The current size of the buffer
    int current_size;

    // The current index of the front of the buffer
    int front;

    // The current index of the rear of the buffer
    int rear;
  };

  // The input topics as XmlRpcValue
  XmlRpc::XmlRpcValue in_topics_xml_;

  // The input topics
  std::vector<std::string> in_topics_;

  // The output topic
  std::string out_topic_;

  // The subscriber queue sizes per input topic
  XmlRpc::XmlRpcValue subscriber_queue_sizes_;

  // The subscribers for the input clouds
  std::vector<ros::Subscriber> cloud_subs_;

  // The publisher for the output cloud
  ros::Publisher out_cloud_pub_;

  // The transform listener
  boost::shared_ptr<tf::TransformListener> tfl_;

  // The duration to wait for a transform
  ros::Duration wait_duration_;

  // The frame in which to aggregate the pointclouds
  std::string target_frame_;

  // The frame in which to publish the pointcloud
  std::string publish_frame_;

  // The final cloud after concatenation that gets published
  sensor_msgs::PointCloud2 final_cloud_;

  // The maximum size of the CloudConcatenator
  int max_queue_size_;

  // The CloudConcatenator that concatenates the pointclouds
  CloudConcatenator cloud_concatenator_;

  // The voxel filter for downsampling the pointcloud
  pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_filter_;

  // The size of the voxel filter
  float voxel_filter_size_;

  // The crop distances as XmlRpcValue per input topic
  XmlRpc::XmlRpcValue crop_distances_xml_;

  // The squared crop distances per input topic (negative if no cropping should be performed)
  std::vector<float> squared_crop_distances_;
};

int main( int argc, char **argv )
{
  ros::init( argc, argv, "decaying_cloud_aggregator" );

  DecayingCloudAggregator ls;

  ros::spin();
}
