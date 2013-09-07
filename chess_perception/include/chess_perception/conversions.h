/*
 * Some code copied in from the old pcl package to make this still compile,
 * should probably be upstreamed to pcl_conversions package at some point.
 */

#ifndef CHESS_PERCEPTION_CONVERSIONS_H_
#define CHESS_PERCEPTION_CONVERSIONS_H_

namespace pcl_broke_again
{

template<typename CloudT> void
toROSMsg (const CloudT& cloud, sensor_msgs::Image& msg)
{
  // Ease the user's burden on specifying width/height for unorganized datasets
  if (cloud.width == 0 && cloud.height == 0)
    throw std::runtime_error("Needs to be a dense like cloud!!");
  else
  {
    if (cloud.points.size () != cloud.width * cloud.height)
      throw std::runtime_error("The width and height do not match the cloud size!");
    msg.height = cloud.height;
    msg.width = cloud.width;
  }

  // sensor_msgs::image_encodings::BGR8;
  msg.encoding = "bgr8";
  msg.step = msg.width * sizeof (uint8_t) * 3;
  msg.data.resize (msg.step * msg.height);
  for (size_t y = 0; y < cloud.height; y++)
  {
    for (size_t x = 0; x < cloud.width; x++)
    {
      uint8_t * pixel = &(msg.data[y * msg.step + x * 3]);
      memcpy (pixel, &cloud (x, y).rgb, 3 * sizeof(uint8_t));
    }
  }
}

}

#endif
