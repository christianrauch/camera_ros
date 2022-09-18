#include "compression.hpp"

// include 'stdio.h' manually before 'jpeglib.h' to define 'size_t'
// https://github.com/libjpeg-turbo/libjpeg-turbo/blob/2.1.4/libjpeg.txt#L825-L829
// "Before including jpeglib.h, include system headers that define at least the
// typedefs FILE and size_t."

// clang-format off
#include <stdio.h>
#include <jpeglib.h>
// clang-format on

//void
//decompress(const std::vector<uint8_t> &data, std::vector<uint8_t> &img_buffer)
sensor_msgs::msg::Image::UniquePtr
//decompress(const sensor_msgs::msg::CompressedImage::SharedPtr &data)
decompress(const sensor_msgs::msg::CompressedImage::UniquePtr &data)
{
  struct jpeg_decompress_struct cinfo;
  struct jpeg_error_mgr jerr;

  cinfo.err = jpeg_std_error(&jerr);

  // silence all messages
  cinfo.err->output_message = [](j_common_ptr) {};

  jpeg_create_decompress(&cinfo);

  //  jpeg_mem_src(&cinfo, data.data(), data.size());
  jpeg_mem_src(&cinfo, data->data.data(), data->data.size());

  int ret = jpeg_read_header(&cinfo, TRUE);
  if (ret != 1) {
    //    std::cerr << "not a jpeg" << std::endl;
    //        return EXIT_FAILURE;
  }

  jpeg_start_decompress(&cinfo);


  //    std::vector<uint8_t> img_buffer(cinfo.output_width*cinfo.output_height*cinfo.output_components);
  sensor_msgs::msg::Image::UniquePtr msg_img;
  msg_img = std::make_unique<sensor_msgs::msg::Image>();
  msg_img->data.resize(cinfo.output_width * cinfo.output_height * cinfo.output_components);
  while (cinfo.output_scanline < cinfo.output_height) {
    unsigned char *buffer_array[1];
    buffer_array[0] =
      msg_img->data.data() + (cinfo.output_scanline * cinfo.output_width * cinfo.output_components);
    jpeg_read_scanlines(&cinfo, buffer_array, 1);
  }

  msg_img->encoding = "rgb8";
  msg_img->header = data->header;
  msg_img->height = cinfo.output_height;
  msg_img->width = cinfo.output_width;
  msg_img->step = cinfo.output_width * cinfo.output_components;

  jpeg_finish_decompress(&cinfo);
  jpeg_destroy_decompress(&cinfo);

  return msg_img;
}
