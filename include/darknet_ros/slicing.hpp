#ifndef UMD_DARKNET_INCLUDE_UMD_DARKNET_SLICING_HPP_
#define UMD_DARKNET_INCLUDE_UMD_DARKNET_SLICING_HPP_


// #include "opencv2/highgui/highgui_c.h"
// #include "opencv2/imgproc/imgproc_c.h"
// #include <opencv2/core/version.hpp>

#include <iostream>
#include <vector>
// #include <opencv2/opencv.hpp>
#include "darknet.h"
#include "image.h"

namespace darknet_ros
{
class Slicing
{

    public:
        Slicing(int slice_height, int slice_width, double slice_overlap_height_ratio, double slice_overlap_width_ratio, int num_of_batches = 1, bool auto_slice_resolution = false);
        ~Slicing();

        void get_sliced_images(const image im, std::vector<image> & sliced_images, std::vector<std::vector<float>>& starting_points);
        std::vector<box> get_sliced_boxes(int image_height, int image_width, int num_slices);
        void transform_prediction_point_to_original_image(const std::vector<float>& slice_start_point, const image& sliced_image, image& joined_image, box& prediction_box);
        void transform_prediction_points_to_original_image(const std::vector<std::vector<float>>& slice_start_points, const std::vector<image>& sliced_images, image& joined_image, std::vector<box>& prediction_boxes);
        void join_images(const image im, const std::vector<image>& sliced_images, const std::vector<std::vector<float>>& slice_start_points, image& joined_image);
       int get_number_of_slices(int image_height, int image_width);

    private:
        int slice_height_{256};
        int slice_width_{256};
        double slice_overlap_height_ratio_{0.2};
        double slice_overlap_width_ratio_{0.2};
        int x_overlap_{0};
        int y_overlap_{0};
        int num_of_batches_{1}; // only supports one. Might change in the furte
        bool auto_slice_resolution_{false};
};
} // namespace umd_darknet

#endif // UMD_DARKNET_INCLUDE_UMD_DARKNET_SLICING_HPP_
