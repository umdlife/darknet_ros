#include "darknet_ros/slicing.hpp"


namespace darknet_ros
{

Slicing::Slicing(int slice_height, int slice_width, double slice_overlap_height_ratio, double slice_overlap_width_ratio, int num_of_batches, bool auto_slice_resolution)
    : slice_height_(slice_height), slice_width_(slice_width), slice_overlap_height_ratio_(slice_overlap_height_ratio), slice_overlap_width_ratio_(slice_overlap_width_ratio), num_of_batches_(num_of_batches), auto_slice_resolution_(auto_slice_resolution)
{
    x_overlap_ = static_cast<int>(slice_width_ * slice_overlap_width_ratio_);
    y_overlap_ = static_cast<int>(slice_height_ * slice_overlap_height_ratio_);
}

Slicing::~Slicing()
{
}

void Slicing::get_sliced_images(const image im, std::vector<image> & sliced_images, std::vector<std::vector<float>>& starting_points)
{
    int image_height = im.h;
    int image_width = im.w;

    if(image_height == 0 || image_width == 0)
    {
        std::cerr << "Image is empty" << std::endl;
        return;
    }
    const int num_slices = get_number_of_slices(image_height, image_width);
    std::cout << "Number of slices: " << num_slices << std::endl;

    std::vector<box> sliced_boxes = get_sliced_boxes(image_height, image_width, num_slices);
    starting_points.reserve(num_slices);
    sliced_images.reserve(num_slices);
    int n_ims = 0;

    // iterate through the slices and get the sliced images
    for (const auto& slice : sliced_boxes)
    {
        n_ims++;
        image sliced_image = crop_image(im, slice.x * image_width, slice.y * image_height, slice.w * image_width, slice.h * image_height);
        sliced_images.push_back(sliced_image);
        starting_points.push_back({slice.x, slice.y});

    }
}

std::vector<box> Slicing::get_sliced_boxes(int image_height, int image_width, int num_slices)
{
    std::vector<box> slices;
    int y_min = 0, y_max = 0;
    // Early exit if the slice dimensions are invalid
    if (slice_height_ <= 0 || slice_width_ <= 0)
    {
        std::cerr << "Slice height and width must be greater than 0" << std::endl;
        return slices;
    }

    // precompute variables
    const int y_overlap = static_cast<int>(slice_height_ * slice_overlap_height_ratio_);
    const int x_overlap = static_cast<int>(slice_width_ * slice_overlap_width_ratio_);
    const float inv_image_width = 1.0f / static_cast<float>(image_width);
    const float inv_image_height = 1.0f / static_cast<float>(image_height);


    slices.reserve(num_slices);

    while ( y_max < image_height)
    {
        int x_min = 0, x_max = 0;
        y_max = y_min + slice_height_;
        while (x_max < image_width)
        {
            x_max = x_min + slice_width_;
            if (y_max > image_height || x_max > image_width)
            {
                int xmax = std::min(image_width, x_max);
                int ymax = std::min(image_height, y_max);
                int xmin = std::max(0, xmax - slice_width_);
                int ymin = std::max(0, ymax - slice_height_);
                box slice = {
                    static_cast<float>(xmin) * inv_image_width,
                    static_cast<float>(ymin) * inv_image_height,
                    static_cast<float>(xmax - xmin) * inv_image_width,
                    static_cast<float>(ymax - ymin) * inv_image_height
                };
                slices.push_back(slice);
            }
            else 
            {
                box slice 
                {
                    static_cast<float>(x_min) * inv_image_width,
                    static_cast<float>(y_min) * inv_image_height,
                    static_cast<float>(x_max-x_min) * inv_image_width,
                    static_cast<float>(y_max-y_min) * inv_image_height
                };
                slices.push_back(slice);
            }
            x_min = x_max - x_overlap;
        }
        y_min = y_max - y_overlap;
    }

    return slices;
}


void Slicing::transform_prediction_points_to_original_image(const std::vector<std::vector<float>>& slice_start_points, const std::vector<image>& sliced_images, image& joined_image, std::vector<box>& prediction_boxes)
{
    int i = 0;
    for (auto& box : prediction_boxes)
    {
        transform_prediction_point_to_original_image(slice_start_points[i], sliced_images[i], joined_image, box);
        i++;
    }
}

void Slicing::transform_prediction_point_to_original_image(const std::vector<float>& slice_start_point, const image& sliced_image, image& joined_image, box& prediction_box)
{
    // Precompute and store the scaled values for efficiency
    float scaled_x = slice_start_point[0] * joined_image.w;
    float scaled_y = slice_start_point[1] * joined_image.h;

    // Calculate the dimensions of the predicted box in the sliced image
    float box_x_scaled = prediction_box.x * sliced_image.w;
    float box_y_scaled = prediction_box.y * sliced_image.h;
    float box_w_scaled = prediction_box.w * sliced_image.w;
    float box_h_scaled = prediction_box.h * sliced_image.h;

    // Transform the predicted box coordinates to the original image
    prediction_box.x = (box_x_scaled + scaled_x) / joined_image.w;
    prediction_box.y = (box_y_scaled + scaled_y) / joined_image.h;
    prediction_box.w = box_w_scaled / joined_image.w;
    prediction_box.h = box_h_scaled / joined_image.h;
}


void Slicing::join_images(const image im, const std::vector<image>& sliced_images, const std::vector<std::vector<float>>& slice_start_points, image& joined_image)
{
    joined_image = make_image(im.w, im.h, im.c);

    for (int i = 0; i < sliced_images.size(); i++)
    {
        image sliced_image = sliced_images[i];
        int x_offset = slice_start_points[i][0] * im.w;
        int y_offset = slice_start_points[i][1] * im.h;
        embed_image(sliced_image, joined_image, x_offset, y_offset);
    }
}

int Slicing::get_number_of_slices(int image_height, int image_width)
{
    const int num_slices_width = std::ceil(static_cast<double>(image_width - slice_width_) / (slice_width_ - x_overlap_)) + 1;
    const int num_slices_height = std::ceil(static_cast<double>(image_height - slice_height_) / (slice_height_ - y_overlap_)) + 1;
    return num_slices_width * num_slices_height;
}




} // namespace darknet_ros


