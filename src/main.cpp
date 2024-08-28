#include "darknet_ros/slicing.hpp"

#include <vector>


#include <opencv2/opencv.hpp>
extern "C" image mat_to_image(cv::Mat m);
extern "C" cv::Mat image_to_mat(image im);




int main(int argc, char** argv)
{
    cv::Mat input_im = cv::imread(argv[1], cv::IMREAD_COLOR);
    darknet_ros::Slicing slicer(640, 640, 0.2, 0.2);
    // std::vector<cv::Rect> slices;
    // std::vector<cv::Mat> sliced_images;
    // std::vector<cv::Point> slice_start_points;
    // cv::Size original_image_size;
    std::string save_path ="/umd2_ws/src/umd_darknet";
    // convert image to darknet image
    image im = mat_to_image(input_im);

    std::vector<image> sliced_images;
    std::vector<std::vector<float>> slice_start_points;
    slicer.get_sliced_images(im, sliced_images, slice_start_points);

    std::vector<box> prediction_boxes;

    for(int i = 0; i < sliced_images.size(); i++)
    {
        cv::Mat sliced_im = image_to_mat(sliced_images[i]);
        std::cout << "Showing sliced image " << i << std::endl;
        // generate random from 0 to 1
        box random_box = {rand() % 100 / 100.0, rand() % 100 / 100.0, 100.0 / sliced_images[0].w, 100.0 / sliced_images[0].h};
        // draw the box
        cv::rectangle(sliced_im, cv::Rect(random_box.x * sliced_im.cols, random_box.y * sliced_im.rows, random_box.w * sliced_im.cols, random_box.h * sliced_im.rows), cv::Scalar(0, 255, 0), 2);
        prediction_boxes.push_back(random_box);
        std::string sliced_images_window = "Sliced Image " + std::to_string(i);
        cv::imshow(sliced_images_window, sliced_im);
        // cv::waitKey(0);
    }

    // joint images back and show
    image joined_image;

    slicer.join_images(im, sliced_images, slice_start_points, joined_image);
    slicer.transform_prediction_points_to_original_image(slice_start_points, sliced_images, joined_image, prediction_boxes);

    cv::Mat joined_im = image_to_mat(joined_image);
    for (int i = 0; i < prediction_boxes.size(); i++)
    {
        cv::rectangle(joined_im, cv::Rect(prediction_boxes[i].x * joined_im.cols, prediction_boxes[i].y * joined_im.rows, prediction_boxes[i].w * joined_im.cols, prediction_boxes[i].h * joined_im.rows), cv::Scalar(0, 255, 0), 2);
    }
    cv::imshow("Joined Image", joined_im);
    cv::waitKey(0);


    // slicer.slice_image(image, sliced_images,slice_start_points,original_image_size, save_path);

    // // pick random box in each slice and convert to original image
    // std::vector<cv::Rect> prediction_boxes;

    // // show the sliced images
    // for (int i = 0; i < sliced_images.size(); i++)
    // {
    //     std::cout << "Showing sliced image " << i << std::endl;
    //     // std::cout << "Slice start point: " << slice_start_points[i].x << ", " << slice_start_points[i].y << std::endl;
    //     std::cout << "Original image size: " << original_image_size.width << ", " << original_image_size.height << std::endl;
    //     // pick a random box in the slice that is within the slice size
    //     cv::Rect random_box = cv::Rect(rand() % sliced_images[i].cols, rand() % sliced_images[i].rows, 100, 100);
    //     prediction_boxes.push_back(random_box);
    //     // show the random box slice
    //     cv::Mat random_box_slice = sliced_images[i].clone();
    //     // use a different color for each box
    //     cv::rectangle(random_box_slice, random_box, cv::Scalar(rand() % 255, rand() % 255, rand() % 255), 2);
    //     cv::imshow("Sliced Image", random_box_slice);
    //     // store the image 
    //     std::string save_path_ = save_path + "/" + std::to_string(i) + ".jpg";
    //     cv::imwrite(save_path_, random_box_slice);
    //     cv::waitKey(0);
    // }

    // // joint images back and show
    // cv::Mat joined_image;
    // slicer.join_images(sliced_images, slice_start_points, original_image_size, joined_image);
    // // convert the prediction boxes to the original image
    // slicer.transform_prediction_points_to_original_image(slice_start_points, prediction_boxes);
    // for (int i = 0; i < prediction_boxes.size(); i++)
    // {
    //     cv::rectangle(joined_image, prediction_boxes[i], cv::Scalar(0, 255, 0), 2);
    // }
    // cv::imshow("Joined Image", joined_image);
    // cv::waitKey(0);

    return 0;
}
