
#include "rpp_localization/ros/rosbag_filter.hpp"
#include <filesystem>

// #include "rpp_localization/models/python_model.hpp"

using namespace rpp_localization;

typedef rpp_localization::RosBagFilter<rpp_localization::Ekf<rpp_localization::ConstantAccelerationModel>> FilterEkf;
typedef rpp_localization::RosBagFilter<rpp_localization::Ukf<rpp_localization::ConstantAccelerationModel>> FilterUkf;
typedef rpp_localization::RosBagFilter<rpp_localization::InEkf<rpp_localization::InEKF::InertialProcess>> FilterInEkf;

/// @brief Prepare arguments for rclcpp initialization with custom
/// path parameters
/// @param argc
/// @param argv
void prepare_args(int& argc, char**& argv, std::string& bag_path, bool& save_file,
  std::string& out_path, std::string& filter_type)
{
  auto vec = new std::vector<char*>
  {
  };

  for (int i = 0; i < argc; i++)
  {
    if (std::string(argv[i]) == "--bag")
    {
      bag_path = std::string(argv[++i]);
      continue;
    }
    if (std::string(argv[i]) == "-s")
    {
      save_file = true;
      if (i + 1 > argc)
        out_path = "";
      else
        out_path = std::string(argv[++i]);
      continue;
    }
    if (std::string(argv[i]) == "--filter-type")
    {
      if (i + 1 > argc)
        continue;
      filter_type = std::string(argv[++i]);
      continue;
    }
    vec->push_back(argv[i]);
  }

  argc = vec->size();
  argv = vec->data();
}


void print_input_args(int argc, char** argv)
{
  auto logger = rclcpp::get_logger("rosbag_filter");
  RCLCPP_INFO(logger, "Num inputs: %d", argc);
  auto a = argv;
  for (int i = 0; i < argc; i++)
  {
    RCLCPP_INFO(logger, *a);
    a++;
  }
}



int main(int argc, char ** argv)
{

  std::string bag_path;
  std::string out_path;
  bool save_file = false;
  std::string filter_type = "ekf";
  prepare_args(argc, argv, bag_path, save_file, out_path, filter_type);
  print_input_args(argc, argv);
  rclcpp::init(argc, argv);

  if (save_file)
  {
    if (!out_path.empty())
    {
      if (!std::filesystem::is_directory(out_path))
      {
        std::filesystem::create_directories(out_path);
      }
    }
    else
    {
      out_path = bag_path.substr(bag_path.find_last_of("/\\") + 1);
    }
  }


  rclcpp::NodeOptions options;
  options.arguments({"rosbag_filter"});

  if (bag_path.empty())
  {
    RCLCPP_ERROR(rclcpp::get_logger("rosbag_filter"), "No bag path provided");
    return 1;
  }

  RosBagMsgProvider provider(bag_path);
  FilterResult result;
  if (filter_type == "ekf")
  {
    auto filter = std::make_shared<FilterEkf>(options, provider, 50);
    filter->init();
    result = filter->filter();
  }
  else if (filter_type == "ukf")
  {
    auto filter = std::make_shared<FilterUkf>(options, provider, 50);
    filter->init();
    result = filter->filter();
  }
  else if (filter_type == "inekf")
  {
    auto filter = std::make_shared<FilterInEkf>(options, provider, 50);
    filter->init();
    result = filter->filter();
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger("rosbag_filter"), "Unknown filter type");
    return 1;
  }

  if (save_file)
  {
    std::ofstream file;
    file.open(out_path + "/states.bin", std::ios::out | std::ios::binary);
    for (auto& state : result.states)
    {
      file.write((char*)state.data(), state.size() * sizeof(double));
    }
    file.close();
    file.open(out_path + "/covariances.bin", std::ios::out | std::ios::binary);
    for (auto& cov : result.covariances)
    {
      Eigen::Map<const Eigen::VectorXd> v(cov.data(), cov.size());
      file.write((char*)v.data(), v.size() * sizeof(double));
    }
    file.close();
    file.open(out_path + "/timestamps.bin", std::ios::out | std::ios::binary);
    for (auto& time : result.timestamps)
    {
      file.write((char*)&time, sizeof(double));
    }
    file.close();
  }

  rclcpp::shutdown();
  return 0;
}

