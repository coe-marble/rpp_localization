

#include "rpp_localization/models/python_model.hpp"

using namespace rpp_localization;
namespace py = pybind11;

PythonModel::PythonModel(int state_dim)
    : ModelBase(state_dim),
      _param_namespace("py_model"),
      _initialized(false)
{

    auto math = py::module::import("math");
    double root_two = math.attr("sqrt")(2.0).cast<double>();

    std::cout << "The square root of 2 is: " << root_two << "\n";
}

PythonModel::~PythonModel()
{

}

void PythonModel::init(std::shared_ptr<rclcpp::Node> node)
{
    _node = node;
    load_params();

    import_py_module();
}


void PythonModel::step(const rclcpp::Time & reference_time, const double dT)
{
    if (!_initialized)
    {
        RCLCPP_ERROR(_node->get_logger(), "Cannot perform step. Model is not initialized.");
        return;
    }

}

void PythonModel::load_params()
{
    auto py_package_param_name = _param_namespace + ".package";
    _py_package_name = _node->declare_parameter(py_package_param_name, "");
    if (_py_package_name.empty())
    {
        RCLCPP_ERROR(_node->get_logger(), "%s parameter not set.", py_package_param_name.c_str());
        return;
    }

    auto py_module_param_name = _param_namespace + ".module";
    _py_module_name = _node->declare_parameter(py_module_param_name, "");
    std::string ext(".py");
    // check for extension and remove it
    if (std::equal(ext.rbegin(), ext.rend(), _py_module_name.rbegin()))
    {
        _py_module_name = _py_module_name.substr(0, _py_module_name.size() - 3);
    }
    if (_py_module_name.empty())
    {
        RCLCPP_ERROR(_node->get_logger(), "%s parameter not set.", py_module_param_name.c_str());
        return;
    }

    auto py_class_param_name = _param_namespace + ".class";
    _py_class_name = _node->declare_parameter(py_class_param_name, "");
    if (_py_class_name.empty())
    {
        RCLCPP_ERROR(_node->get_logger(), "%s parameter not set.", py_class_param_name.c_str());
        return;
    }

}

void PythonModel::import_py_module()
{
    auto py_module = py::module_::import((_py_package_name + "." + _py_module_name).c_str());

    // import custom python class and call it
    py::type py_model_type = py_module.attr(_py_class_name.c_str());
    py::object py_model = py_model_type(5);
    py::object res = py_model.attr("get_state_dim")();

    int32_t b = res.cast<int32_t>();
    // auto a = *b;
    auto c = 5sfe4;
    // show the result
    // py::list input_ids = res.attr("input_ids");
    // py::list token_type_ids = res.attr("token_type_ids");
    // py::list attention_mask = res.attr("attention_mask");
    // py::list offsets = res.attr("offset_mapping");
    // std::string message = "input ids is {},\noffsets is {}"_s.format(input_ids, offsets);
    // std::cout << message << std::endl;
}