#pragma once

#include "ukf_estimator.hpp"

// Selected only by the profiling benchmark's temporary workspace.
class ProfiledUkf final : public UkfBase<true> {};
