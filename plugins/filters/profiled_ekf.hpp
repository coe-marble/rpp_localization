#pragma once

#include "ekf_estimator.hpp"

// Selected only by the profiling benchmark's temporary workspace.
class ProfiledEkf final : public EkfBase<true> {};
