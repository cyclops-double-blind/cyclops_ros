#include "cyclops_ros_backend/publisher.hpp"
#include "cyclops/cyclops.hpp"

#include <thread>

namespace cyclops {
  class OptimizationThreadSpinner {
  private:
    std::shared_ptr<CyclopsMain> _cyclops_main;
    std::shared_ptr<RosPublisherContext> _publisher;

    bool _started = false;
    void spin();

  public:
    OptimizationThreadSpinner(
      std::shared_ptr<CyclopsMain> cyclops_main,
      std::shared_ptr<RosPublisherContext> publisher);
    std::thread start();
  };
}  // namespace cyclops
