#include <chrono>
#include <thread>

#include "cartographer/common/task.h"
#include "glog/logging.h"

int main(int argc, char** argv) {
  cartographer::common::ThreadPool p(2);

  cartographer::common::Task a([]() {
	  LOG(INFO) << "a";
	  std::this_thread::sleep_for(std::chrono::milliseconds(500));
	  LOG(INFO) << "!a";
  });

  cartographer::common::Task b1([]() {
  	  LOG(INFO) << "b1";
	  std::this_thread::sleep_for(std::chrono::milliseconds(500));
	  LOG(INFO) << "!b1";
    });
  b1.AddDependency(&a);
  cartographer::common::Task b2([]() {
  	  LOG(INFO) << "b2";
	  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	  LOG(INFO) << "!b2";
    });
  b2.AddDependency(&a);

  cartographer::common::TasksSchedulingTask c(
		  [&b1, &b2](cartographer::common::Task::TaskDispatcher dispatcher) {
	  LOG(INFO) << "Dispatching tasks";
	  dispatcher(&b1);
	  dispatcher(&b2);
  });
  c.AddDependency(&a);


  cartographer::common::Task d([]() {
    	  LOG(INFO) << "d";
    	  std::this_thread::sleep_for(std::chrono::milliseconds(500));
    	  LOG(INFO) << "!d";
      });
  d.AddDependency(&c);

  c.Dispatch(&p);
  d.Dispatch(&p);
  a.Dispatch(&p);
  //std::this_thread::sleep_for(std::chrono::milliseconds(500));
  //b.Dispatch(&p);
  std::this_thread::sleep_for(std::chrono::milliseconds(5000));
}
