
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <vulkan_glasses_for_robots/visim_processor.h>

#include <iostream>

DEFINE_string(visim_project_folder, "/media/secssd/dataset/50s_house_45/visim",
              "visensor simulator project folder");
DEFINE_string(camera_id, "cam0", "cammera id on the visim project");
DEFINE_string(output_folder_path, "/media/secssd/tmp/render_test/a2",
              "result path");
DEFINE_int32(step_skip, 1, "step skip");
DEFINE_string(resource_folder, "/media/secssd/tmp/render_test/a2",
              "result path");

int main(int argc, char *argv[]) {
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::SetLogDestination(google::INFO, "/media/secssd/tmp/foobar.log");
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  VisimProcessor processor;
  if (!processor.initialization(FLAGS_visim_project_folder, FLAGS_camera_id,
                                FLAGS_output_folder_path, FLAGS_step_skip))
    return EXIT_FAILURE;
  std::cout << "Press enter to process...";
  //getchar();
  processor.runHeadless();
  std::cout << "Finished. Press enter to terminate...";
  //getchar();
  return EXIT_SUCCESS;
}
