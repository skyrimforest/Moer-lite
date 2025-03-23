#include <CoreLayer/Math/Math.h>
#include <FunctionLayer/Camera/Pinhole.h>
#include <FunctionLayer/Integrator/Integrator.h>
#include <FunctionLayer/Sampler/Sampler.h>
#include <FunctionLayer/Scene/Scene.h>
#include <FunctionLayer/Texture/Mipmap.h>
#include <ResourceLayer/Factory.h>
#include <ResourceLayer/FileUtil.h>
#include <ResourceLayer/Image.h>
#include <ResourceLayer/JsonUtil.h>
#include <chrono>
#include <fstream>
#include <regex>
#include <stdio.h>
#include <chrono>
#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60
int last_percent = -1;
std::chrono::time_point<std::chrono::system_clock> each_percent_start[100];
inline void printProgress(float percentage)
{
  int val = (int)(percentage * 100);
  if (val == last_percent)
    return;
  each_percent_start[val] = std::chrono::system_clock::now();
  float time_elapsed = (std::chrono::duration_cast<std::chrono::milliseconds>(
                            each_percent_start[val] - each_percent_start[0]))
                           .count() /
                       1000.f;
  float total_time_predict;
  if (val <= 3)
  {
    total_time_predict = time_elapsed * 100 / val;
  }
  else
  {
    float each_time_latest3 =
        (std::chrono::duration_cast<std::chrono::milliseconds>(
             each_percent_start[val] - each_percent_start[val - 3]))
            .count() /
        1000.f / 3;
    total_time_predict = each_time_latest3 * (100 - val);
    total_time_predict += time_elapsed;
  }
  last_percent = val;
  int lpad = (int)(percentage * PBWIDTH);
  int rpad = PBWIDTH - lpad;
  printf("\r%3d%% [%.*s%*s] %.2fs/%.2fs%s", val, lpad, PBSTR, rpad, "",
         time_elapsed, total_time_predict, "                          ");
  fflush(stdout);
}


int main(int argc, char **argv) {
  const std::string sceneDir = std::string(argv[1]);
  FileUtil::setWorkingDirectory(sceneDir);
  std::string sceneJsonPath = FileUtil::getFullPath("scene.json");
  std::ifstream fstm(sceneJsonPath);
  Json json = Json::parse(fstm);
  auto camera = Factory::construct_class<Camera>(json["camera"]);
  auto scene = std::make_shared<Scene>(json["scene"]);
  auto integrator = Factory::construct_class<Integrator>(json["integrator"]);
  auto sampler = Factory::construct_class<Sampler>(json["sampler"]);
  int spp = sampler->xSamples * sampler->ySamples;
  int width = camera->film->size[0], height = camera->film->size[1];

  auto start = std::chrono::system_clock::now();

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      Vector2f NDC{(float)x / width, (float)y / height};
      Spectrum li(.0f);
      for (int i = 0; i < spp; ++i) {
        Ray ray = camera->sampleRayDifferentials(
            CameraSample{sampler->next2D()}, NDC);
        li += integrator->li(ray, *scene, sampler);
      }
      camera->film->deposit({x, y}, li / spp);

      int finished = x + y * width;
      if (finished % 5 == 0) {
        printProgress((float)finished / (height * width));
      }
    }
  }
  printProgress(1.f);

  auto end = std::chrono::system_clock::now();

  printf("\nRendering costs %.2fs\n",
         (std::chrono::duration_cast<std::chrono::milliseconds>(end - start))
                 .count() /
             1000.f);

  //* 目前支持输出为png/hdr两种格式
  std::string outputName =
      fetchRequired<std::string>(json["output"], "filename");
  if (std::regex_match(outputName, std::regex("(.*)(\\.png)"))) {
    camera->film->savePNG(outputName.c_str());
  } else if (std::regex_match(outputName, std::regex("(.*)(\\.hdr)"))) {
    camera->film->saveHDR(outputName.c_str());
  } else {
    std::cout << "Only support output as PNG/HDR\n";
  }
}
