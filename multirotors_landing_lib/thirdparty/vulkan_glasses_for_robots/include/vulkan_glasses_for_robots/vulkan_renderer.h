/*
* based on Vulkan Example - Minimal headless rendering example
*
* Copyright (C) 2017 by Sascha Willems - www.saschawillems.de
* Copyright (C) 2018 by Lucas Teixeira
*
* This code is licensed under the MIT license (MIT)
* (http://opensource.org/licenses/MIT)
*/

#pragma once
//#define DEBUG 1
#include <glog/logging.h>

#include <eigen3/Eigen/Core>
#include <map>
#include <opencv2/core.hpp>
#include <vector>

#define GLM_FORCE_RADIANS
#define GLM_FORCE_DEPTH_ZERO_TO_ONE
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/hash.hpp>

typedef std::vector<float> VecQuality;

#include <vulkan/vulkan.h>
#include <vulkan_glasses_for_robots/VulkanTools.h>

//#define DEBUG (!NDEBUG) //enable validation layers

#define BUFFER_ELEMENTS 32

static VKAPI_ATTR VkBool32 VKAPI_CALL debugMessageCallback(
    VkDebugReportFlagsEXT flags, VkDebugReportObjectTypeEXT objectType,
    uint64_t object, size_t location, int32_t messageCode,
    const char* pLayerPrefix, const char* pMessage, void* pUserData) {
  std::cout << "[VALIDATION]: " << pLayerPrefix << " - " << pMessage << "\n";
  return VK_FALSE;
}

struct Vertex {
  glm::vec3 pos;
  //glm::vec3 color;
  glm::vec2 texCoord;

  static VkVertexInputBindingDescription getBindingDescription() {
    VkVertexInputBindingDescription bindingDescription = {};
    bindingDescription.binding = 0;
    bindingDescription.stride = sizeof(Vertex);
    bindingDescription.inputRate = VK_VERTEX_INPUT_RATE_VERTEX;
    return bindingDescription;
  }

  static std::array<VkVertexInputAttributeDescription, 2>
  getAttributeDescriptions() {
    std::array<VkVertexInputAttributeDescription, 2> attributeDescriptions = {};

    attributeDescriptions[0].binding = 0;
    attributeDescriptions[0].location = 0;
    attributeDescriptions[0].format = VK_FORMAT_R32G32B32_SFLOAT;
    attributeDescriptions[0].offset = offsetof(Vertex, pos);

    //        attributeDescriptions[1].binding = 0;
    //        attributeDescriptions[1].location = 1;
    //        attributeDescriptions[1].format = VK_FORMAT_R32G32B32_SFLOAT;
    //        attributeDescriptions[1].offset = offsetof(Vertex, color);

    attributeDescriptions[1].binding = 0;
    attributeDescriptions[1].location = 1;
    attributeDescriptions[1].format = VK_FORMAT_R32G32_SFLOAT;
    attributeDescriptions[1].offset = offsetof(Vertex, texCoord);

    return attributeDescriptions;
  }

  bool operator==(const Vertex& other) const {
    return pos == other.pos &&
           texCoord == other.texCoord;  //&& color == other.color
  }
};

namespace std {
template <>
struct hash<Vertex> {
  size_t operator()(Vertex const& vertex) const {
    return (hash<glm::vec3>()(vertex.pos) ^
            (hash<glm::vec2>()(vertex.texCoord) << 1));
  }
};
}  // namespace std

namespace vrglasses_for_robots {

struct ThreeDModel {
  std::string name;
  std::string obj_file;
  std::string texture_file;
  long unsigned int begin_vertex_index;
  long unsigned int begin_vertex_count;
  size_t material_index;

  ThreeDModel()
      : name("none"),
        obj_file("none"),
        texture_file("none"),
        begin_vertex_index(0),
        begin_vertex_count(0),
        material_index(9999) {}
};

struct SceneItem {
  std::string model_name;
  glm::mat4 T_World2Model;
  SceneItem() : model_name("none") {
    T_World2Model = glm::mat4(1.0);
  }
};

struct Texture2D {
  VkImage textureImage;
  VkDeviceMemory textureImageMemory;
  VkImageView textureImageView;
  VkSampler textureSampler;

  VkDescriptorSet descriptorSet;
};

class VulkanRenderer {
  private:
  VkInstance instance;
  VkPhysicalDevice physicalDevice;
  VkDevice device;
  uint32_t queueFamilyIndex;
  VkPipelineCache pipelineCache;
  VkQueue queue;
  VkCommandPool commandPool;
  VkCommandBuffer commandBuffer;
  VkDescriptorSetLayout descriptorSetLayout;
  VkPipelineLayout pipelineLayout;
  VkPipeline pipeline;
  std::vector<VkShaderModule> shaderModules;

  VkBuffer vertexStagingBuffer, indexStagingBuffer, vertexBuffer, indexBuffer;
  VkDeviceMemory vertexMemory, indexMemory, vertexStagingMemory,
      indexStagingMemory;

  VkImage dstImage;
  VkDeviceMemory dstImageMemory;
  VkBuffer image_buffer;
  VkDeviceMemory image_buffer_memory;

  VkDescriptorPool descriptorPool;

  uint32_t width_, height_;

  std::string shader_vert_spv_;
  std::string shader_frag_spv_;
  float far_, near_;
  /*
     Create framebuffer attachments
  */

  VkFormat colorFormat = VK_FORMAT_R8G8B8A8_UNORM;
  VkFormat depthFormat = VK_FORMAT_D32_SFLOAT_S8_UINT;

  struct FrameBufferAttachment {
    VkImage image;
    VkDeviceMemory memory;
    VkImageView view;
  };

  /*
     Prepare vertex and index buffers
  */

  std::vector<Vertex> vertices_;
  std::vector<uint32_t> indices_;
  //size_t max_landmark_count_, max_indice_count_;
  std::vector<ThreeDModel> models_;
  std::map<std::string, size_t> models_index_;
  std::vector<SceneItem> scene_items_;

  std::vector<Texture2D> textures_;

  glm::mat4 vp_cv_, projection_cv_;

  VkFramebuffer framebuffer;
  FrameBufferAttachment colorAttachment, depthAttachment;
  VkRenderPass renderPass;

  VkDebugReportCallbackEXT debugReportCallback{};

  uint32_t getMemoryTypeIndex(uint32_t typeBits,
                              VkMemoryPropertyFlags properties);

  VkResult createBuffer(VkBufferUsageFlags usageFlags,
                        VkMemoryPropertyFlags memoryPropertyFlags,
                        VkBuffer* buffer, VkDeviceMemory* memory,
                        VkDeviceSize size, void* data = nullptr);

  /*
     Submit command buffer to a queue and wait for fence until queue
     operations have been finished
  */
  void submitWork(VkCommandBuffer cmdBuffer, VkQueue queue);

  void initVulkan(bool enableValidation);

  void buildRenderPass(uint32_t width, uint32_t height);

  void drawTriangles(uint32_t width, uint32_t height);

  void saveImageDepthmap(uint32_t width, uint32_t height,
                         cv::Mat& result_depth_map,
                         cv::Mat& result_attribute_map);

  VkCommandBuffer beginSingleTimeCommands();

  void endSingleTimeCommands(VkCommandBuffer commandBuffer);

  void copyBuffer(VkBuffer srcBuffer, VkBuffer dstBuffer, VkDeviceSize size);

  void createVertexBuffer();

  void createIndexBuffer();

  static glm::mat4 ProjectionMatrixForCameraIntrinsics(float width,
                                                       float height, float fx,
                                                       float fy, float cx,
                                                       float cy, float near,
                                                       float far);

  void allocateDataBuffers();
  void releaseMeshDataBuffers();

  static void buildOpenglProjectionFromIntrinsics(
      glm::mat4& matPerspective, glm::mat4& matProjection,
      glm::mat4& matCVProjection, int img_width, int img_height, float alpha,
      float beta, float skew, float u0, float v0, float near, float far);

  float convertZbufferToDepth(float near, float far, float zValue);

  void createTextureImage(Texture2D& tex, std::string filename_texture);
  void setupDescriptorSet(Texture2D& tex);

  void createBuffer(VkDeviceSize size, VkBufferUsageFlags usage,
                    VkMemoryPropertyFlags properties, VkBuffer& buffer,
                    VkDeviceMemory& bufferMemory);
  uint32_t findMemoryType(uint32_t typeFilter,
                          VkMemoryPropertyFlags properties);
  void createImage(uint32_t width, uint32_t height, VkFormat format,
                   VkImageTiling tiling, VkImageUsageFlags usage,
                   VkMemoryPropertyFlags properties, VkImage& image,
                   VkDeviceMemory& imageMemory);
  void transitionImageLayout(VkImage image, VkFormat format,
                             VkImageLayout oldLayout, VkImageLayout newLayout);
  VkImageView createImageView(VkImage image, VkFormat format,
                              VkImageAspectFlags aspectFlags);
  void copyBufferToImage(VkBuffer buffer, VkImage image, uint32_t width,
                         uint32_t height);
  void setupDescriptorPool();

  public:
  VulkanRenderer(uint32_t width, uint32_t height, float near, float far,
                 const std::string& shader_spv_folder);

  void setCamera(float p_focal_u, float p_focal_v, float p_center_u,
                 float p_center_v);
  void setCamera(glm::mat4 mvp);

  //
  bool loadMeshs(const std::string& model_folder,
                 const std::string& model_list);

  bool loadMesh(const std::string& filename_model_obj,
                const std::string& filename_model_tex);

  void noFileScene();

  bool loadScene(const std::string& scene_file);

  void copyVertex();

  void renderMesh(cv::Mat& result_depth_map, cv::Mat& result_attribute_map);

  void cleanUp();

  void clearModels();

  ~VulkanRenderer();

  //void loadMeshs(const std::string &filename_model_obj);
  //bool loadVertex(const std::string &filename_model_obj);
  bool loadVertex(const size_t model_idx);
};
}  // namespace vrglasses_for_robots
