#include <vulkan_glasses_for_robots/vulkan_renderer.h>

#define STB_IMAGE_IMPLEMENTATION
#include <vulkan_glasses_for_robots/stb_image.h>

#define TINYOBJLOADER_IMPLEMENTATION
#include <vulkan_glasses_for_robots/tiny_obj_loader.h>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/string_cast.hpp>
#include <opencv2/imgproc.hpp>
#include <unordered_map>

uint32_t vrglasses_for_robots::VulkanRenderer::getMemoryTypeIndex(
    uint32_t typeBits, VkMemoryPropertyFlags properties) {
  VkPhysicalDeviceMemoryProperties deviceMemoryProperties;
  vkGetPhysicalDeviceMemoryProperties(physicalDevice, &deviceMemoryProperties);
  for (uint32_t i = 0; i < deviceMemoryProperties.memoryTypeCount; i++) {
    if ((typeBits & 1) == 1) {
      if ((deviceMemoryProperties.memoryTypes[i].propertyFlags & properties) ==
          properties) {
        return i;
      }
    }
    typeBits >>= 1;
  }
  return 0;
}

VkResult vrglasses_for_robots::VulkanRenderer::createBuffer(
    VkBufferUsageFlags usageFlags, VkMemoryPropertyFlags memoryPropertyFlags,
    VkBuffer *buffer, VkDeviceMemory *memory, VkDeviceSize size, void *data) {
  // Create the buffer handle
  VkBufferCreateInfo bufferCreateInfo =
      vks::initializers::bufferCreateInfo(usageFlags, size);
  bufferCreateInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
  VK_CHECK_RESULT(vkCreateBuffer(device, &bufferCreateInfo, nullptr, buffer));

  // Create the memory backing up the buffer handle
  VkMemoryRequirements memReqs;
  VkMemoryAllocateInfo memAlloc = vks::initializers::memoryAllocateInfo();
  vkGetBufferMemoryRequirements(device, *buffer, &memReqs);
  memAlloc.allocationSize = memReqs.size;
  memAlloc.memoryTypeIndex =
      getMemoryTypeIndex(memReqs.memoryTypeBits, memoryPropertyFlags);
  VK_CHECK_RESULT(vkAllocateMemory(device, &memAlloc, nullptr, memory));

  if (data != nullptr) {
    void *mapped;
    VK_CHECK_RESULT(vkMapMemory(device, *memory, 0, size, 0, &mapped));
    memcpy(mapped, data, size);
    vkUnmapMemory(device, *memory);
  }

  VK_CHECK_RESULT(vkBindBufferMemory(device, *buffer, *memory, 0));

  return VK_SUCCESS;
}

void vrglasses_for_robots::VulkanRenderer::submitWork(VkCommandBuffer cmdBuffer,
                                                      VkQueue queue) {
  VkSubmitInfo submitInfo = vks::initializers::submitInfo();
  submitInfo.commandBufferCount = 1;
  submitInfo.pCommandBuffers = &cmdBuffer;
  VkFenceCreateInfo fenceInfo = vks::initializers::fenceCreateInfo();
  VkFence fence;
  VK_CHECK_RESULT(vkCreateFence(device, &fenceInfo, nullptr, &fence));
  VK_CHECK_RESULT(vkQueueSubmit(queue, 1, &submitInfo, fence));
  VK_CHECK_RESULT(vkWaitForFences(device, 1, &fence, VK_TRUE, UINT64_MAX));
  vkDestroyFence(device, fence, nullptr);
}

void vrglasses_for_robots::VulkanRenderer::initVulkan(bool enableValidation) {
  VkApplicationInfo appInfo = {};
  appInfo.sType = VK_STRUCTURE_TYPE_APPLICATION_INFO;
  appInfo.pApplicationName = "Mesh2Depth";
  appInfo.pEngineName = "No Engine";
  appInfo.apiVersion = VK_API_VERSION_1_0;

  /*
      Vulkan instance creation (without surface extensions)
*/
  VkInstanceCreateInfo instanceCreateInfo = {};
  instanceCreateInfo.sType = VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO;
  instanceCreateInfo.pApplicationInfo = &appInfo;

  uint32_t layerCount = 0;

  const char *validationLayers[] = {"VK_LAYER_LUNARG_standard_validation"};
  layerCount = 1;
  bool layersAvailable = true;

#if DEBUG
  if (enableValidation) {
    // Check if layers are available
    uint32_t instanceLayerCount;
    vkEnumerateInstanceLayerProperties(&instanceLayerCount, nullptr);
    std::vector<VkLayerProperties> instanceLayers(instanceLayerCount);
    vkEnumerateInstanceLayerProperties(&instanceLayerCount,
                                       instanceLayers.data());

    for (auto layerName : validationLayers) {
      bool layerAvailable = false;
      for (auto instanceLayer : instanceLayers) {
        if (strcmp(instanceLayer.layerName, layerName) == 0) {
          layerAvailable = true;
          break;
        }
      }
      if (!layerAvailable) {
        layersAvailable = false;
        break;
      }
    }

    if (layersAvailable) {
      instanceCreateInfo.ppEnabledLayerNames = validationLayers;
      const char *validationExt = VK_EXT_DEBUG_REPORT_EXTENSION_NAME;
      instanceCreateInfo.enabledLayerCount = layerCount;
      instanceCreateInfo.enabledExtensionCount = 1;
      instanceCreateInfo.ppEnabledExtensionNames = &validationExt;
    }
  }
#endif
  VK_CHECK_RESULT(vkCreateInstance(&instanceCreateInfo, nullptr, &instance));

#if DEBUG
  if (layersAvailable) {
    VkDebugReportCallbackCreateInfoEXT debugReportCreateInfo = {};
    debugReportCreateInfo.sType =
        VK_STRUCTURE_TYPE_DEBUG_REPORT_CALLBACK_CREATE_INFO_EXT;
    debugReportCreateInfo.flags =
        VK_DEBUG_REPORT_ERROR_BIT_EXT | VK_DEBUG_REPORT_WARNING_BIT_EXT;
    debugReportCreateInfo.pfnCallback =
        (PFN_vkDebugReportCallbackEXT)debugMessageCallback;

    // We have to explicitly load this function.
    PFN_vkCreateDebugReportCallbackEXT vkCreateDebugReportCallbackEXT =
        reinterpret_cast<PFN_vkCreateDebugReportCallbackEXT>(
            vkGetInstanceProcAddr(instance, "vkCreateDebugReportCallbackEXT"));
    assert(vkCreateDebugReportCallbackEXT);
    VK_CHECK_RESULT(vkCreateDebugReportCallbackEXT(
        instance, &debugReportCreateInfo, nullptr, &debugReportCallback));
  }
#endif

  /*
          Vulkan device creation
  */
  uint32_t deviceCount = 0;
  VK_CHECK_RESULT(vkEnumeratePhysicalDevices(instance, &deviceCount, nullptr));
  std::vector<VkPhysicalDevice> physicalDevices(deviceCount);
  VK_CHECK_RESULT(vkEnumeratePhysicalDevices(instance, &deviceCount,
                                             physicalDevices.data()));
  physicalDevice = physicalDevices[0];

  VkPhysicalDeviceProperties deviceProperties;
  vkGetPhysicalDeviceProperties(physicalDevice, &deviceProperties);
  std::cout << "GPU: " << deviceProperties.deviceName << "\n";

  // Request a single graphics queue
  const float defaultQueuePriority(0.0f);
  VkDeviceQueueCreateInfo queueCreateInfo = {};
  uint32_t queueFamilyCount;
  vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &queueFamilyCount,
                                           nullptr);
  std::vector<VkQueueFamilyProperties> queueFamilyProperties(queueFamilyCount);
  vkGetPhysicalDeviceQueueFamilyProperties(physicalDevice, &queueFamilyCount,
                                           queueFamilyProperties.data());
  for (uint32_t i = 0; i < static_cast<uint32_t>(queueFamilyProperties.size());
       i++) {
    if (queueFamilyProperties[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) {
      queueFamilyIndex = i;
      queueCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO;
      queueCreateInfo.queueFamilyIndex = i;
      queueCreateInfo.queueCount = 1;
      queueCreateInfo.pQueuePriorities = &defaultQueuePriority;
      break;
    }
  }
  // Create logical device
  VkDeviceCreateInfo deviceCreateInfo = {};
  deviceCreateInfo.sType = VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO;
  deviceCreateInfo.queueCreateInfoCount = 1;
  deviceCreateInfo.pQueueCreateInfos = &queueCreateInfo;
  VK_CHECK_RESULT(
      vkCreateDevice(physicalDevice, &deviceCreateInfo, nullptr, &device));

  // Get a graphics queue
  vkGetDeviceQueue(device, queueFamilyIndex, 0, &queue);

  // Command pool
  VkCommandPoolCreateInfo cmdPoolInfo = {};
  cmdPoolInfo.sType = VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO;
  cmdPoolInfo.queueFamilyIndex = queueFamilyIndex;
  cmdPoolInfo.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
  VK_CHECK_RESULT(
      vkCreateCommandPool(device, &cmdPoolInfo, nullptr, &commandPool));
}

void vrglasses_for_robots::VulkanRenderer::buildRenderPass(uint32_t width,
                                                           uint32_t height) {
  // vks::tools::getSupportedDepthFormat(physicalDevice, &depthFormat);
  {
    // Color attachment
    VkImageCreateInfo image = vks::initializers::imageCreateInfo();
    image.imageType = VK_IMAGE_TYPE_2D;
    image.format = colorFormat;
    image.extent.width = width;
    image.extent.height = height;
    image.extent.depth = 1;
    image.mipLevels = 1;
    image.arrayLayers = 1;
    image.samples = VK_SAMPLE_COUNT_1_BIT;
    image.tiling = VK_IMAGE_TILING_OPTIMAL;
    image.usage =
        VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT;

    VkMemoryAllocateInfo memAlloc = vks::initializers::memoryAllocateInfo();
    VkMemoryRequirements memReqs;

    VK_CHECK_RESULT(
        vkCreateImage(device, &image, nullptr, &colorAttachment.image));
    vkGetImageMemoryRequirements(device, colorAttachment.image, &memReqs);
    memAlloc.allocationSize = memReqs.size;
    memAlloc.memoryTypeIndex = getMemoryTypeIndex(
        memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    VK_CHECK_RESULT(
        vkAllocateMemory(device, &memAlloc, nullptr, &colorAttachment.memory));
    VK_CHECK_RESULT(vkBindImageMemory(device, colorAttachment.image,
                                      colorAttachment.memory, 0));

    VkImageViewCreateInfo colorImageView =
        vks::initializers::imageViewCreateInfo();
    colorImageView.viewType = VK_IMAGE_VIEW_TYPE_2D;
    colorImageView.format = colorFormat;
    colorImageView.subresourceRange = {};
    colorImageView.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    colorImageView.subresourceRange.baseMipLevel = 0;
    colorImageView.subresourceRange.levelCount = 1;
    colorImageView.subresourceRange.baseArrayLayer = 0;
    colorImageView.subresourceRange.layerCount = 1;
    colorImageView.image = colorAttachment.image;
    VK_CHECK_RESULT(vkCreateImageView(device, &colorImageView, nullptr,
                                      &colorAttachment.view));

    // Depth stencil attachment (reuse all configuration, but format and usage)
    image.format = depthFormat;
    image.usage = VK_IMAGE_USAGE_DEPTH_STENCIL_ATTACHMENT_BIT |
                  VK_IMAGE_USAGE_TRANSFER_SRC_BIT;

    VK_CHECK_RESULT(
        vkCreateImage(device, &image, nullptr, &depthAttachment.image));
    vkGetImageMemoryRequirements(device, depthAttachment.image, &memReqs);
    memAlloc.allocationSize = memReqs.size;
    memAlloc.memoryTypeIndex = getMemoryTypeIndex(
        memReqs.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
    VK_CHECK_RESULT(
        vkAllocateMemory(device, &memAlloc, nullptr, &depthAttachment.memory));
    VK_CHECK_RESULT(vkBindImageMemory(device, depthAttachment.image,
                                      depthAttachment.memory, 0));

    VkImageViewCreateInfo depthStencilView =
        vks::initializers::imageViewCreateInfo();
    depthStencilView.viewType = VK_IMAGE_VIEW_TYPE_2D;
    depthStencilView.format = depthFormat;
    depthStencilView.flags = 0;
    depthStencilView.subresourceRange = {};
    depthStencilView.subresourceRange.aspectMask =
        VK_IMAGE_ASPECT_DEPTH_BIT | VK_IMAGE_ASPECT_STENCIL_BIT;
    depthStencilView.subresourceRange.baseMipLevel = 0;
    depthStencilView.subresourceRange.levelCount = 1;
    depthStencilView.subresourceRange.baseArrayLayer = 0;
    depthStencilView.subresourceRange.layerCount = 1;
    depthStencilView.image = depthAttachment.image;
    VK_CHECK_RESULT(vkCreateImageView(device, &depthStencilView, nullptr,
                                      &depthAttachment.view));
  }

  /*
        Create renderpass
*/
  {
    std::array<VkAttachmentDescription, 2> attchmentDescriptions = {};
    // Color attachment
    attchmentDescriptions[0].format = colorFormat;
    attchmentDescriptions[0].samples = VK_SAMPLE_COUNT_1_BIT;
    attchmentDescriptions[0].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    attchmentDescriptions[0].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    attchmentDescriptions[0].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attchmentDescriptions[0].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attchmentDescriptions[0].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attchmentDescriptions[0].finalLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;
    // Depth attachment
    attchmentDescriptions[1].format = depthFormat;
    attchmentDescriptions[1].samples = VK_SAMPLE_COUNT_1_BIT;
    attchmentDescriptions[1].loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
    attchmentDescriptions[1].storeOp = VK_ATTACHMENT_STORE_OP_STORE;
    attchmentDescriptions[1].stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE;
    attchmentDescriptions[1].stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
    attchmentDescriptions[1].initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    attchmentDescriptions[1].finalLayout = VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL;

    VkAttachmentReference colorReference = {
        0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
    VkAttachmentReference depthReference = {
        1, VK_IMAGE_LAYOUT_DEPTH_STENCIL_ATTACHMENT_OPTIMAL};

    VkSubpassDescription subpassDescription = {};
    subpassDescription.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS;
    subpassDescription.colorAttachmentCount = 1;
    subpassDescription.pColorAttachments = &colorReference;
    subpassDescription.pDepthStencilAttachment = &depthReference;

    // Use subpass dependencies for layout transitions
    std::array<VkSubpassDependency, 2> dependencies;

    dependencies[0].srcSubpass = VK_SUBPASS_EXTERNAL;
    dependencies[0].dstSubpass = 0;
    dependencies[0].srcStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
    dependencies[0].dstStageMask =
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[0].srcAccessMask = VK_ACCESS_MEMORY_READ_BIT;
    dependencies[0].dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
                                    VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependencies[0].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

    dependencies[1].srcSubpass = 0;
    dependencies[1].dstSubpass = VK_SUBPASS_EXTERNAL;
    dependencies[1].srcStageMask =
        VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
    dependencies[1].dstStageMask = VK_PIPELINE_STAGE_BOTTOM_OF_PIPE_BIT;
    dependencies[1].srcAccessMask = VK_ACCESS_COLOR_ATTACHMENT_READ_BIT |
                                    VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
    dependencies[1].dstAccessMask = VK_ACCESS_MEMORY_READ_BIT;
    dependencies[1].dependencyFlags = VK_DEPENDENCY_BY_REGION_BIT;

    // Create the actual renderpass
    VkRenderPassCreateInfo renderPassInfo = {};
    renderPassInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO;
    renderPassInfo.attachmentCount =
        static_cast<uint32_t>(attchmentDescriptions.size());
    renderPassInfo.pAttachments = attchmentDescriptions.data();
    renderPassInfo.subpassCount = 1;
    renderPassInfo.pSubpasses = &subpassDescription;
    renderPassInfo.dependencyCount = static_cast<uint32_t>(dependencies.size());
    renderPassInfo.pDependencies = dependencies.data();
    VK_CHECK_RESULT(
        vkCreateRenderPass(device, &renderPassInfo, nullptr, &renderPass));

    VkImageView attachments[2];
    attachments[0] = colorAttachment.view;
    attachments[1] = depthAttachment.view;

    VkFramebufferCreateInfo framebufferCreateInfo =
        vks::initializers::framebufferCreateInfo();
    framebufferCreateInfo.renderPass = renderPass;
    framebufferCreateInfo.attachmentCount = 2;
    framebufferCreateInfo.pAttachments = attachments;
    framebufferCreateInfo.width = width;
    framebufferCreateInfo.height = height;
    framebufferCreateInfo.layers = 1;
    VK_CHECK_RESULT(vkCreateFramebuffer(device, &framebufferCreateInfo, nullptr,
                                        &framebuffer));
  }

  /*
          Prepare graphics pipeline
  */
  {
    std::vector<VkDescriptorSetLayoutBinding> setLayoutBindings = {
        // Binding 0 : Fragment shader image sampler
        vks::initializers::descriptorSetLayoutBinding(
            VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
            VK_SHADER_STAGE_FRAGMENT_BIT, 0)};

    VkDescriptorSetLayoutCreateInfo descriptorLayout =
        vks::initializers::descriptorSetLayoutCreateInfo(setLayoutBindings);
    VK_CHECK_RESULT(vkCreateDescriptorSetLayout(device, &descriptorLayout,
                                                nullptr, &descriptorSetLayout));

    VkPipelineLayoutCreateInfo pipelineLayoutCreateInfo =
        vks::initializers::pipelineLayoutCreateInfo(&descriptorSetLayout, 1);

    // MVP via push constant block
    VkPushConstantRange pushConstantRange =
        vks::initializers::pushConstantRange(VK_SHADER_STAGE_VERTEX_BIT,
                                             sizeof(glm::mat4), 0);
    pipelineLayoutCreateInfo.pushConstantRangeCount = 1;
    pipelineLayoutCreateInfo.pPushConstantRanges = &pushConstantRange;

    VK_CHECK_RESULT(vkCreatePipelineLayout(device, &pipelineLayoutCreateInfo,
                                           nullptr, &pipelineLayout));

    VkPipelineCacheCreateInfo pipelineCacheCreateInfo = {};
    pipelineCacheCreateInfo.sType =
        VK_STRUCTURE_TYPE_PIPELINE_CACHE_CREATE_INFO;
    VK_CHECK_RESULT(vkCreatePipelineCache(device, &pipelineCacheCreateInfo,
                                          nullptr, &pipelineCache));

    // Create pipeline
    VkPipelineInputAssemblyStateCreateInfo inputAssemblyState =
        vks::initializers::pipelineInputAssemblyStateCreateInfo(
            VK_PRIMITIVE_TOPOLOGY_TRIANGLE_LIST, 0, VK_FALSE);

    VkPipelineRasterizationStateCreateInfo rasterizationState =
        vks::initializers::pipelineRasterizationStateCreateInfo(
            VK_POLYGON_MODE_FILL, VK_CULL_MODE_BACK_BIT,
            VK_FRONT_FACE_COUNTER_CLOCKWISE);

    VkPipelineColorBlendAttachmentState blendAttachmentState =
        vks::initializers::pipelineColorBlendAttachmentState(0xf, VK_FALSE);

    VkPipelineColorBlendStateCreateInfo colorBlendState =
        vks::initializers::pipelineColorBlendStateCreateInfo(
            1, &blendAttachmentState);

    VkPipelineDepthStencilStateCreateInfo depthStencilState =
        vks::initializers::pipelineDepthStencilStateCreateInfo(
            VK_TRUE, VK_TRUE, VK_COMPARE_OP_LESS_OR_EQUAL);

    VkPipelineViewportStateCreateInfo viewportState =
        vks::initializers::pipelineViewportStateCreateInfo(1, 1);

    VkPipelineMultisampleStateCreateInfo multisampleState =
        vks::initializers::pipelineMultisampleStateCreateInfo(
            VK_SAMPLE_COUNT_1_BIT);

    std::vector<VkDynamicState> dynamicStateEnables = {
        VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
    VkPipelineDynamicStateCreateInfo dynamicState =
        vks::initializers::pipelineDynamicStateCreateInfo(dynamicStateEnables);

    VkGraphicsPipelineCreateInfo pipelineCreateInfo =
        vks::initializers::pipelineCreateInfo(pipelineLayout, renderPass);

    std::array<VkPipelineShaderStageCreateInfo, 2> shaderStages{};

    pipelineCreateInfo.pInputAssemblyState = &inputAssemblyState;
    pipelineCreateInfo.pRasterizationState = &rasterizationState;
    pipelineCreateInfo.pColorBlendState = &colorBlendState;
    pipelineCreateInfo.pMultisampleState = &multisampleState;
    pipelineCreateInfo.pViewportState = &viewportState;
    pipelineCreateInfo.pDepthStencilState = &depthStencilState;
    pipelineCreateInfo.pDynamicState = &dynamicState;
    pipelineCreateInfo.stageCount = static_cast<uint32_t>(shaderStages.size());
    pipelineCreateInfo.pStages = shaderStages.data();

    // Vertex bindings an attributes
    // Binding description
    std::vector<VkVertexInputBindingDescription> vertexInputBindings = {
        Vertex::getBindingDescription()};

    // Attribute descriptions
    std::array<VkVertexInputAttributeDescription, 2> vertexInputAttributes =
        Vertex::getAttributeDescriptions();

    VkPipelineVertexInputStateCreateInfo vertexInputState =
        vks::initializers::pipelineVertexInputStateCreateInfo();
    vertexInputState.vertexBindingDescriptionCount =
        static_cast<uint32_t>(vertexInputBindings.size());
    vertexInputState.pVertexBindingDescriptions = vertexInputBindings.data();
    vertexInputState.vertexAttributeDescriptionCount =
        static_cast<uint32_t>(vertexInputAttributes.size());
    vertexInputState.pVertexAttributeDescriptions =
        vertexInputAttributes.data();

    pipelineCreateInfo.pVertexInputState = &vertexInputState;

    shaderStages[0].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shaderStages[0].stage = VK_SHADER_STAGE_VERTEX_BIT;
    shaderStages[0].pName = "main";
    shaderStages[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
    shaderStages[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT;
    shaderStages[1].pName = "main";

    shaderStages[0].module =
        vks::tools::loadShader(shader_vert_spv_.c_str(), device);
    shaderStages[1].module =
        vks::tools::loadShader(shader_frag_spv_.c_str(), device);

    shaderModules = {shaderStages[0].module, shaderStages[1].module};
    VK_CHECK_RESULT(vkCreateGraphicsPipelines(
        device, pipelineCache, 1, &pipelineCreateInfo, nullptr, &pipeline));
  }

  // create buffer for copy
  {
    // Create the linear tiled destination image to copy to and to read the
    // memory from
    VkImageCreateInfo imgCreateInfo(vks::initializers::imageCreateInfo());
    imgCreateInfo.imageType = VK_IMAGE_TYPE_2D;
    imgCreateInfo.format = colorFormat;
    imgCreateInfo.extent.width = width;
    imgCreateInfo.extent.height = height;
    imgCreateInfo.extent.depth = 1;
    imgCreateInfo.arrayLayers = 1;
    imgCreateInfo.mipLevels = 1;
    imgCreateInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
    imgCreateInfo.samples = VK_SAMPLE_COUNT_1_BIT;
    imgCreateInfo.tiling = VK_IMAGE_TILING_LINEAR;
    imgCreateInfo.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT;

    // Create the image
    VK_CHECK_RESULT(vkCreateImage(device, &imgCreateInfo, nullptr, &dstImage));
    // Create memory to back up the image
    VkMemoryRequirements memRequirements;
    VkMemoryAllocateInfo memAllocInfo(vks::initializers::memoryAllocateInfo());
    vkGetImageMemoryRequirements(device, dstImage, &memRequirements);
    memAllocInfo.allocationSize = memRequirements.size;
    // Memory must be host visible to copy from
    memAllocInfo.memoryTypeIndex =
        getMemoryTypeIndex(memRequirements.memoryTypeBits,
                           VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                               VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    VK_CHECK_RESULT(
        vkAllocateMemory(device, &memAllocInfo, nullptr, &dstImageMemory));
    VK_CHECK_RESULT(vkBindImageMemory(device, dstImage, dstImageMemory, 0));

    VkDeviceSize mem_size = height * width * 4;  // assuming D32
    createBuffer(VK_BUFFER_USAGE_TRANSFER_DST_BIT,
                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                     VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                 &image_buffer, &image_buffer_memory, mem_size);
  }
}

void vrglasses_for_robots::VulkanRenderer::drawTriangles(uint32_t width,
                                                         uint32_t height) {
  /*
          Command buffer creation
  */
  {
    VkCommandBuffer commandBuffer;
    VkCommandBufferAllocateInfo cmdBufAllocateInfo =
        vks::initializers::commandBufferAllocateInfo(
            commandPool, VK_COMMAND_BUFFER_LEVEL_PRIMARY, 1);
    VK_CHECK_RESULT(
        vkAllocateCommandBuffers(device, &cmdBufAllocateInfo, &commandBuffer));

    VkCommandBufferBeginInfo cmdBufInfo =
        vks::initializers::commandBufferBeginInfo();

    VK_CHECK_RESULT(vkBeginCommandBuffer(commandBuffer, &cmdBufInfo));

    VkClearValue clearValues[2];
    clearValues[0].color = {{0.92f, 0.81f, 0.53f, 0.0f}};
    clearValues[1].depthStencil = {1.0f, 0};

    VkRenderPassBeginInfo renderPassBeginInfo = {};
    renderPassBeginInfo.sType = VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO;
    renderPassBeginInfo.renderArea.extent.width = width;
    renderPassBeginInfo.renderArea.extent.height = height;
    renderPassBeginInfo.clearValueCount = 2;
    renderPassBeginInfo.pClearValues = clearValues;
    renderPassBeginInfo.renderPass = renderPass;
    renderPassBeginInfo.framebuffer = framebuffer;

    vkCmdBeginRenderPass(commandBuffer, &renderPassBeginInfo,
                         VK_SUBPASS_CONTENTS_INLINE);

    VkViewport viewport = {};
    viewport.height = (float)height;
    viewport.width = (float)width;
    viewport.minDepth = (float)0.0f;
    viewport.maxDepth = (float)1.0f;
    vkCmdSetViewport(commandBuffer, 0, 1, &viewport);

    // Update dynamic scissor state
    VkRect2D scissor = {};
    scissor.extent.width = width;
    scissor.extent.height = height;
    vkCmdSetScissor(commandBuffer, 0, 1, &scissor);

    vkCmdBindPipeline(commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipeline);

    // Render scene
    VkDeviceSize offsets[1] = {0};
    vkCmdBindVertexBuffers(commandBuffer, 0, 1, &vertexBuffer, offsets);
    vkCmdBindIndexBuffer(commandBuffer, indexBuffer, 0, VK_INDEX_TYPE_UINT32);

    for (size_t idx = 0; idx < scene_items_.size(); idx++) {
      glm::mat4 mvp_cv = vp_cv_ * scene_items_[idx].T_World2Model;

      vkCmdPushConstants(commandBuffer, pipelineLayout,
                         VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(mvp_cv),
                         &mvp_cv);
      size_t model_idx = models_index_[scene_items_[idx].model_name];
      vkCmdBindDescriptorSets(
          commandBuffer, VK_PIPELINE_BIND_POINT_GRAPHICS, pipelineLayout, 0, 1,
          &textures_[models_[model_idx].material_index].descriptorSet, 0,
          nullptr);

      vkCmdDrawIndexed(commandBuffer, models_[model_idx].begin_vertex_count, 1,
                       models_[model_idx].begin_vertex_index, 0, 0);
    }

    vkCmdEndRenderPass(commandBuffer);

    VK_CHECK_RESULT(vkEndCommandBuffer(commandBuffer));

    submitWork(commandBuffer, queue);

    vkDeviceWaitIdle(device);
  }
}

// -------------------------------
// convert from a zBuffer value into the corresponding depth value
// apart from value range transfer from [-1,1] to [0,1] and the perspective
// division done by opengl
// this function inverts the projection and NDC transform for the z-value
// near: distance to near clip plane
// far: distance to far clip plane
// zValue: value in depth buffer
// http://lists.apple.com/archives/mac-opengl/2005/Oct/msg00063.html
// -------------------------------

float vrglasses_for_robots::VulkanRenderer::convertZbufferToDepth(
    float near, float far, float zValue) {
  // float wz = 2.0 * zValue - 1.0;  // Convert Z from [0, 1] to [-1, 1] This
  // step should not be used in vulkan
  return 2.0 * near * far / (far + near - zValue * (far - near));
}

void vrglasses_for_robots::VulkanRenderer::saveImageDepthmap(
    uint32_t width, uint32_t height, cv::Mat &result_depth_map,
    cv::Mat &result_attribute_map) {
  result_depth_map.create(height, width, CV_32F);
  result_attribute_map.create(height, width, CV_8UC4);
  /*
          Copy framebuffer image to host visible image
  */
  const char *imagedata;
  {
    // Do the actual blit from the offscreen image to our host visible
    // destination image
    VkCommandBufferAllocateInfo cmdBufAllocateInfo =
        vks::initializers::commandBufferAllocateInfo(
            commandPool, VK_COMMAND_BUFFER_LEVEL_PRIMARY, 1);
    VkCommandBuffer copyCmd;
    VK_CHECK_RESULT(
        vkAllocateCommandBuffers(device, &cmdBufAllocateInfo, &copyCmd));
    VkCommandBufferBeginInfo cmdBufInfo =
        vks::initializers::commandBufferBeginInfo();
    VK_CHECK_RESULT(vkBeginCommandBuffer(copyCmd, &cmdBufInfo));

    VkImageMemoryBarrier imageMemoryBarrier =
        vks::initializers::imageMemoryBarrier();

    // Transition destination image to transfer destination layout
    vks::tools::insertImageMemoryBarrier(
        copyCmd, dstImage, 0, VK_ACCESS_TRANSFER_WRITE_BIT,
        VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
        VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT,
        VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1});

    // colorAttachment.image is already in
    // VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, and does not need to be
    // transitioned

    VkImageCopy imageCopyRegion{};
    imageCopyRegion.srcSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    imageCopyRegion.srcSubresource.layerCount = 1;
    imageCopyRegion.dstSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    imageCopyRegion.dstSubresource.layerCount = 1;
    imageCopyRegion.extent.width = width;
    imageCopyRegion.extent.height = height;
    imageCopyRegion.extent.depth = 1;

    vkCmdCopyImage(copyCmd, colorAttachment.image,
                   VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, dstImage,
                   VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &imageCopyRegion);

    // Transition destination image to general layout, which is the required
    // layout for mapping the image memory later on
    vks::tools::insertImageMemoryBarrier(
        copyCmd, dstImage, VK_ACCESS_TRANSFER_WRITE_BIT,
        VK_ACCESS_MEMORY_READ_BIT, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
        VK_IMAGE_LAYOUT_GENERAL, VK_PIPELINE_STAGE_TRANSFER_BIT,
        VK_PIPELINE_STAGE_TRANSFER_BIT,
        VkImageSubresourceRange{VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1});

    VkPhysicalDeviceMemoryProperties memory_properties;
    vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memory_properties);

    VkBufferImageCopy region = {};
    region.bufferOffset = 0;
    region.bufferRowLength = 0;
    region.bufferImageHeight = 0;
    region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_DEPTH_BIT;
    region.imageSubresource.mipLevel = 0;
    region.imageSubresource.baseArrayLayer = 0;
    region.imageSubresource.layerCount = 1;
    region.imageOffset = (VkOffset3D){0, 0, 0};
    region.imageExtent = (VkExtent3D){width, height, 1};

    vkCmdCopyImageToBuffer(copyCmd, depthAttachment.image,
                           VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL, image_buffer,
                           1, &region);

    VK_CHECK_RESULT(vkEndCommandBuffer(copyCmd));

    submitWork(copyCmd, queue);

    // Get layout of the image (including row pitch)
    VkImageSubresource subResource{};
    subResource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
    VkSubresourceLayout subResourceLayout;

    vkGetImageSubresourceLayout(device, dstImage, &subResource,
                                &subResourceLayout);

    // Map image memory so we can start copying from it
    vkMapMemory(device, dstImageMemory, 0, VK_WHOLE_SIZE, 0,
                (void **)&imagedata);
    imagedata += subResourceLayout.offset;

    {
      VkDeviceSize mem_size = height * width * 4;  // assuming 8UC4
      memcpy(result_attribute_map.data, imagedata, mem_size);
    }

    /*
          Save host visible framebuffer image to disk (ppm format)
  */
    if (/* DISABLES CODE */ (0)) {
      const char *filename = "headless.ppm";

      std::ofstream file(filename, std::ios::out | std::ios::binary);

      // ppm header
      file << "P6\n" << width << "\n" << height << "\n" << 255 << "\n";

      // If source is BGR (destination is always RGB) and we can't use blit
      // (which does automatic conversion), we'll have to manually swizzle
      // color components
      bool colorSwizzle = false;
      // Check if source is BGR and needs swizzle
      std::vector<VkFormat> formatsBGR = {VK_FORMAT_B8G8R8A8_SRGB,
                                          VK_FORMAT_B8G8R8A8_UNORM,
                                          VK_FORMAT_B8G8R8A8_SNORM};
      colorSwizzle = (std::find(formatsBGR.begin(), formatsBGR.end(),
                                colorFormat) != formatsBGR.end());

      // ppm binary pixel data
      for (int32_t y = 0; y < height; y++) {
        unsigned int *row = (unsigned int *)imagedata;
        for (int32_t x = 0; x < width; x++) {
          if (colorSwizzle) {
            file.write((char *)row + 2, 1);
            file.write((char *)row + 1, 1);
            file.write((char *)row, 1);
          } else {
            file.write((char *)row, 3);
          }
          row++;
        }
        imagedata += subResourceLayout.rowPitch;
      }
      file.close();
    }
    vkUnmapMemory(device, dstImageMemory);

    {
      void *mapped;
      VkDeviceSize mem_size = height * width * 4;  // assuming D32
      VK_CHECK_RESULT(
          vkMapMemory(device, image_buffer_memory, 0, mem_size, 0, &mapped));

      memcpy(result_depth_map.data, mapped, mem_size);
      vkUnmapMemory(device, image_buffer_memory);

      // 2.0 * near* far / (far + near - zValue * (far - near));

      //      result_depth_map.convertTo(result_depth_map, CV_32F, 2, -1);
      result_depth_map.convertTo(result_depth_map, CV_32F,
                                 -1.0 * static_cast<double>(far_ - near_),
                                 static_cast<double>(far_ + near_));

      result_depth_map = (2.0f * far_ * near_) / result_depth_map;

      // cv::flip(result_depth_map, result_depth_map, 0);
      cv::threshold(result_depth_map, result_depth_map, far_ - 0.0001, far_,
                    cv::THRESH_TOZERO_INV);
    }
    // LOG("Framebuffer image saved to %s\n", filename);
  }
}

void vrglasses_for_robots::VulkanRenderer::setupDescriptorPool() {
  // Example uses one ubo and one image sampler
  std::vector<VkDescriptorPoolSize> poolSizes = {
      vks::initializers::descriptorPoolSize(
          VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,
          static_cast<uint32_t>(textures_.size()))};

  VkDescriptorPoolCreateInfo descriptorPoolInfo =
      vks::initializers::descriptorPoolCreateInfo(
          static_cast<uint32_t>(poolSizes.size()), poolSizes.data(),
          static_cast<uint32_t>(textures_.size()) + 1);

  VK_CHECK_RESULT(vkCreateDescriptorPool(device, &descriptorPoolInfo, nullptr,
                                         &descriptorPool));
}

void vrglasses_for_robots::VulkanRenderer::setupDescriptorSet(Texture2D &tex) {
  VkDescriptorSetAllocateInfo allocInfo =
      vks::initializers::descriptorSetAllocateInfo(descriptorPool,
                                                   &descriptorSetLayout, 1);

  VK_CHECK_RESULT(
      vkAllocateDescriptorSets(device, &allocInfo, &(tex.descriptorSet)));

  // Setup a descriptor image info for the current texture to be used as a
  // combined image sampler
  VkDescriptorImageInfo textureDescriptor;
  textureDescriptor.imageView =
      tex.textureImageView;  // The image's view (images are never directly accessed
                             // by the shader, but rather through views defining
                             // subresources)
  textureDescriptor.sampler =
      tex.textureSampler;  // The sampler (Telling the pipeline how to sample the
                           // texture, including repeat, border, etc.)
  textureDescriptor.imageLayout =
      VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL;  // The current layout of the
                                                 // image (Note: Should always
                                                 // fit the actual use, e.g.
                                                 // shader read)

  std::vector<VkWriteDescriptorSet> writeDescriptorSets = {
      // Binding 1 : Fragment shader texture sampler
      //	Fragment shader: layout (binding = 0) uniform sampler2D
      // samplerColor;
      vks::initializers::writeDescriptorSet(
          tex.descriptorSet,
          VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER,  // The descriptor set will
                                                      // use a combined image
                                                      // sampler (sampler and
                                                      // image could be split)
          0,                                          // Shader binding point 0
          &textureDescriptor)  // Pointer to the descriptor image for our texture
  };

  vkUpdateDescriptorSets(device,
                         static_cast<uint32_t>(writeDescriptorSets.size()),
                         writeDescriptorSets.data(), 0, NULL);
}

void vrglasses_for_robots::VulkanRenderer::createBuffer(
    VkDeviceSize size, VkBufferUsageFlags usage,
    VkMemoryPropertyFlags properties, VkBuffer &buffer,
    VkDeviceMemory &bufferMemory) {
  VkBufferCreateInfo bufferInfo = {};
  bufferInfo.sType = VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO;
  bufferInfo.size = size;
  bufferInfo.usage = usage;
  bufferInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  if (vkCreateBuffer(device, &bufferInfo, nullptr, &buffer) != VK_SUCCESS) {
    throw std::runtime_error("failed to create buffer!");
  }

  VkMemoryRequirements memRequirements;
  vkGetBufferMemoryRequirements(device, buffer, &memRequirements);

  VkMemoryAllocateInfo allocInfo = {};
  allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  allocInfo.allocationSize = memRequirements.size;
  allocInfo.memoryTypeIndex =
      findMemoryType(memRequirements.memoryTypeBits, properties);

  if (vkAllocateMemory(device, &allocInfo, nullptr, &bufferMemory) !=
      VK_SUCCESS) {
    throw std::runtime_error("failed to allocate buffer memory!");
  }

  vkBindBufferMemory(device, buffer, bufferMemory, 0);
}

uint32_t vrglasses_for_robots::VulkanRenderer::findMemoryType(
    uint32_t typeFilter, VkMemoryPropertyFlags properties) {
  VkPhysicalDeviceMemoryProperties memProperties;
  vkGetPhysicalDeviceMemoryProperties(physicalDevice, &memProperties);

  for (uint32_t i = 0; i < memProperties.memoryTypeCount; i++) {
    if ((typeFilter & (1 << i)) && (memProperties.memoryTypes[i].propertyFlags &
                                    properties) == properties) {
      return i;
    }
  }

  throw std::runtime_error("failed to find suitable memory type!");
}

void vrglasses_for_robots::VulkanRenderer::createTextureImage(
    Texture2D &tex, std::string filename_texture) {
  int texWidth, texHeight, texChannels;
  stbi_uc *pixels = stbi_load(filename_texture.c_str(), &texWidth, &texHeight,
                              &texChannels, STBI_rgb_alpha);
  VkDeviceSize imageSize = texWidth * texHeight * 4;

  if (!pixels) {
    throw std::runtime_error("failed to load texture image! <" +
                             filename_texture + ">");
  }

  VkBuffer stagingBuffer;
  VkDeviceMemory stagingBufferMemory;
  createBuffer(imageSize, VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
               VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                   VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
               stagingBuffer, stagingBufferMemory);

  void *data;
  vkMapMemory(device, stagingBufferMemory, 0, imageSize, 0, &data);
  memcpy(data, pixels, static_cast<size_t>(imageSize));
  vkUnmapMemory(device, stagingBufferMemory);

  stbi_image_free(pixels);

  createImage(texWidth, texHeight, VK_FORMAT_R8G8B8A8_UNORM,
              VK_IMAGE_TILING_OPTIMAL,
              VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_SAMPLED_BIT,
              VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, tex.textureImage,
              tex.textureImageMemory);

  transitionImageLayout(tex.textureImage, VK_FORMAT_R8G8B8A8_UNORM,
                        VK_IMAGE_LAYOUT_UNDEFINED,
                        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL);
  copyBufferToImage(stagingBuffer, tex.textureImage,
                    static_cast<uint32_t>(texWidth),
                    static_cast<uint32_t>(texHeight));
  transitionImageLayout(tex.textureImage, VK_FORMAT_R8G8B8A8_UNORM,
                        VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL,
                        VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);

  vkDestroyBuffer(device, stagingBuffer, nullptr);
  vkFreeMemory(device, stagingBufferMemory, nullptr);

  //================================

  tex.textureImageView = createImageView(
      tex.textureImage, VK_FORMAT_R8G8B8A8_UNORM, VK_IMAGE_ASPECT_COLOR_BIT);

  //================================

  VkSamplerCreateInfo samplerInfo = {};
  samplerInfo.sType = VK_STRUCTURE_TYPE_SAMPLER_CREATE_INFO;
  samplerInfo.magFilter = VK_FILTER_NEAREST;
  samplerInfo.minFilter = VK_FILTER_NEAREST;
  samplerInfo.addressModeU = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  samplerInfo.addressModeV = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  samplerInfo.addressModeW = VK_SAMPLER_ADDRESS_MODE_REPEAT;
  samplerInfo.anisotropyEnable = VK_FALSE;
  samplerInfo.maxAnisotropy = 16;
  samplerInfo.borderColor = VK_BORDER_COLOR_INT_OPAQUE_BLACK;
  samplerInfo.unnormalizedCoordinates = VK_FALSE;
  samplerInfo.compareEnable = VK_FALSE;
  samplerInfo.compareOp = VK_COMPARE_OP_ALWAYS;
  samplerInfo.mipmapMode = VK_SAMPLER_MIPMAP_MODE_NEAREST;

  if (vkCreateSampler(device, &samplerInfo, nullptr, &tex.textureSampler) !=
      VK_SUCCESS) {
    throw std::runtime_error("failed to create texture sampler!");
  }
}

void vrglasses_for_robots::VulkanRenderer::copyBufferToImage(VkBuffer buffer,
                                                             VkImage image,
                                                             uint32_t width,
                                                             uint32_t height) {
  VkCommandBuffer commandBuffer = beginSingleTimeCommands();

  VkBufferImageCopy region = {};
  region.bufferOffset = 0;
  region.bufferRowLength = 0;
  region.bufferImageHeight = 0;
  region.imageSubresource.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  region.imageSubresource.mipLevel = 0;
  region.imageSubresource.baseArrayLayer = 0;
  region.imageSubresource.layerCount = 1;
  region.imageOffset = {0, 0, 0};
  region.imageExtent = {width, height, 1};

  vkCmdCopyBufferToImage(commandBuffer, buffer, image,
                         VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &region);

  endSingleTimeCommands(commandBuffer);
}

void vrglasses_for_robots::VulkanRenderer::createImage(
    uint32_t width, uint32_t height, VkFormat format, VkImageTiling tiling,
    VkImageUsageFlags usage, VkMemoryPropertyFlags properties, VkImage &image,
    VkDeviceMemory &imageMemory) {
  VkImageCreateInfo imageInfo = {};
  imageInfo.sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO;
  imageInfo.imageType = VK_IMAGE_TYPE_2D;
  imageInfo.extent.width = width;
  imageInfo.extent.height = height;
  imageInfo.extent.depth = 1;
  imageInfo.mipLevels = 1;
  imageInfo.arrayLayers = 1;
  imageInfo.format = format;
  imageInfo.tiling = tiling;
  imageInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  imageInfo.usage = usage;
  imageInfo.samples = VK_SAMPLE_COUNT_1_BIT;
  imageInfo.sharingMode = VK_SHARING_MODE_EXCLUSIVE;

  if (vkCreateImage(device, &imageInfo, nullptr, &image) != VK_SUCCESS) {
    throw std::runtime_error("failed to create image!");
  }

  VkMemoryRequirements memRequirements;
  vkGetImageMemoryRequirements(device, image, &memRequirements);

  VkMemoryAllocateInfo allocInfo = {};
  allocInfo.sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO;
  allocInfo.allocationSize = memRequirements.size;
  allocInfo.memoryTypeIndex =
      findMemoryType(memRequirements.memoryTypeBits, properties);

  if (vkAllocateMemory(device, &allocInfo, nullptr, &imageMemory) !=
      VK_SUCCESS) {
    throw std::runtime_error("failed to allocate image memory!");
  }

  vkBindImageMemory(device, image, imageMemory, 0);
}

void vrglasses_for_robots::VulkanRenderer::transitionImageLayout(
    VkImage image, VkFormat format, VkImageLayout oldLayout,
    VkImageLayout newLayout) {
  VkCommandBuffer commandBuffer = beginSingleTimeCommands();

  VkImageMemoryBarrier barrier = {};
  barrier.sType = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
  barrier.oldLayout = oldLayout;
  barrier.newLayout = newLayout;
  barrier.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  barrier.image = image;
  barrier.subresourceRange.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  barrier.subresourceRange.baseMipLevel = 0;
  barrier.subresourceRange.levelCount = 1;
  barrier.subresourceRange.baseArrayLayer = 0;
  barrier.subresourceRange.layerCount = 1;

  VkPipelineStageFlags sourceStage;
  VkPipelineStageFlags destinationStage;

  if (oldLayout == VK_IMAGE_LAYOUT_UNDEFINED &&
      newLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL) {
    barrier.srcAccessMask = 0;
    barrier.dstAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;

    sourceStage = VK_PIPELINE_STAGE_TOP_OF_PIPE_BIT;
    destinationStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
  } else if (oldLayout == VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL &&
             newLayout == VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL) {
    barrier.srcAccessMask = VK_ACCESS_TRANSFER_WRITE_BIT;
    barrier.dstAccessMask = VK_ACCESS_SHADER_READ_BIT;

    sourceStage = VK_PIPELINE_STAGE_TRANSFER_BIT;
    destinationStage = VK_PIPELINE_STAGE_FRAGMENT_SHADER_BIT;
  } else {
    throw std::invalid_argument("unsupported layout transition!");
  }

  vkCmdPipelineBarrier(commandBuffer, sourceStage, destinationStage, 0, 0,
                       nullptr, 0, nullptr, 1, &barrier);

  endSingleTimeCommands(commandBuffer);
}

VkImageView vrglasses_for_robots::VulkanRenderer::createImageView(
    VkImage image, VkFormat format, VkImageAspectFlags aspectFlags) {
  VkImageViewCreateInfo viewInfo = {};
  viewInfo.sType = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
  viewInfo.image = image;
  viewInfo.viewType = VK_IMAGE_VIEW_TYPE_2D;
  viewInfo.format = format;
  viewInfo.subresourceRange.aspectMask = aspectFlags;
  viewInfo.subresourceRange.baseMipLevel = 0;
  viewInfo.subresourceRange.levelCount = 1;
  viewInfo.subresourceRange.baseArrayLayer = 0;
  viewInfo.subresourceRange.layerCount = 1;

  VkImageView imageView;
  if (vkCreateImageView(device, &viewInfo, nullptr, &imageView) != VK_SUCCESS) {
    throw std::runtime_error("failed to create texture image view!");
  }

  return imageView;
}

VkCommandBuffer
vrglasses_for_robots::VulkanRenderer::beginSingleTimeCommands() {
  VkCommandBufferAllocateInfo allocInfo = {};
  allocInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO;
  allocInfo.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
  allocInfo.commandPool = commandPool;
  allocInfo.commandBufferCount = 1;

  VkCommandBuffer commandBuffer;
  vkAllocateCommandBuffers(device, &allocInfo, &commandBuffer);

  VkCommandBufferBeginInfo beginInfo = {};
  beginInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
  beginInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

  vkBeginCommandBuffer(commandBuffer, &beginInfo);

  return commandBuffer;
}

void vrglasses_for_robots::VulkanRenderer::endSingleTimeCommands(
    VkCommandBuffer commandBuffer) {
  vkEndCommandBuffer(commandBuffer);

  VkSubmitInfo submitInfo = {};
  submitInfo.sType = VK_STRUCTURE_TYPE_SUBMIT_INFO;
  submitInfo.commandBufferCount = 1;
  submitInfo.pCommandBuffers = &commandBuffer;

  vkQueueSubmit(queue, 1, &submitInfo, VK_NULL_HANDLE);
  vkQueueWaitIdle(queue);

  vkFreeCommandBuffers(device, commandPool, 1, &commandBuffer);
}

void vrglasses_for_robots::VulkanRenderer::copyBuffer(VkBuffer srcBuffer,
                                                      VkBuffer dstBuffer,
                                                      VkDeviceSize size) {
  VkCommandBuffer commandBuffer = beginSingleTimeCommands();

  VkBufferCopy copyRegion = {};
  copyRegion.size = size;
  vkCmdCopyBuffer(commandBuffer, srcBuffer, dstBuffer, 1, &copyRegion);

  endSingleTimeCommands(commandBuffer);
}

void vrglasses_for_robots::VulkanRenderer::createVertexBuffer() {
  VkDeviceSize bufferSize = sizeof(vertices_[0]) * vertices_.size();
  VkBuffer stagingBuffer;
  VkDeviceMemory stagingBufferMemory;
  createBuffer(VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
               VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                   VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
               &stagingBuffer, &stagingBufferMemory, bufferSize);

  void *data;
  vkMapMemory(device, stagingBufferMemory, 0, bufferSize, 0, &data);
  memcpy(data, vertices_.data(), static_cast<size_t>(bufferSize));
  vkUnmapMemory(device, stagingBufferMemory);

  createBuffer(
      VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_VERTEX_BUFFER_BIT,
      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, &vertexBuffer, &vertexMemory,
      bufferSize);

  copyBuffer(stagingBuffer, vertexBuffer, bufferSize);

  vkDestroyBuffer(device, stagingBuffer, nullptr);
  vkFreeMemory(device, stagingBufferMemory, nullptr);
}

void vrglasses_for_robots::VulkanRenderer::createIndexBuffer() {
  VkDeviceSize bufferSize = sizeof(indices_[0]) * indices_.size();

  VkBuffer stagingBuffer;
  VkDeviceMemory stagingBufferMemory;
  createBuffer(VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
               VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                   VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
               &stagingBuffer, &stagingBufferMemory, bufferSize);

  void *data;
  vkMapMemory(device, stagingBufferMemory, 0, bufferSize, 0, &data);
  memcpy(data, indices_.data(), static_cast<size_t>(bufferSize));
  vkUnmapMemory(device, stagingBufferMemory);

  createBuffer(
      VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_INDEX_BUFFER_BIT,
      VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, &indexBuffer, &indexMemory,
      bufferSize);

  copyBuffer(stagingBuffer, indexBuffer, bufferSize);

  vkDestroyBuffer(device, stagingBuffer, nullptr);
  vkFreeMemory(device, stagingBufferMemory, nullptr);
}

void vrglasses_for_robots::VulkanRenderer::releaseMeshDataBuffers() {
  vkDestroyBuffer(device, vertexBuffer, nullptr);
  vkFreeMemory(device, vertexMemory, nullptr);
  vkDestroyBuffer(device, indexBuffer, nullptr);
  vkFreeMemory(device, indexMemory, nullptr);
}

vrglasses_for_robots::VulkanRenderer::VulkanRenderer(
    uint32_t width, uint32_t height, float near, float far,
    const std::string &shader_spv_folder)
    : width_(width), height_(height), far_(far), near_(near) {
  std::cout << "Running headless rendering example\n";

  boost::filesystem::path shader_folder =
      boost::filesystem::path(shader_spv_folder);
  shader_vert_spv_ =
      (shader_folder / "vrglasses4robots_shader.vert.spv").string();
  shader_frag_spv_ =
      (shader_folder / "vrglasses4robots_shader.frag.spv").string();

  // shader_vert_spv_(shader_vert_spv),
  //    shader_frag_spv_(shader_frag_spv)
  // shader_spv_folder

  initVulkan(true);

  buildRenderPass(width_, height_);

  vkQueueWaitIdle(queue);
}

void vrglasses_for_robots::VulkanRenderer::buildOpenglProjectionFromIntrinsics(
    glm::mat4 &matPerspective, glm::mat4 &matProjection,
    glm::mat4 &matCVProjection, int img_width, int img_height, float alpha,
    float beta, float skew, float u0, float v0, float near, float far) {
  // These parameters define the final viewport that is rendered into by the
  // camera.
  float l = 0;
  float r = img_width;
  float b = 0;
  float t = img_height;

  // near and far clipping planes, these only matter for the mapping from
  // world-space z-coordinate into the depth coordinate for OpenGL
  float n = near;
  float f = far;

  //  // set the viewport parameters
  //  viewport[0] = l;
  //  viewport[1] = b;
  //  viewport[2] = r - l;
  //  viewport[3] = t - b;

  // construct an orthographic matrix which maps from projected coordinates to
  // normalized device coordinates in the range
  // [-1, 1].  OpenGL then maps coordinates in NDC to the current viewport
  glm::mat4 ndc(0);
  ndc[0][0] = 2.0 / (r - l);
  ndc[3][0] = -(r + l) / (r - l);
  ndc[1][1] = 2.0 / (t - b);
  ndc[3][1] = -(t + b) / (t - b);
  ndc[2][2] = -2.0 / (f - n);
  ndc[3][2] = -(f + n) / (f - n);
  ndc[3][3] = 1.0;

  // construct a projection matrix, this is identical to the projection matrix
  // computed for the intrinsic,
  // except an additional row is inserted to map the z-coordinate to OpenGL.
  // CMatrix4<T> matProjection(0);    // the 3rd column is inverted to make the
  // camera look towards +Z (instead of -Z in opengl)
  matProjection = glm::mat4(0);
  matProjection[0][0] = alpha;
  matProjection[1][0] = skew;
  matProjection[2][0] = -u0;
  matProjection[1][1] = beta;
  matProjection[2][1] = -v0;
  matProjection[2][2] = (n + f);
  matProjection[3][2] = n * f;
  matProjection[2][3] = -1.0;

  matCVProjection = glm::mat4(0);
  matCVProjection[0][0] = alpha;
  matCVProjection[1][0] = skew;
  matCVProjection[2][0] = u0;
  matCVProjection[1][1] = beta;
  matCVProjection[2][1] = v0;
  matCVProjection[2][2] = 1;
  matCVProjection[3][2] = 0;
  matCVProjection[2][3] = 1;
  matCVProjection[3][3] = 1;

  // resulting OpenGL frustum is the product of the orthographic
  // mapping to normalized device coordinates and the augmented camera intrinsic
  // matrix
  matPerspective = ndc * matProjection;
}

void vrglasses_for_robots::VulkanRenderer::setCamera(float p_focal_u,
                                                     float p_focal_v,
                                                     float p_center_u,
                                                     float p_center_v) {
  glm::mat4 mv = glm::mat4(1.0);  // convert between opencv to opengl camera
  mv[1][1] = 1.0;
  mv[2][2] = -1.0;

  glm::mat4 projection, perpective;

  buildOpenglProjectionFromIntrinsics(perpective, projection, projection_cv_,
                                      width_, height_, p_focal_u, p_focal_v, 0,
                                      p_center_u, p_center_v, near_, far_);
  vp_cv_ = perpective * mv;
  //
}

void vrglasses_for_robots::VulkanRenderer::setCamera(glm::mat4 mvp) {
  vp_cv_ = mvp;
}

void vrglasses_for_robots::VulkanRenderer::renderMesh(
    cv::Mat &result_depth_map, cv::Mat &result_attribute_map) {
  drawTriangles(width_, height_);

  saveImageDepthmap(width_, height_, result_depth_map, result_attribute_map);
  // sparseTest(landmarks_3d, result_depth_map);

  vkQueueWaitIdle(queue);
  // releaseMeshDataBuffers();
}

void vrglasses_for_robots::VulkanRenderer::cleanUp() {
  vkDestroyBuffer(device, vertexBuffer, nullptr);
  vkFreeMemory(device, vertexMemory, nullptr);
  vkDestroyBuffer(device, indexBuffer, nullptr);
  vkFreeMemory(device, indexMemory, nullptr);

  // Clean up resources for copy
  vkFreeMemory(device, dstImageMemory, nullptr);
  vkDestroyImage(device, dstImage, nullptr);
  vkDestroyBuffer(device, image_buffer, nullptr);
  vkFreeMemory(device, image_buffer_memory, nullptr);

  vkDestroyImageView(device, colorAttachment.view, nullptr);
  vkDestroyImage(device, colorAttachment.image, nullptr);
  vkFreeMemory(device, colorAttachment.memory, nullptr);
  vkDestroyImageView(device, depthAttachment.view, nullptr);
  vkDestroyImage(device, depthAttachment.image, nullptr);
  vkFreeMemory(device, depthAttachment.memory, nullptr);
  for (size_t i = 0; i < textures_.size(); i++) {
    vkDestroySampler(device, textures_[i].textureSampler, nullptr);
    vkDestroyImageView(device, textures_[i].textureImageView, nullptr);
    vkDestroyImage(device, textures_[i].textureImage, nullptr);
    vkFreeMemory(device, textures_[i].textureImageMemory, nullptr);
  }
  vkDestroyDescriptorPool(device, descriptorPool, nullptr);

  vkDestroyRenderPass(device, renderPass, nullptr);
  vkDestroyFramebuffer(device, framebuffer, nullptr);
  vkDestroyPipelineLayout(device, pipelineLayout, nullptr);
  vkDestroyDescriptorSetLayout(device, descriptorSetLayout, nullptr);
  vkDestroyPipeline(device, pipeline, nullptr);
  vkDestroyPipelineCache(device, pipelineCache, nullptr);
  vkDestroyCommandPool(device, commandPool, nullptr);
  for (auto shadermodule : shaderModules) {
    vkDestroyShaderModule(device, shadermodule, nullptr);
  }
  vkDestroyDevice(device, nullptr);
#if DEBUG
  if (debugReportCallback) {
    PFN_vkDestroyDebugReportCallbackEXT vkDestroyDebugReportCallback =
        reinterpret_cast<PFN_vkDestroyDebugReportCallbackEXT>(
            vkGetInstanceProcAddr(instance, "vkDestroyDebugReportCallbackEXT"));
    assert(vkDestroyDebugReportCallback);
    vkDestroyDebugReportCallback(instance, debugReportCallback, nullptr);
  }
#endif
  vkDestroyInstance(instance, nullptr);
}

void vrglasses_for_robots::VulkanRenderer::clearModels() {
  // Clear Models and associated containers
  models_.clear();
  indices_.clear();
  vertices_.clear();
  // Clear scene item
  scene_items_.clear();
  // Buffers / memory
  if (!textures_.empty()) {
    // Clean everything up
    cleanUp();
    // Restart
    initVulkan(true);
    buildRenderPass(width_, height_);
    vkQueueWaitIdle(queue);
  }
  // Clear texture
  textures_.clear();
}

vrglasses_for_robots::VulkanRenderer::~VulkanRenderer() {
  cleanUp();
}

bool vrglasses_for_robots::VulkanRenderer::loadMesh(
    const std::string &filename_model_obj,
    const std::string &filename_model_tex) {
  assert(models_.size() == 0);
  models_.push_back(ThreeDModel());
  models_[0].name = "model_one";
  models_[0].obj_file = filename_model_obj;
  models_[0].texture_file = filename_model_tex;
  models_index_["model_one"] = 0;

  // load geometry
  loadVertex(0);

  // load textures
  textures_.resize(1);

  setupDescriptorPool();

  models_[0].material_index = 0;
  createTextureImage(textures_[0], models_[0].texture_file);
  setupDescriptorSet(textures_[0]);

  copyVertex();

  return false;
}

bool vrglasses_for_robots::VulkanRenderer::loadMeshs(
    const std::string &model_folder, const std::string &model_list) {
  boost::filesystem::path folder = boost::filesystem::path(model_folder);
  std::vector<std::string> ids;
  std::vector<std::string> objs;
  std::vector<std::string> texs;

  if (boost::filesystem::exists(model_list)) {
    std::ifstream file(model_list.c_str());
    if (file.is_open()) {
      std::string line;

      while (std::getline(file, line)) {
        boost::trim(line);
        if (line.empty())
          continue;

        std::vector<std::string> strs;
        boost::split(strs, line, boost::is_any_of(";"));

        models_.push_back(ThreeDModel());
        models_.back().name = boost::trim_copy(strs[0]);
        models_.back().obj_file = (folder / boost::trim_copy(strs[1])).string();
        models_.back().texture_file =
            (folder / boost::trim_copy(strs[2])).string();
        models_index_[models_.back().name] = models_.size() - 1;

        if (!boost::filesystem::exists(models_.back().obj_file)) {
          throw std::invalid_argument("the file does not exit: " +
                                      models_.back().obj_file);
        }
        if (!boost::filesystem::exists(models_.back().texture_file)) {
          throw std::invalid_argument("the file does not exit: " +
                                      models_.back().texture_file);
        }
      }
      if (models_.size() == 0) {
        throw std::invalid_argument("no valid model in the file");
      }
    } else {
      throw std::invalid_argument("file could not be opened");
    }
  } else {
    throw std::invalid_argument("model list file could not be opened ");
  }
  // load geometry
  for (size_t idx = 0; idx < models_.size(); idx++) {
    loadVertex(idx);
    //models_[idx].material_index = idx;
  }

  // load textures
  textures_.resize(models_.size());

  setupDescriptorPool();
  for (size_t idx = 0; idx < models_.size(); idx++) {
    models_[idx].material_index = idx;
    createTextureImage(textures_[idx], models_[idx].texture_file);
    setupDescriptorSet(textures_[idx]);
  }

  copyVertex();

  return true;
}

bool vrglasses_for_robots::VulkanRenderer::loadVertex(const size_t model_idx) {
  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;
  std::string warn, err;

  if (!tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                        models_[model_idx].obj_file.c_str())) {
    throw std::runtime_error(warn + err);
  }

  std::unordered_map<Vertex, uint32_t> uniqueVertices = {};

  models_[model_idx].begin_vertex_index = indices_.size();

  for (const auto &shape : shapes) {
    for (const auto &index : shape.mesh.indices) {
      Vertex vertex = {};

      vertex.pos = {attrib.vertices[3 * index.vertex_index + 0],
                    attrib.vertices[3 * index.vertex_index + 1],
                    attrib.vertices[3 * index.vertex_index + 2]};

      vertex.texCoord = {attrib.texcoords[2 * index.texcoord_index + 0],
                         1.0f - attrib.texcoords[2 * index.texcoord_index + 1]};

      // vertex.color = {1.0f, 1.0f, 1.0f};

      if (uniqueVertices.count(vertex) == 0) {
        uniqueVertices[vertex] = static_cast<uint32_t>(vertices_.size());
        vertices_.push_back(vertex);
      }

      indices_.push_back(uniqueVertices[vertex]);
    }
  }

  models_[model_idx].begin_vertex_count =
      indices_.size() - models_[model_idx].begin_vertex_index;

  return true;
}

void vrglasses_for_robots::VulkanRenderer::copyVertex() {
  std::vector<Vertex> &vertices = vertices_;
  std::vector<uint32_t> &indices = indices_;

  const VkDeviceSize vertexBufferSize = vertices.size() * sizeof(Vertex);
  const VkDeviceSize indexBufferSize = indices.size() * sizeof(uint32_t);

  VkBuffer stagingBuffer;
  VkDeviceMemory stagingMemory;

  // Command buffer for copy commands (reused)
  VkCommandBufferAllocateInfo cmdBufAllocateInfo =
      vks::initializers::commandBufferAllocateInfo(
          commandPool, VK_COMMAND_BUFFER_LEVEL_PRIMARY, 1);
  VkCommandBuffer copyCmd;
  VK_CHECK_RESULT(
      vkAllocateCommandBuffers(device, &cmdBufAllocateInfo, &copyCmd));
  VkCommandBufferBeginInfo cmdBufInfo =
      vks::initializers::commandBufferBeginInfo();

  // Copy input data to VRAM using a staging buffer
  {
    // Vertices
    createBuffer(VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                     VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                 &stagingBuffer, &stagingMemory, vertexBufferSize,
                 vertices.data());

    createBuffer(
        VK_BUFFER_USAGE_VERTEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, &vertexBuffer, &vertexMemory,
        vertexBufferSize);

    VK_CHECK_RESULT(vkBeginCommandBuffer(copyCmd, &cmdBufInfo));
    VkBufferCopy copyRegion = {};
    copyRegion.size = vertexBufferSize;
    vkCmdCopyBuffer(copyCmd, stagingBuffer, vertexBuffer, 1, &copyRegion);
    VK_CHECK_RESULT(vkEndCommandBuffer(copyCmd));

    submitWork(copyCmd, queue);

    vkDestroyBuffer(device, stagingBuffer, nullptr);
    vkFreeMemory(device, stagingMemory, nullptr);

    // Indices
    createBuffer(VK_BUFFER_USAGE_TRANSFER_SRC_BIT,
                 VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT |
                     VK_MEMORY_PROPERTY_HOST_COHERENT_BIT,
                 &stagingBuffer, &stagingMemory, indexBufferSize,
                 indices.data());

    createBuffer(
        VK_BUFFER_USAGE_INDEX_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT,
        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT, &indexBuffer, &indexMemory,
        indexBufferSize);

    VK_CHECK_RESULT(vkBeginCommandBuffer(copyCmd, &cmdBufInfo));
    copyRegion.size = indexBufferSize;
    vkCmdCopyBuffer(copyCmd, stagingBuffer, indexBuffer, 1, &copyRegion);
    VK_CHECK_RESULT(vkEndCommandBuffer(copyCmd));

    submitWork(copyCmd, queue);

    vkDestroyBuffer(device, stagingBuffer, nullptr);
    vkFreeMemory(device, stagingMemory, nullptr);
  }
}

glm::mat4 parsePose(std::string pose_text) {
  std::vector<std::string> strs;
  boost::split(strs, pose_text, boost::is_any_of(" "));
  glm::mat4 result = glm::translate(
      glm::mat4(1.0),
      glm::vec3(std::stod(strs[0]), std::stod(strs[1]), std::stod(strs[2])));
  result *= glm::eulerAngleZ((float)glm::radians(std::stod(strs[3])));
  return result;
}

bool vrglasses_for_robots::VulkanRenderer::loadScene(
    const std::string &scene_file) {
  if (boost::filesystem::exists(scene_file)) {
    std::ifstream file(scene_file.c_str());
    if (file.is_open()) {
      std::string line;

      while (std::getline(file, line)) {
        boost::trim(line);
        if (line.empty())
          continue;

        std::vector<std::string> strs;
        boost::split(strs, line, boost::is_any_of(";"));
        scene_items_.push_back(SceneItem());
        scene_items_.back().model_name = strs[0];
        scene_items_.back().T_World2Model = parsePose(strs[1]);
      }
    } else {
      throw std::invalid_argument("file could not be opened");
    }
  } else {
    throw std::invalid_argument("model list file could not be opened ");
  }
  return true;
}
void vrglasses_for_robots::VulkanRenderer::noFileScene() {
  for (size_t idx = 0; idx < models_.size(); idx++) {
    scene_items_.push_back(SceneItem());
    scene_items_.back().model_name = models_[idx].name;
    scene_items_.back().T_World2Model = glm::mat4(1.0);
  }
}
