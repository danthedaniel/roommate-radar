// Overlay textures. SetOverlayRaw makes the compositor re-create the overlay's texture on every call, and the overlay
// flickers while it does (the dashboard panel on every hover change). So pixels go into our own Vulkan images,
// handed over with SetOverlayTexture; each overlay has two and alternates, so we never write the one on screen.
#pragma once
#include <vulkan/vulkan.h>
#include <openvr.h>

#include <cstdint>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

class Textures {
  public:
    // Vulkan on the device the compositor renders with; false: use SetOverlayRaw.
    bool init() {
        std::vector<std::string> iext = split(req(true, nullptr));
        std::vector<const char*> ip;
        for (auto& e : iext) {
            ip.push_back(e.c_str());
        }
        VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
        app.pApplicationName = "guardian";
        app.apiVersion = VK_API_VERSION_1_1;
        VkInstanceCreateInfo ici{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
        ici.pApplicationInfo = &app;
        ici.enabledExtensionCount = uint32_t(ip.size());
        ici.ppEnabledExtensionNames = ip.data();
        if (vkCreateInstance(&ici, nullptr, &inst_)) {
            return false;
        }
        uint64_t out = 0;
        vr::VRSystem()->GetOutputDevice(&out, vr::TextureType_Vulkan, inst_);
        phys_ = VkPhysicalDevice(out);
        if (!phys_) {
            return false;
        }

        uint32_t n = 0;
        vkGetPhysicalDeviceQueueFamilyProperties(phys_, &n, nullptr);
        std::vector<VkQueueFamilyProperties> qf(n);
        vkGetPhysicalDeviceQueueFamilyProperties(phys_, &n, qf.data());
        for (qfam_ = 0; qfam_ < n && !(qf[qfam_].queueFlags & VK_QUEUE_GRAPHICS_BIT); qfam_++) {
        }
        if (qfam_ == n) {
            return false;
        }
        std::vector<std::string> dext = split(req(false, phys_));
        std::vector<const char*> dp;
        for (auto& e : dext) {
            dp.push_back(e.c_str());
        }
        float prio = 1;
        VkDeviceQueueCreateInfo qci{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
        qci.queueFamilyIndex = qfam_;
        qci.queueCount = 1;
        qci.pQueuePriorities = &prio;
        VkDeviceCreateInfo dci{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
        dci.queueCreateInfoCount = 1;
        dci.pQueueCreateInfos = &qci;
        dci.enabledExtensionCount = uint32_t(dp.size());
        dci.ppEnabledExtensionNames = dp.data();
        if (vkCreateDevice(phys_, &dci, nullptr, &dev_)) {
            return false;
        }
        vkGetDeviceQueue(dev_, qfam_, 0, &queue_);
        VkCommandPoolCreateInfo pci{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
        pci.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT;
        pci.queueFamilyIndex = qfam_;
        VkCommandBufferAllocateInfo cai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
        cai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY;
        cai.commandBufferCount = 1;
        VkFenceCreateInfo fci{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
        return !vkCreateCommandPool(dev_, &pci, nullptr, &pool_) && (cai.commandPool = pool_, true) &&
               !vkAllocateCommandBuffers(dev_, &cai, &cmd_) && !vkCreateFence(dev_, &fci, nullptr, &fence_);
    }

    // Shows w x h RGBA pixels on overlay h (Vulkan if init() succeeded, else SetOverlayRaw).
    void set(vr::VROverlayHandle_t ov, const std::vector<uint8_t>& px, int w, int h) {
        if (!dev_) {
            vr::VROverlay()->SetOverlayRaw(ov, (void*)px.data(), w, h, 4);
            return;
        }
        Slot& s = slots_[ov];
        Image& im = s.img[s.next];
        s.next ^= 1;
        if (im.w != w || im.h != h) {
            destroy(im);
            if (!create(im, w, h)) {
                vr::VROverlay()->SetOverlayRaw(ov, (void*)px.data(), w, h, 4);
                return;
            }
        }
        if (px.size() > staging_size_ && !grow_staging(px.size())) {
            return;
        }
        memcpy(staging_map_, px.data(), px.size());

        vkResetCommandBuffer(cmd_, 0);
        VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
        bi.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;
        vkBeginCommandBuffer(cmd_, &bi);
        barrier(im.image, VK_IMAGE_LAYOUT_UNDEFINED, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 0,
                VK_ACCESS_TRANSFER_WRITE_BIT);
        VkBufferImageCopy copy{};
        copy.imageSubresource = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 0, 1};
        copy.imageExtent = {uint32_t(w), uint32_t(h), 1};
        vkCmdCopyBufferToImage(cmd_, staging_, im.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copy);
        // The layout OpenVR expects Vulkan textures in.
        barrier(im.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, VK_IMAGE_LAYOUT_TRANSFER_SRC_OPTIMAL,
                VK_ACCESS_TRANSFER_WRITE_BIT, VK_ACCESS_TRANSFER_READ_BIT);
        vkEndCommandBuffer(cmd_);
        VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
        si.commandBufferCount = 1;
        si.pCommandBuffers = &cmd_;
        vkResetFences(dev_, 1, &fence_);
        vkQueueSubmit(queue_, 1, &si, fence_);
        vkWaitForFences(dev_, 1, &fence_, VK_TRUE, UINT64_MAX);

        vr::VRVulkanTextureData_t vk{
            uint64_t(im.image),   dev_, phys_, inst_, queue_, qfam_, uint32_t(w), uint32_t(h), VK_FORMAT_R8G8B8A8_UNORM,
            VK_SAMPLE_COUNT_1_BIT};
        vr::Texture_t tex{&vk, vr::TextureType_Vulkan, vr::ColorSpace_Auto};  // 8-bit UNORM = gamma, like Raw
        vr::VROverlay()->SetOverlayTexture(ov, &tex);
    }

  private:
    struct Image {
        VkImage image = VK_NULL_HANDLE;
        VkDeviceMemory mem = VK_NULL_HANDLE;
        int w = 0, h = 0;
    };
    struct Slot {
        Image img[2];
        int next = 0;
    };
    std::unordered_map<vr::VROverlayHandle_t, Slot> slots_;
    VkInstance inst_ = VK_NULL_HANDLE;
    VkPhysicalDevice phys_ = VK_NULL_HANDLE;
    VkDevice dev_ = VK_NULL_HANDLE;
    VkQueue queue_ = VK_NULL_HANDLE;
    uint32_t qfam_ = 0;
    VkCommandPool pool_ = VK_NULL_HANDLE;
    VkCommandBuffer cmd_ = VK_NULL_HANDLE;
    VkFence fence_ = VK_NULL_HANDLE;
    VkBuffer staging_ = VK_NULL_HANDLE;
    VkDeviceMemory staging_mem_ = VK_NULL_HANDLE;
    void* staging_map_ = nullptr;
    size_t staging_size_ = 0;

    static std::string req(bool instance, VkPhysicalDevice phys) {
        char buf[4096] = "";
        if (instance) {
            vr::VRCompositor()->GetVulkanInstanceExtensionsRequired(buf, sizeof buf);
        } else {
            vr::VRCompositor()->GetVulkanDeviceExtensionsRequired(phys, buf, sizeof buf);
        }
        return buf;
    }
    static std::vector<std::string> split(const std::string& s) {
        std::vector<std::string> out;
        for (size_t i = 0, j; i < s.size(); i = j + 1) {
            j = s.find(' ', i);
            if (j == std::string::npos) {
                j = s.size();
            }
            if (j > i) {
                out.push_back(s.substr(i, j - i));
            }
        }
        return out;
    }
    uint32_t memory_type(uint32_t bits, VkMemoryPropertyFlags want) {
        VkPhysicalDeviceMemoryProperties mp;
        vkGetPhysicalDeviceMemoryProperties(phys_, &mp);
        for (uint32_t i = 0; i < mp.memoryTypeCount; i++) {
            if ((bits >> i & 1) && (mp.memoryTypes[i].propertyFlags & want) == want) {
                return i;
            }
        }
        return UINT32_MAX;
    }
    bool create(Image& im, int w, int h) {
        VkImageCreateInfo ci{VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO};
        ci.imageType = VK_IMAGE_TYPE_2D;
        ci.format = VK_FORMAT_R8G8B8A8_UNORM;
        ci.extent = {uint32_t(w), uint32_t(h), 1};
        ci.mipLevels = ci.arrayLayers = 1;
        ci.samples = VK_SAMPLE_COUNT_1_BIT;
        ci.tiling = VK_IMAGE_TILING_OPTIMAL;
        ci.usage = VK_IMAGE_USAGE_TRANSFER_DST_BIT | VK_IMAGE_USAGE_TRANSFER_SRC_BIT | VK_IMAGE_USAGE_SAMPLED_BIT;
        if (vkCreateImage(dev_, &ci, nullptr, &im.image)) {
            return false;
        }
        VkMemoryRequirements mr;
        vkGetImageMemoryRequirements(dev_, im.image, &mr);
        VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        ai.allocationSize = mr.size;
        ai.memoryTypeIndex = memory_type(mr.memoryTypeBits, VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT);
        if (vkAllocateMemory(dev_, &ai, nullptr, &im.mem) || vkBindImageMemory(dev_, im.image, im.mem, 0)) {
            return false;
        }
        im.w = w, im.h = h;
        return true;
    }
    void destroy(Image& im) {
        if (im.image) {
            vkDestroyImage(dev_, im.image, nullptr);
        }
        if (im.mem) {
            vkFreeMemory(dev_, im.mem, nullptr);
        }
        im = {};
    }
    bool grow_staging(size_t size) {
        if (staging_) {
            vkDestroyBuffer(dev_, staging_, nullptr), vkFreeMemory(dev_, staging_mem_, nullptr);
        }
        // Cleared so a failed allocation below can't leave the freed handles to be freed again next time.
        staging_ = VK_NULL_HANDLE, staging_mem_ = VK_NULL_HANDLE, staging_size_ = 0;
        VkBufferCreateInfo bci{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
        bci.size = size;
        bci.usage = VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
        if (vkCreateBuffer(dev_, &bci, nullptr, &staging_)) {
            return false;
        }
        VkMemoryRequirements mr;
        vkGetBufferMemoryRequirements(dev_, staging_, &mr);
        VkMemoryAllocateInfo ai{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
        ai.allocationSize = mr.size;
        ai.memoryTypeIndex =
            memory_type(mr.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
        if (vkAllocateMemory(dev_, &ai, nullptr, &staging_mem_) ||
            vkBindBufferMemory(dev_, staging_, staging_mem_, 0) ||
            vkMapMemory(dev_, staging_mem_, 0, VK_WHOLE_SIZE, 0, &staging_map_)) {
            return false;
        }
        staging_size_ = size;
        return true;
    }
    void barrier(VkImage image, VkImageLayout from, VkImageLayout to, VkAccessFlags src, VkAccessFlags dst) {
        VkImageMemoryBarrier b{VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER};
        b.oldLayout = from, b.newLayout = to, b.srcAccessMask = src, b.dstAccessMask = dst;
        b.srcQueueFamilyIndex = b.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
        b.image = image;
        b.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
        vkCmdPipelineBarrier(cmd_, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 0,
                             nullptr, 1, &b);
    }
};
