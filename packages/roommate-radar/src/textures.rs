//! Overlay textures. SetOverlayRaw makes the compositor re-create the overlay's texture on every call, and the overlay
//! flickers while it does (the dashboard panel on every hover change). So pixels go into our own Vulkan images, handed
//! over with SetOverlayTexture; each overlay has two and alternates, so we never write the one on screen.
use std::collections::HashMap;
use std::ffi::c_void;

use ash::vk::{self, Handle};

use crate::draw::Image;
use crate::openvr::{self, VROverlayHandle_t, vr};

const FORMAT: vk::Format = vk::Format::R8G8B8A8_UNORM;

#[derive(Default)]
struct Texture {
    image: vk::Image,
    mem: vk::DeviceMemory,
    w: u32,
    h: u32,
}

#[derive(Default)]
struct Slot {
    tex: [Texture; 2],
    next: usize,
}

struct Staging {
    buf: vk::Buffer,
    mem: vk::DeviceMemory,
    map: *mut c_void,
    size: usize,
}

struct Vulkan {
    _entry: ash::Entry,
    instance: ash::Instance,
    phys: vk::PhysicalDevice,
    dev: ash::Device,
    queue: vk::Queue,
    qfam: u32,
    cmd: vk::CommandBuffer,
    fence: vk::Fence,
    staging: Option<Staging>,
    slots: HashMap<VROverlayHandle_t, Slot>,
}

/// Vulkan on the device the compositor renders with, else SetOverlayRaw.
pub struct Textures(Option<Vulkan>);

impl Textures {
    pub fn new() -> Textures {
        let v = unsafe { Vulkan::new() };
        if let Err(e) = &v {
            eprintln!("roommate-radar: Vulkan textures: {e}; using SetOverlayRaw");
        }
        Textures(v.ok())
    }

    /// Shows the pixels on overlay `ov`.
    pub fn set(&mut self, ov: VROverlayHandle_t, img: &Image) {
        let shown = self.0.as_mut().is_some_and(|v| unsafe { v.set(ov, img) }.is_ok());
        if !shown {
            vr().set_raw(ov, &img.px, img.w as u32, img.h as u32);
        }
    }
}

impl Vulkan {
    unsafe fn new() -> Result<Vulkan, String> {
        let err = |e: vk::Result| e.to_string();
        let entry = unsafe { ash::Entry::load() }.map_err(|e| e.to_string())?;
        let iext = vr().vulkan_extensions(None);
        let iptr: Vec<_> = iext.iter().map(|e| e.as_ptr()).collect();
        let app = vk::ApplicationInfo::default().application_name(c"roommate-radar").api_version(vk::API_VERSION_1_1);
        let ici = vk::InstanceCreateInfo::default().application_info(&app).enabled_extension_names(&iptr);
        let instance = unsafe { entry.create_instance(&ici, None) }.map_err(err)?;
        let phys = vk::PhysicalDevice::from_raw(vr().output_device(instance.handle().as_raw()));
        if phys.is_null() {
            return Err("no output device".into());
        }
        let qf = unsafe { instance.get_physical_device_queue_family_properties(phys) };
        let qfam =
            qf.iter().position(|q| q.queue_flags.contains(vk::QueueFlags::GRAPHICS)).ok_or("no graphics queue")? as u32;
        let dext = vr().vulkan_extensions(Some(phys.as_raw()));
        let dptr: Vec<_> = dext.iter().map(|e| e.as_ptr()).collect();
        let qci = [vk::DeviceQueueCreateInfo::default().queue_family_index(qfam).queue_priorities(&[1.0])];
        let dci = vk::DeviceCreateInfo::default().queue_create_infos(&qci).enabled_extension_names(&dptr);
        let dev = unsafe { instance.create_device(phys, &dci, None) }.map_err(err)?;
        let queue = unsafe { dev.get_device_queue(qfam, 0) };
        let pci = vk::CommandPoolCreateInfo::default()
            .flags(vk::CommandPoolCreateFlags::RESET_COMMAND_BUFFER)
            .queue_family_index(qfam);
        let pool = unsafe { dev.create_command_pool(&pci, None) }.map_err(err)?;
        let cai = vk::CommandBufferAllocateInfo::default()
            .command_pool(pool)
            .level(vk::CommandBufferLevel::PRIMARY)
            .command_buffer_count(1);
        let cmd = unsafe { dev.allocate_command_buffers(&cai) }.map_err(err)?[0];
        let fence = unsafe { dev.create_fence(&vk::FenceCreateInfo::default(), None) }.map_err(err)?;
        Ok(Vulkan { _entry: entry, instance, phys, dev, queue, qfam, cmd, fence, staging: None, slots: HashMap::new() })
    }

    unsafe fn set(&mut self, ov: VROverlayHandle_t, img: &Image) -> Result<(), vk::Result> {
        let (w, h) = (img.w as u32, img.h as u32);
        let slot = self.slots.entry(ov).or_default();
        let i = slot.next;
        slot.next ^= 1;
        if slot.tex[i].w != w || slot.tex[i].h != h {
            let old = std::mem::take(&mut slot.tex[i]);
            unsafe {
                self.dev.destroy_image(old.image, None);
                self.dev.free_memory(old.mem, None);
            }
            let t = unsafe { self.create(w, h) }?;
            self.slots.get_mut(&ov).unwrap().tex[i] = t;
        }
        let image = self.slots[&ov].tex[i].image;
        if self.staging.as_ref().is_none_or(|s| s.size < img.px.len()) {
            unsafe { self.grow_staging(img.px.len()) }?;
        }
        let st = self.staging.as_ref().unwrap();
        unsafe { std::ptr::copy_nonoverlapping(img.px.as_ptr(), st.map.cast::<u8>(), img.px.len()) };

        let (dev, cmd) = (&self.dev, self.cmd);
        unsafe {
            dev.reset_command_buffer(cmd, vk::CommandBufferResetFlags::empty())?;
            let bi = vk::CommandBufferBeginInfo::default().flags(vk::CommandBufferUsageFlags::ONE_TIME_SUBMIT);
            dev.begin_command_buffer(cmd, &bi)?;
            self.barrier(
                image,
                vk::ImageLayout::UNDEFINED,
                vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                vk::AccessFlags::empty(),
                vk::AccessFlags::TRANSFER_WRITE,
            );
            let copy = vk::BufferImageCopy::default()
                .image_subresource(vk::ImageSubresourceLayers {
                    aspect_mask: vk::ImageAspectFlags::COLOR,
                    mip_level: 0,
                    base_array_layer: 0,
                    layer_count: 1,
                })
                .image_extent(vk::Extent3D { width: w, height: h, depth: 1 });
            dev.cmd_copy_buffer_to_image(cmd, st.buf, image, vk::ImageLayout::TRANSFER_DST_OPTIMAL, &[copy]);
            // The layout OpenVR expects Vulkan textures in.
            self.barrier(
                image,
                vk::ImageLayout::TRANSFER_DST_OPTIMAL,
                vk::ImageLayout::TRANSFER_SRC_OPTIMAL,
                vk::AccessFlags::TRANSFER_WRITE,
                vk::AccessFlags::TRANSFER_READ,
            );
            dev.end_command_buffer(cmd)?;
            let cmds = [cmd];
            dev.reset_fences(&[self.fence])?;
            dev.queue_submit(self.queue, &[vk::SubmitInfo::default().command_buffers(&cmds)], self.fence)?;
            dev.wait_for_fences(&[self.fence], true, u64::MAX)?;
        }

        let data = openvr::VRVulkanTextureData_t {
            m_nImage: image.as_raw(),
            m_pDevice: self.dev.handle().as_raw() as *mut _,
            m_pPhysicalDevice: self.phys.as_raw() as *mut _,
            m_pInstance: self.instance.handle().as_raw() as *mut _,
            m_pQueue: self.queue.as_raw() as *mut _,
            m_nQueueFamilyIndex: self.qfam,
            m_nWidth: w,
            m_nHeight: h,
            m_nFormat: FORMAT.as_raw() as u32,
            m_nSampleCount: 1,
        };
        let tex = openvr::Texture_t {
            handle: &data as *const _ as *mut c_void,
            eType: openvr::ETextureType_TextureType_Vulkan,
            eColorSpace: openvr::EColorSpace_ColorSpace_Auto, // 8-bit UNORM = gamma, like Raw
        };
        vr().set_texture(ov, &tex);
        Ok(())
    }

    fn memory_type(&self, bits: u32, want: vk::MemoryPropertyFlags) -> Result<u32, vk::Result> {
        let mp = unsafe { self.instance.get_physical_device_memory_properties(self.phys) };
        (0..mp.memory_type_count)
            .find(|&i| bits >> i & 1 != 0 && mp.memory_types[i as usize].property_flags.contains(want))
            .ok_or(vk::Result::ERROR_OUT_OF_DEVICE_MEMORY)
    }

    unsafe fn create(&self, w: u32, h: u32) -> Result<Texture, vk::Result> {
        let ci = vk::ImageCreateInfo::default()
            .image_type(vk::ImageType::TYPE_2D)
            .format(FORMAT)
            .extent(vk::Extent3D { width: w, height: h, depth: 1 })
            .mip_levels(1)
            .array_layers(1)
            .samples(vk::SampleCountFlags::TYPE_1)
            .tiling(vk::ImageTiling::OPTIMAL)
            .usage(
                vk::ImageUsageFlags::TRANSFER_DST | vk::ImageUsageFlags::TRANSFER_SRC | vk::ImageUsageFlags::SAMPLED,
            );
        let image = unsafe { self.dev.create_image(&ci, None) }?;
        let mr = unsafe { self.dev.get_image_memory_requirements(image) };
        let mem = unsafe {
            self.alloc(mr, vk::MemoryPropertyFlags::DEVICE_LOCAL, |m| self.dev.bind_image_memory(image, m, 0))
        };
        // set() tries again every frame: a failure mustn't leak
        mem.inspect_err(|_| unsafe { self.dev.destroy_image(image, None) }).map(|mem| Texture { image, mem, w, h })
    }

    /// Memory for `mr`, bound by `bind` (freed again if that fails).
    unsafe fn alloc(
        &self,
        mr: vk::MemoryRequirements,
        want: vk::MemoryPropertyFlags,
        bind: impl FnOnce(vk::DeviceMemory) -> Result<(), vk::Result>,
    ) -> Result<vk::DeviceMemory, vk::Result> {
        let ai = vk::MemoryAllocateInfo::default()
            .allocation_size(mr.size)
            .memory_type_index(self.memory_type(mr.memory_type_bits, want)?);
        let mem = unsafe { self.dev.allocate_memory(&ai, None) }?;
        bind(mem).inspect_err(|_| unsafe { self.dev.free_memory(mem, None) }).map(|()| mem)
    }

    unsafe fn grow_staging(&mut self, size: usize) -> Result<(), vk::Result> {
        if let Some(s) = self.staging.take() {
            unsafe {
                self.dev.destroy_buffer(s.buf, None);
                self.dev.free_memory(s.mem, None);
            }
        }
        let bci = vk::BufferCreateInfo::default().size(size as u64).usage(vk::BufferUsageFlags::TRANSFER_SRC);
        let buf = unsafe { self.dev.create_buffer(&bci, None) }?;
        let mr = unsafe { self.dev.get_buffer_memory_requirements(buf) };
        let host = vk::MemoryPropertyFlags::HOST_VISIBLE | vk::MemoryPropertyFlags::HOST_COHERENT;
        let mapped = unsafe { self.alloc(mr, host, |m| self.dev.bind_buffer_memory(buf, m, 0)) }.and_then(|mem| {
            let map = unsafe { self.dev.map_memory(mem, 0, vk::WHOLE_SIZE, vk::MemoryMapFlags::empty()) };
            map.inspect_err(|_| unsafe { self.dev.free_memory(mem, None) }).map(|map| (mem, map))
        });
        let (mem, map) = mapped.inspect_err(|_| unsafe { self.dev.destroy_buffer(buf, None) })?;
        self.staging = Some(Staging { buf, mem, map, size });
        Ok(())
    }

    unsafe fn barrier(
        &self,
        image: vk::Image,
        from: vk::ImageLayout,
        to: vk::ImageLayout,
        src: vk::AccessFlags,
        dst: vk::AccessFlags,
    ) {
        let b = vk::ImageMemoryBarrier::default()
            .old_layout(from)
            .new_layout(to)
            .src_access_mask(src)
            .dst_access_mask(dst)
            .src_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
            .dst_queue_family_index(vk::QUEUE_FAMILY_IGNORED)
            .image(image)
            .subresource_range(vk::ImageSubresourceRange {
                aspect_mask: vk::ImageAspectFlags::COLOR,
                base_mip_level: 0,
                level_count: 1,
                base_array_layer: 0,
                layer_count: 1,
            });
        unsafe {
            self.dev.cmd_pipeline_barrier(
                self.cmd,
                vk::PipelineStageFlags::TRANSFER,
                vk::PipelineStageFlags::TRANSFER,
                vk::DependencyFlags::empty(),
                &[],
                &[],
                &[b],
            )
        };
    }
}
