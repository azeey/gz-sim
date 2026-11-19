use std::collections::HashSet;
use std::sync::OnceLock;

use bevy::image::{ImageFilterMode, ImageSampler, ImageSamplerDescriptor};
use bevy::prelude::*;
use bevy::render::render_resource::{TextureDimension, TextureFormat, TextureUsages};

static SRGB_TO_LINEAR: OnceLock<[f32; 256]> = OnceLock::new();

fn srgb_to_linear_lut() -> &'static [f32; 256] {
    SRGB_TO_LINEAR.get_or_init(|| {
        let mut lut = [0.0f32; 256];
        for (i, slot) in lut.iter_mut().enumerate() {
            let c = (i as f32) / 255.0;
            *slot = if c <= 0.04045 {
                c / 12.92
            } else {
                ((c + 0.055) / 1.055).powf(2.4)
            };
        }
        lut
    })
}

#[inline]
fn linear_to_srgb_u8(v: f32) -> u8 {
    let v = v.clamp(0.0, 1.0);
    let s = if v <= 0.0031308 {
        v * 12.92
    } else {
        1.055 * v.powf(1.0 / 2.4) - 0.055
    };
    (s * 255.0 + 0.5).clamp(0.0, 255.0) as u8
}

fn downsample_rgba8(src: &[u8], src_w: usize, src_h: usize, is_srgb: bool) -> (Vec<u8>, usize, usize) {
    let dst_w = (src_w / 2).max(1);
    let dst_h = (src_h / 2).max(1);
    let mut dst = vec![0u8; dst_w * dst_h * 4];

    if is_srgb {
        let lut = srgb_to_linear_lut();
        for y in 0..dst_h {
            let y0 = (y * 2).min(src_h - 1);
            let y1 = (y * 2 + 1).min(src_h - 1);
            for x in 0..dst_w {
                let x0 = (x * 2).min(src_w - 1);
                let x1 = (x * 2 + 1).min(src_w - 1);
                let i00 = (y0 * src_w + x0) * 4;
                let i10 = (y0 * src_w + x1) * 4;
                let i01 = (y1 * src_w + x0) * 4;
                let i11 = (y1 * src_w + x1) * 4;
                let o = (y * dst_w + x) * 4;
                for c in 0..3 {
                    let avg = 0.25
                        * (lut[src[i00 + c] as usize]
                            + lut[src[i10 + c] as usize]
                            + lut[src[i01 + c] as usize]
                            + lut[src[i11 + c] as usize]);
                    dst[o + c] = linear_to_srgb_u8(avg);
                }
                let a = (src[i00 + 3] as u32
                    + src[i10 + 3] as u32
                    + src[i01 + 3] as u32
                    + src[i11 + 3] as u32
                    + 2)
                    / 4;
                dst[o + 3] = a as u8;
            }
        }
    } else {
        for y in 0..dst_h {
            let y0 = (y * 2).min(src_h - 1);
            let y1 = (y * 2 + 1).min(src_h - 1);
            for x in 0..dst_w {
                let x0 = (x * 2).min(src_w - 1);
                let x1 = (x * 2 + 1).min(src_w - 1);
                let i00 = (y0 * src_w + x0) * 4;
                let i10 = (y0 * src_w + x1) * 4;
                let i01 = (y1 * src_w + x0) * 4;
                let i11 = (y1 * src_w + x1) * 4;
                let o = (y * dst_w + x) * 4;
                for c in 0..4 {
                    let avg = (src[i00 + c] as u32
                        + src[i10 + c] as u32
                        + src[i01 + c] as u32
                        + src[i11 + c] as u32
                        + 2)
                        / 4;
                    dst[o + c] = avg as u8;
                }
            }
        }
    }
    (dst, dst_w, dst_h)
}

fn downsample_nchan_unorm(src: &[u8], src_w: usize, src_h: usize, ch: usize) -> (Vec<u8>, usize, usize) {
    let dst_w = (src_w / 2).max(1);
    let dst_h = (src_h / 2).max(1);
    let mut dst = vec![0u8; dst_w * dst_h * ch];
    for y in 0..dst_h {
        let y0 = (y * 2).min(src_h - 1);
        let y1 = (y * 2 + 1).min(src_h - 1);
        for x in 0..dst_w {
            let x0 = (x * 2).min(src_w - 1);
            let x1 = (x * 2 + 1).min(src_w - 1);
            let i00 = (y0 * src_w + x0) * ch;
            let i10 = (y0 * src_w + x1) * ch;
            let i01 = (y1 * src_w + x0) * ch;
            let i11 = (y1 * src_w + x1) * ch;
            let o = (y * dst_w + x) * ch;
            for c in 0..ch {
                let avg = (src[i00 + c] as u32
                    + src[i10 + c] as u32
                    + src[i01 + c] as u32
                    + src[i11 + c] as u32
                    + 2)
                    / 4;
                dst[o + c] = avg as u8;
            }
        }
    }
    (dst, dst_w, dst_h)
}

/// Automatically generates full mipmap chains and enables 16x anisotropic filtering
/// on all newly loaded 2D glTF and streamed textures.
///
/// MUST run in `PostUpdate.after(AssetEventSystems)` so it mutates `Assets<Image>`
/// in-place on the exact same frame `AssetEvent::Added` is emitted — BEFORE
/// `ExtractSchedule` uploads `GpuImage` and BEFORE `MaterialPlugin` builds GPU
/// bind groups (otherwise `PreparedMaterial` would permanently bind the initial
/// 1-mip texture view and anisotropy=1 sampler).
pub fn generate_texture_mipmaps_system(
    mut events: MessageReader<AssetEvent<Image>>,
    mut images: ResMut<Assets<Image>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
) {
    let mut generated_ids: HashSet<AssetId<Image>> = HashSet::new();

    for ev in events.read() {
        let id = match ev {
            AssetEvent::Added { id } | AssetEvent::Modified { id } => *id,
            _ => continue,
        };
        let Some(img) = images.get_mut_untracked(id) else {
            continue;
        };

        let desc = &img.texture_descriptor;
        if desc.dimension != TextureDimension::D2
            || desc.size.depth_or_array_layers != 1
            || desc.mip_level_count > 1
            || desc.sample_count != 1
            || desc.usage.contains(TextureUsages::RENDER_ATTACHMENT)
            || desc.usage.contains(TextureUsages::STORAGE_BINDING)
        {
            continue;
        }

        let w = desc.size.width as usize;
        let h = desc.size.height as usize;
        if w <= 1 && h <= 1 {
            continue;
        }

        enum Kind {
            Rgba8Srgb,
            Rgba8Unorm,
            Rg8Unorm,
            R8Unorm,
        }
        let kind = match desc.format {
            TextureFormat::Rgba8UnormSrgb => Kind::Rgba8Srgb,
            TextureFormat::Rgba8Unorm => Kind::Rgba8Unorm,
            TextureFormat::Rg8Unorm => Kind::Rg8Unorm,
            TextureFormat::R8Unorm => Kind::R8Unorm,
            _ => continue,
        };

        let bpp = match kind {
            Kind::Rgba8Srgb | Kind::Rgba8Unorm => 4,
            Kind::Rg8Unorm => 2,
            Kind::R8Unorm => 1,
        };

        let Some(ref mut data) = img.data else {
            continue;
        };
        if data.len() != w * h * bpp {
            continue;
        }

        let mut mip_count = 1u32;
        let mut cur_w = w;
        let mut cur_h = h;
        let mut prev_level = data.clone();
        // Reserve ~33% extra for full mip pyramid
        data.reserve(data.len() / 3 + 64);

        while cur_w > 1 || cur_h > 1 {
            let (next_level, nw, nh) = match kind {
                Kind::Rgba8Srgb => downsample_rgba8(&prev_level, cur_w, cur_h, true),
                Kind::Rgba8Unorm => downsample_rgba8(&prev_level, cur_w, cur_h, false),
                Kind::Rg8Unorm => downsample_nchan_unorm(&prev_level, cur_w, cur_h, 2),
                Kind::R8Unorm => downsample_nchan_unorm(&prev_level, cur_w, cur_h, 1),
            };
            data.extend_from_slice(&next_level);
            prev_level = next_level;
            cur_w = nw;
            cur_h = nh;
            mip_count += 1;
        }

        img.texture_descriptor.mip_level_count = mip_count;

        // Ensure trilinear + 16x anisotropic filtering on the sampler
        match &mut img.sampler {
            ImageSampler::Descriptor(d) => {
                d.mag_filter = ImageFilterMode::Linear;
                d.min_filter = ImageFilterMode::Linear;
                d.mipmap_filter = ImageFilterMode::Linear;
                d.anisotropy_clamp = 16;
            }
            ImageSampler::Default => {
                let mut d = ImageSamplerDescriptor::linear();
                d.set_anisotropic_filter(16);
                img.sampler = ImageSampler::Descriptor(d);
            }
        }
        generated_ids.insert(id);
    }

    if !generated_ids.is_empty() {
        // Also touch any StandardMaterial referencing these images so its GPU bind
        // group is guaranteed to bind the new mipmapped TextureView and 16x Sampler
        // even if the material was prepared before the image finished loading.
        let mut touched_mats = 0u32;
        let mat_ids: Vec<AssetId<StandardMaterial>> = materials
            .iter()
            .filter_map(|(mat_id, mat)| {
                let refs = [
                    mat.base_color_texture.as_ref(),
                    mat.normal_map_texture.as_ref(),
                    mat.metallic_roughness_texture.as_ref(),
                    mat.occlusion_texture.as_ref(),
                    mat.emissive_texture.as_ref(),
                ];
                if refs
                    .into_iter()
                    .flatten()
                    .any(|h| generated_ids.contains(&h.id()))
                {
                    Some(mat_id)
                } else {
                    None
                }
            })
            .collect();
        for mat_id in mat_ids {
            let _ = materials.get_mut(mat_id);
            touched_mats += 1;
        }
        info!(
            "Generated full mipmap chains + 16x anisotropy for {} textures (refreshed {} materials)",
            generated_ids.len(),
            touched_mats
        );
    }
}
