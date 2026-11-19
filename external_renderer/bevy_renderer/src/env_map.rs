use std::f32::consts::{PI, TAU};

use bevy::asset::RenderAssetUsages;
use bevy::prelude::*;
use bevy::render::render_resource::{
    Extent3d, TextureDimension, TextureFormat, TextureUsages, TextureViewDescriptor,
    TextureViewDimension,
};

/// Convert IEEE-754 f32 to IEEE-754 binary16 (f16) bits.
#[inline]
fn f32_to_f16_bits(val: f32) -> u16 {
    let val = if val.is_finite() {
        val.clamp(-65504.0, 65504.0)
    } else {
        0.0
    };
    let bits = val.to_bits();
    let sign = ((bits >> 16) & 0x8000) as u16;
    let exp = ((bits >> 23) & 0xff) as i32 - 127 + 15;
    let frac = bits & 0x007f_ffff;

    if exp <= 0 {
        if exp < -10 {
            return sign;
        }
        let mant = (frac | 0x0080_0000) >> (1 - exp);
        return sign | ((mant + 0x1000) >> 13) as u16;
    } else if exp >= 31 {
        return sign | 0x7bff; // max finite f16
    }

    let rounded = frac + 0x1000;
    if (rounded & 0x0080_0000) != 0 {
        sign | (((exp + 1).min(30) as u16) << 10)
    } else {
        sign | ((exp as u16) << 10) | ((rounded >> 13) as u16)
    }
}

/// Convert a Bevy Y-up cubemap face index (0=+X, 1=-X, 2=+Y, 3=-Y, 4=+Z, 5=-Z)
/// and face UV (u, v in [-1, 1], u right, v down) into a normalized 3D direction.
#[inline]
fn cube_face_dir(face: usize, u: f32, v: f32) -> Vec3 {
    let d = match face {
        0 => Vec3::new(1.0, -v, -u),  // +X
        1 => Vec3::new(-1.0, -v, u),  // -X
        2 => Vec3::new(u, 1.0, v),    // +Y
        3 => Vec3::new(u, -1.0, -v),  // -Y
        4 => Vec3::new(u, -v, 1.0),   // +Z
        _ => Vec3::new(-u, -v, -1.0), // -Z
    };
    d.normalize()
}

/// Load a Radiance `.hdr` equirectangular environment map (using Gazebo's Z-up
/// equirectangular convention: `u = 0.5 - atan2(wy, wx) / 2pi`, `v = acos(wz) / pi`)
/// and resample it into a power-of-two `Rgba16Float` cubemap `Image` ready for
/// Bevy's GPU-filtered `GeneratedEnvironmentMapLight`.
pub fn load_equirect_hdr_as_cubemap(path: &str, face_size: u32) -> Option<Image> {
    let face_size = face_size.next_power_of_two().clamp(64, 512);
    let dyn_img = match image::open(path) {
        Ok(im) => im,
        Err(err) => {
            warn!("Failed to load environment map '{path}': {err}");
            return None;
        }
    };
    let rgb = dyn_img.into_rgb32f();
    let src_w = rgb.width() as usize;
    let src_h = rgb.height() as usize;
    if src_w < 2 || src_h < 2 {
        warn!("Environment map '{path}' has invalid dimensions {src_w}x{src_h}");
        return None;
    }
    let pixels = rgb.as_raw();

    let sample_bilinear = |u: f32, v: f32| -> [f32; 3] {
        let fx = u.rem_euclid(1.0) * (src_w as f32) - 0.5;
        let fy = (v.clamp(0.0, 1.0) * (src_h as f32) - 0.5).clamp(0.0, (src_h - 1) as f32);
        let x0 = (fx.floor() as isize).rem_euclid(src_w as isize) as usize;
        let x1 = (x0 + 1) % src_w;
        let y0 = (fy.floor() as usize).min(src_h - 1);
        let y1 = (y0 + 1).min(src_h - 1);
        let dx = fx - fx.floor();
        let dy = fy - fy.floor();

        let i00 = (y0 * src_w + x0) * 3;
        let i10 = (y0 * src_w + x1) * 3;
        let i01 = (y1 * src_w + x0) * 3;
        let i11 = (y1 * src_w + x1) * 3;

        let mut out = [0.0f32; 3];
        for c in 0..3 {
            let top = pixels[i00 + c] * (1.0 - dx) + pixels[i10 + c] * dx;
            let bot = pixels[i01 + c] * (1.0 - dx) + pixels[i11 + c] * dx;
            let v = top * (1.0 - dy) + bot * dy;
            out[c] = if v.is_finite() && v > 0.0 { v } else { 0.0 };
        }
        out
    };

    let fs = face_size as usize;
    let mut data = Vec::with_capacity(6 * fs * fs * 8);
    let one_f16 = f32_to_f16_bits(1.0).to_le_bytes();

    for face in 0..6 {
        for y in 0..fs {
            let vf = ((y as f32 + 0.5) / fs as f32) * 2.0 - 1.0;
            for x in 0..fs {
                let uf = ((x as f32 + 0.5) / fs as f32) * 2.0 - 1.0;
                // Bevy Y-up direction -> Gazebo Z-up direction via Rx(+90 deg):
                //   wx = bx, wy = -bz, wz = by
                let b = cube_face_dir(face, uf, vf);
                let wx = b.x;
                let wy = -b.z;
                let wz = b.y.clamp(-1.0, 1.0);

                let u_eq = (0.5 - wy.atan2(wx) / TAU).rem_euclid(1.0);
                let v_eq = (wz.acos() / PI).clamp(0.0, 1.0);
                let c = sample_bilinear(u_eq, v_eq);

                data.extend_from_slice(&f32_to_f16_bits(c[0]).to_le_bytes());
                data.extend_from_slice(&f32_to_f16_bits(c[1]).to_le_bytes());
                data.extend_from_slice(&f32_to_f16_bits(c[2]).to_le_bytes());
                data.extend_from_slice(&one_f16);
            }
        }
    }

    let mut img = Image::new(
        Extent3d {
            width: face_size,
            height: face_size,
            depth_or_array_layers: 6,
        },
        TextureDimension::D2,
        data,
        TextureFormat::Rgba16Float,
        RenderAssetUsages::default(),
    );
    img.texture_descriptor.usage = TextureUsages::TEXTURE_BINDING | TextureUsages::COPY_DST;
    img.texture_view_descriptor = Some(TextureViewDescriptor {
        dimension: Some(TextureViewDimension::Cube),
        ..default()
    });

    info!("Loaded HDR environment map '{path}' ({src_w}x{src_h}) -> {face_size}x{face_size}x6 Rgba16Float cubemap");
    Some(img)
}
