use bevy::anti_alias::taa::TemporalAntiAliasing;
use bevy::camera::Hdr;
use bevy::core_pipeline::prepass::{DepthPrepass, MotionVectorPrepass, NormalPrepass};
use bevy::core_pipeline::tonemapping::Tonemapping;
use bevy::light::atmosphere::ScatteringMedium;
use bevy::light::{
    Atmosphere, AtmosphereEnvironmentMapLight, DirectionalLight, EnvironmentMapLight,
    GeneratedEnvironmentMapLight, GlobalAmbientLight, PointLight, ShadowFilteringMethod, SpotLight,
};
use bevy::pbr::{
    AtmosphereSettings, ScreenSpaceAmbientOcclusion, ScreenSpaceAmbientOcclusionQualityLevel,
};
use bevy::post_process::bloom::Bloom;
use bevy::prelude::*;
use bevy::render::camera::{MipBias, TemporalJitter};
use bevy::render::view::Msaa;

use crate::camera_control::MainViewportCamera;
use crate::protocol::{
    QUALITY_AGX, QUALITY_BLOOM, QUALITY_DEFAULT, QUALITY_IBL, QUALITY_MSAA, QUALITY_SHADOWS,
    QUALITY_SSAO,
};

#[derive(Resource)]
pub struct QualityState {
    pub flags: u32,
    pub dirty: bool,
    pub world_has_sky: bool,
    pub sky_intensity: f32,
    pub env_map_path: Option<String>,
    pub env_map_cubemap: Option<Handle<Image>>,
    pub env_map_intensity: f32,
    pub env_map_dirty: bool,
    pub env_map_active: bool,
    pub ambient_color: Color,
    pub ambient_brightness: f32,
    pub atmosphere_entity: Option<Entity>,
    pub scattering_medium: Option<Handle<ScatteringMedium>>,
}

impl Default for QualityState {
    fn default() -> Self {
        Self {
            flags: QUALITY_DEFAULT,
            dirty: true,
            world_has_sky: false,
            sky_intensity: 1.0,
            env_map_path: None,
            env_map_cubemap: None,
            env_map_intensity: 350.0,
            env_map_dirty: false,
            env_map_active: false,
            ambient_color: Color::srgb(0.4, 0.4, 0.4),
            ambient_brightness: 260.0,
            atmosphere_entity: None,
            scattering_medium: None,
        }
    }
}

pub fn apply_quality_settings_system(
    mut commands: Commands,
    mut quality: ResMut<QualityState>,
    mut scattering_media: ResMut<Assets<ScatteringMedium>>,
    mut ambient: ResMut<GlobalAmbientLight>,
    camera_q: Query<Entity, With<MainViewportCamera>>,
    mut dir_lights: Query<(&mut DirectionalLight, &crate::scene_sync::SdfLightMeta)>,
    mut point_lights: Query<(&mut PointLight, &crate::scene_sync::SdfLightMeta)>,
    mut spot_lights: Query<(&mut SpotLight, &crate::scene_sync::SdfLightMeta)>,
) {
    if !quality.dirty {
        return;
    }
    let Ok(cam_entity) = camera_q.single() else {
        return;
    };
    quality.dirty = false;

    let flags = quality.flags;
    let use_ibl = (flags & QUALITY_IBL) != 0;
    let use_ssao = (flags & QUALITY_SSAO) != 0;
    let use_bloom = (flags & QUALITY_BLOOM) != 0;
    let use_agx = (flags & QUALITY_AGX) != 0;
    let use_shadows = (flags & QUALITY_SHADOWS) != 0;
    let use_msaa = (flags & QUALITY_MSAA) != 0 && !use_ssao;

    // IBL: prefer explicit HDR environment probe (indoor/studio) over procedural sky Atmosphere
    let has_env_probe = quality.env_map_cubemap.is_some();
    if use_ibl && has_env_probe {
        if let Some(atmo_ent) = quality.atmosphere_entity.take() {
            commands.entity(atmo_ent).despawn();
        }
        commands
            .entity(cam_entity)
            .remove::<AtmosphereSettings>()
            .remove::<AtmosphereEnvironmentMapLight>();

        if quality.env_map_dirty || !quality.env_map_active {
            if let Some(ref cubemap) = quality.env_map_cubemap {
                // Remove any prior generated EnvironmentMapLight so Bevy re-filters the new cubemap
                commands
                    .entity(cam_entity)
                    .remove::<EnvironmentMapLight>()
                    .insert(GeneratedEnvironmentMapLight {
                        environment_map: cubemap.clone(),
                        intensity: quality.env_map_intensity.max(1.0),
                        rotation: Quat::IDENTITY,
                        affects_lightmapped_mesh_diffuse: true,
                    });
                quality.env_map_dirty = false;
                quality.env_map_active = true;
            }
        }
    } else if use_ibl && quality.world_has_sky {
        if quality.env_map_active {
            commands
                .entity(cam_entity)
                .remove::<GeneratedEnvironmentMapLight>()
                .remove::<EnvironmentMapLight>();
            quality.env_map_active = false;
        }
        let medium = quality
            .scattering_medium
            .get_or_insert_with(|| scattering_media.add(ScatteringMedium::earth(256, 256)))
            .clone();
        if quality.atmosphere_entity.is_none() {
            let atmo_ent = commands.spawn(Atmosphere::earth(medium)).id();
            quality.atmosphere_entity = Some(atmo_ent);
        }
        commands.entity(cam_entity).insert((
            AtmosphereSettings::default(),
            AtmosphereEnvironmentMapLight {
                intensity: quality.sky_intensity.max(0.25),
                ..default()
            },
        ));
    } else {
        if let Some(atmo_ent) = quality.atmosphere_entity.take() {
            commands.entity(atmo_ent).despawn();
        }
        commands
            .entity(cam_entity)
            .remove::<AtmosphereSettings>()
            .remove::<AtmosphereEnvironmentMapLight>()
            .remove::<GeneratedEnvironmentMapLight>()
            .remove::<EnvironmentMapLight>();
        quality.env_map_active = false;
    }

    // Shadow filtering:
    // - SSAO + TAA mode: Temporal filtering + PCSS soft shadows (denoised cleanly by TAA)
    // - MSAA mode: Deterministic Castano '13 Gaussian PCF (zero blue-noise grain/speckle)
    commands.entity(cam_entity).insert(if use_ssao {
        ShadowFilteringMethod::Temporal
    } else {
        ShadowFilteringMethod::Gaussian
    });

    // HDR + Tonemapping
    commands.entity(cam_entity).insert((
        Hdr,
        if use_agx {
            Tonemapping::AgX
        } else {
            Tonemapping::Reinhard
        },
    ));

    // Bloom
    if use_bloom {
        commands.entity(cam_entity).insert(Bloom::NATURAL);
    } else {
        commands.entity(cam_entity).remove::<Bloom>();
    }

    // SSAO + TAA vs MSAA (Bevy SSAO requires Msaa::Off).
    // IMPORTANT: TemporalAntiAliasing and ScreenSpaceAmbientOcclusion pull in required
    // components (TemporalJitter, MipBias(-1.0), DepthPrepass, NormalPrepass,
    // MotionVectorPrepass). ECS .remove() does not strip required components, so we
    // must explicitly remove them when disabling SSAO+TAA — otherwise MipBias(-1.0)
    // and TemporalJitter remain stuck on the camera in MSAA mode, undersampling
    // mipmaps by 2x and causing severe brick-wall aliasing/shimmering.
    if use_ssao {
        commands.entity(cam_entity).insert((
            Msaa::Off,
            ScreenSpaceAmbientOcclusion {
                quality_level: ScreenSpaceAmbientOcclusionQualityLevel::High,
                constant_object_thickness: 0.25,
            },
            TemporalAntiAliasing::default(),
        ));
    } else {
        commands
            .entity(cam_entity)
            .remove::<ScreenSpaceAmbientOcclusion>()
            .remove::<TemporalAntiAliasing>()
            .remove::<TemporalJitter>()
            .remove::<MipBias>()
            .remove::<DepthPrepass>()
            .remove::<NormalPrepass>()
            .remove::<MotionVectorPrepass>()
            .insert(if use_msaa { Msaa::Sample4 } else { Msaa::Off });
    }

    // Ambient light (keep low when HDR probe or sky IBL is active for rich contrast)
    ambient.color = quality.ambient_color;
    ambient.brightness = if use_ibl && has_env_probe {
        quality.ambient_brightness * 0.15
    } else if use_ibl && quality.world_has_sky {
        quality.ambient_brightness * 0.28
    } else {
        quality.ambient_brightness
    };

    // Shadow toggle across all SDF lights (enable PCSS variable penumbra only when TAA is active
    // so MSAA stays 100% noise-free)
    for (mut l, meta) in &mut dir_lights {
        let on = use_shadows && meta.cast_shadows;
        l.shadow_maps_enabled = on;
        l.contact_shadows_enabled = false;
        l.soft_shadow_size = if on && use_ssao { Some(1.5) } else { None };
    }
    for (mut l, meta) in &mut point_lights {
        let on = use_shadows && meta.cast_shadows;
        l.shadow_maps_enabled = on;
        l.contact_shadows_enabled = false;
        l.soft_shadows_enabled = on && use_ssao;
    }
    for (mut l, meta) in &mut spot_lights {
        let on = use_shadows && meta.cast_shadows;
        l.shadow_maps_enabled = on;
        l.contact_shadows_enabled = false;
        l.soft_shadows_enabled = on && use_ssao;
    }

    info!(
        "Applied Bevy quality flags=0x{flags:02x} (IBL={use_ibl}, SSAO+TAA={use_ssao}, Bloom={use_bloom}, AgX={use_agx}, Shadows={use_shadows}, MSAA={use_msaa})"
    );
}
