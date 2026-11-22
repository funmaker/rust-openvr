//! The `TrackedCamera` interface provides access to cameras built into tracked devices.
//!
//! Applications can query a camera's frame size, intrinsics and projection, and acquire a video
//! streaming service to poll for camera frames. The camera is a shared, global resource, so it may
//! go inactive when there are no active consumers or when the headset is idle.

use std::mem;

use openvr_sys as sys;

use crate::errors::VRTrackedCameraError;
use crate::{
    TextureBounds, TrackedCamera, TrackedDeviceIndex, TrackedDevicePose, TrackingUniverseOrigin,
};

/// Handle to an acquired camera video stream.
pub type TrackedCameraHandle = sys::TrackedCameraHandle_t;

/// The type of camera frame to request.
#[derive(Debug, Copy, Clone, Eq, PartialEq)]
pub enum TrackedCameraFrameType {
    /// The camera video frame size in pixels, still distorted.
    Distorted = sys::EVRTrackedCameraFrameType_VRTrackedCameraFrameType_Distorted as isize,
    /// An undistorted inscribed rectangle region in pixels, without invalid regions.
    Undistorted = sys::EVRTrackedCameraFrameType_VRTrackedCameraFrameType_Undistorted as isize,
    /// The maximum undistorted size in pixels, including invalid regions.
    ///
    /// A non-zero alpha component identifies valid regions.
    MaximumUndistorted =
        sys::EVRTrackedCameraFrameType_VRTrackedCameraFrameType_MaximumUndistorted as isize,
}

impl From<sys::EVRTrackedCameraFrameType> for TrackedCameraFrameType {
    fn from(value: sys::EVRTrackedCameraFrameType) -> Self {
        match value {
            sys::EVRTrackedCameraFrameType_VRTrackedCameraFrameType_Distorted => Self::Distorted,
            sys::EVRTrackedCameraFrameType_VRTrackedCameraFrameType_Undistorted => {
                Self::Undistorted
            }
            sys::EVRTrackedCameraFrameType_VRTrackedCameraFrameType_MaximumUndistorted => {
                Self::MaximumUndistorted
            }
            _ => panic!("unrecognized tracked camera frame type"),
        }
    }
}

impl From<TrackedCameraFrameType> for sys::EVRTrackedCameraFrameType {
    fn from(value: TrackedCameraFrameType) -> Self {
        value as _
    }
}

/// Size of a camera frame.
#[derive(Default, Debug, Copy, Clone, Eq, PartialEq)]
pub struct CameraFrameSize {
    /// Width in pixels.
    pub width: u32,
    /// Height in pixels.
    pub height: u32,
    /// Size of the frame buffer in bytes.
    pub frame_buffer_size: u32,
}

/// Pinhole camera intrinsics.
#[derive(Default, Debug, Copy, Clone, PartialEq)]
pub struct CameraIntrinsics {
    /// Focal length in pixels for the x and y axes.
    pub focal_length: [f32; 2],
    /// Principal point in pixels.
    pub center: [f32; 2],
}

/// Size and texture bounds of a camera video stream texture.
#[derive(Debug, Clone)]
pub struct CameraTextureSize {
    /// Bounds of the camera image within the texture.
    pub bounds: TextureBounds,
    /// Width in pixels.
    pub width: u32,
    /// Height in pixels.
    pub height: u32,
}

/// Metadata for a camera video frame.
#[derive(Debug, Copy, Clone)]
pub struct CameraVideoStreamFrameHeader {
    /// The type of the frame.
    pub frame_type: TrackedCameraFrameType,
    /// Width in pixels.
    pub width: u32,
    /// Height in pixels.
    pub height: u32,
    /// Number of bytes per pixel.
    pub bytes_per_pixel: u32,
    /// Sequence number of the frame.
    pub frame_sequence: u32,
    /// Pose of the camera when the frame was captured.
    pub tracked_device_pose: TrackedDevicePose,
    /// Mid-point of the exposure of the image in host system ticks.
    pub frame_exposure_time: u64,
}

impl From<sys::CameraVideoStreamFrameHeader_t> for CameraVideoStreamFrameHeader {
    fn from(value: sys::CameraVideoStreamFrameHeader_t) -> Self {
        Self {
            frame_type: value.eFrameType.into(),
            width: value.nWidth,
            height: value.nHeight,
            bytes_per_pixel: value.nBytesPerPixel,
            frame_sequence: value.nFrameSequence,
            tracked_device_pose: value.trackedDevicePose.into(),
            frame_exposure_time: value.ulFrameExposureTime,
        }
    }
}

impl TrackedCamera {
    /// Returns whether the device has a camera.
    ///
    /// This is equivalent to reading the `Prop_HasCamera_Bool` tracked device property.
    pub fn has_camera(&self, device: TrackedDeviceIndex) -> bool {
        let mut has_camera = false;

        unsafe { self.0.HasCamera.unwrap()(device.0, &mut has_camera) };

        has_camera
    }

    /// Gets the size of a camera frame.
    pub fn get_camera_frame_size(
        &self,
        device: TrackedDeviceIndex,
        frame_type: TrackedCameraFrameType,
    ) -> Result<CameraFrameSize, VRTrackedCameraError> {
        let mut width = mem::MaybeUninit::uninit();
        let mut height = mem::MaybeUninit::uninit();
        let mut frame_buffer_size = mem::MaybeUninit::uninit();

        let error = unsafe {
            self.0.GetCameraFrameSize.unwrap()(
                device.0,
                frame_type.into(),
                width.as_mut_ptr(),
                height.as_mut_ptr(),
                frame_buffer_size.as_mut_ptr(),
            )
        };
        VRTrackedCameraError::new(error)?;

        Ok(CameraFrameSize {
            width: unsafe { width.assume_init() },
            height: unsafe { height.assume_init() },
            frame_buffer_size: unsafe { frame_buffer_size.assume_init() },
        })
    }

    /// Gets the intrinsics of a camera.
    pub fn get_camera_intrinsics(
        &self,
        device: TrackedDeviceIndex,
        camera_index: u32,
        frame_type: TrackedCameraFrameType,
    ) -> Result<CameraIntrinsics, VRTrackedCameraError> {
        let mut focal_length = mem::MaybeUninit::uninit();
        let mut center = mem::MaybeUninit::uninit();

        let error = unsafe {
            self.0.GetCameraIntrinsics.unwrap()(
                device.0,
                camera_index,
                frame_type.into(),
                focal_length.as_mut_ptr(),
                center.as_mut_ptr(),
            )
        };
        VRTrackedCameraError::new(error)?;

        Ok(CameraIntrinsics {
            focal_length: unsafe { focal_length.assume_init() }.v,
            center: unsafe { center.assume_init() }.v,
        })
    }

    /// Gets the projection matrix of a camera.
    pub fn get_camera_projection(
        &self,
        device: TrackedDeviceIndex,
        camera_index: u32,
        frame_type: TrackedCameraFrameType,
        near_z: f32,
        far_z: f32,
    ) -> Result<[[f32; 4]; 4], VRTrackedCameraError> {
        let mut projection = mem::MaybeUninit::uninit();

        let error = unsafe {
            self.0.GetCameraProjection.unwrap()(
                device.0,
                camera_index,
                frame_type.into(),
                near_z,
                far_z,
                projection.as_mut_ptr(),
            )
        };
        VRTrackedCameraError::new(error)?;

        Ok(unsafe { projection.assume_init() }.m)
    }

    /// Acquires the video streaming service for a device.
    ///
    /// Acquiring permits video streaming for the caller. If the camera has not been activated yet,
    /// the first frames may be delayed while it spins up and adjusts its exposure.
    pub fn acquire_video_streaming_service(
        &self,
        device: TrackedDeviceIndex,
    ) -> Result<TrackedCameraHandle, VRTrackedCameraError> {
        let mut handle = mem::MaybeUninit::uninit();

        let error = unsafe {
            self.0.AcquireVideoStreamingService.unwrap()(device.0, handle.as_mut_ptr())
        };
        VRTrackedCameraError::new(error)?;

        Ok(unsafe { handle.assume_init() })
    }

    /// Releases the video streaming service for a device.
    pub fn release_video_streaming_service(
        &self,
        handle: TrackedCameraHandle,
    ) -> Result<(), VRTrackedCameraError> {
        let error = unsafe { self.0.ReleaseVideoStreamingService.unwrap()(handle) };
        VRTrackedCameraError::new(error)
    }

    /// Polls the current camera frame header without copying the frame buffer.
    ///
    /// Useful for checking whether a new frame is available before doing the more expensive full
    /// frame-buffer copy.
    pub fn get_video_stream_frame_header(
        &self,
        handle: TrackedCameraHandle,
        frame_type: TrackedCameraFrameType,
    ) -> Result<CameraVideoStreamFrameHeader, VRTrackedCameraError> {
        let mut header = mem::MaybeUninit::uninit();

        let error = unsafe {
            self.0.GetVideoStreamFrameBuffer.unwrap()(
                handle,
                frame_type.into(),
                std::ptr::null_mut(),
                0,
                header.as_mut_ptr(),
                mem::size_of::<sys::CameraVideoStreamFrameHeader_t>() as u32,
            )
        };
        VRTrackedCameraError::new(error)?;

        Ok(unsafe { header.assume_init() }.into())
    }

    /// Copies the latest camera frame into `buffer`.
    ///
    /// The image data is provided as RGBA data, with four bytes per pixel. If no frame is available
    /// yet, for example while the camera is spinning up or after it has been reactivated, this
    /// returns `VRTrackedCameraError::NoFrameAvailable`.
    pub fn get_video_stream_frame_buffer(
        &self,
        handle: TrackedCameraHandle,
        frame_type: TrackedCameraFrameType,
        buffer: &mut [u8],
    ) -> Result<CameraVideoStreamFrameHeader, VRTrackedCameraError> {
        let mut header = mem::MaybeUninit::uninit();

        let error = unsafe {
            self.0.GetVideoStreamFrameBuffer.unwrap()(
                handle,
                frame_type.into(),
                buffer.as_mut_ptr() as *mut _,
                buffer.len() as u32,
                header.as_mut_ptr(),
                mem::size_of::<sys::CameraVideoStreamFrameHeader_t>() as u32,
            )
        };
        VRTrackedCameraError::new(error)?;

        Ok(unsafe { header.assume_init() }.into())
    }

    /// Gets the size and texture bounds of a camera video stream texture.
    pub fn get_video_stream_texture_size(
        &self,
        device: TrackedDeviceIndex,
        frame_type: TrackedCameraFrameType,
    ) -> Result<CameraTextureSize, VRTrackedCameraError> {
        let mut bounds = mem::MaybeUninit::uninit();
        let mut width = mem::MaybeUninit::uninit();
        let mut height = mem::MaybeUninit::uninit();

        let error = unsafe {
            self.0.GetVideoStreamTextureSize.unwrap()(
                device.0,
                frame_type.into(),
                bounds.as_mut_ptr(),
                width.as_mut_ptr(),
                height.as_mut_ptr(),
            )
        };
        VRTrackedCameraError::new(error)?;

        Ok(CameraTextureSize {
            bounds: TextureBounds(unsafe { bounds.assume_init() }),
            width: unsafe { width.assume_init() },
            height: unsafe { height.assume_init() },
        })
    }

    /// Sets the tracking space used for camera poses.
    pub fn set_camera_tracking_space(&self, origin: TrackingUniverseOrigin) {
        unsafe {
            self.0.SetCameraTrackingSpace.unwrap()(origin as sys::ETrackingUniverseOrigin)
        }
    }

    /// Returns the tracking space used for camera poses.
    pub fn get_camera_tracking_space(&self) -> TrackingUniverseOrigin {
        unsafe { self.0.GetCameraTrackingSpace.unwrap()() }.into()
    }
}
