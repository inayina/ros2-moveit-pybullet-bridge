function cameraBaseHost(): string {
  if (import.meta.env.DEV || window.location.port === '8080') {
    return '';
  }
  return `http://${window.location.hostname}:8766`;
}

/** MJPEG stream — stable img src, no cache-bust polling (avoids flicker). */
export function resolveCameraMjpegUrl(): string {
  if (import.meta.env.VITE_CAMERA_MJPEG_URL) {
    return import.meta.env.VITE_CAMERA_MJPEG_URL;
  }
  const prefix = cameraBaseHost();
  return prefix ? `${prefix}/hoc/camera/mjpeg` : '/hoc/camera/mjpeg';
}

/** Single JPEG snapshot (scripts / verify only). */
export function resolveCameraPreviewUrl(): string {
  if (import.meta.env.VITE_CAMERA_URL) {
    return import.meta.env.VITE_CAMERA_URL;
  }
  const prefix = cameraBaseHost();
  return prefix ? `${prefix}/hoc/camera/latest.jpg` : '/hoc/camera/latest.jpg';
}
