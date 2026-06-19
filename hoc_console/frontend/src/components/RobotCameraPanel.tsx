import { memo, useEffect, useState } from 'react';
import { useDashboardStore } from '../stores/dashboardStore';
import { resolveCameraMjpegUrl } from '../utils/cameraUrl';

export const RobotCameraPanel = memo(function RobotCameraPanel() {
  const connected = useDashboardStore((s) => s.connected);
  const [failed, setFailed] = useState(false);
  const mjpegUrl = resolveCameraMjpegUrl();

  useEffect(() => {
    if (!connected) {
      setFailed(false);
    }
  }, [connected]);

  return (
    <div className="panel panel--camera">
      <h3>机械臂实时画面</h3>
      <div className="camera-viewport">
        {connected && !failed ? (
          <img
            className="camera-frame"
            src={mjpegUrl}
            alt="PyBullet 仿真画面"
            draggable={false}
            onError={() => setFailed(true)}
            onLoad={() => setFailed(false)}
          />
        ) : (
          <div className="camera-placeholder">
            {!connected && 'WebSocket 未连接'}
            {connected && failed &&
              '相机无画面：请确认 pybullet_bridge 已启动且已安装 pillow（python3 -m pip install pillow）'}
            {connected && !failed && '加载相机…'}
          </div>
        )}
      </div>
      <p className="panel-caption">
        MJPEG /hoc/camera/mjpeg · 源话题 /bridge/camera/image_compressed
      </p>
    </div>
  );
});
