package ai.sunnypilot.camerabridge;

import android.app.Notification;
import android.app.NotificationChannel;
import android.app.NotificationManager;
import android.app.PendingIntent;
import android.app.Service;
import android.content.Intent;
import android.content.pm.ServiceInfo;
import android.os.IBinder;
import android.os.PowerManager;
import android.util.Log;

import java.io.File;

public final class CameraService extends Service {
  private static final String TAG = "SunnypilotCamera";
  private static final String CHANNEL_ID = "camera_capture";
  private static final int NOTIFICATION_ID = 41;

  static {
    System.loadLibrary("android_camerad");
  }

  private long nativeHandle;
  private PowerManager.WakeLock wakeLock;

  private static native long nativeStart(String socketPath, String cameraId, int width, int height,
                                         int bufferCount);
  private static native void nativeStop(long handle);

  @Override
  public void onCreate() {
    super.onCreate();
    NotificationManager notifications = getSystemService(NotificationManager.class);
    notifications.createNotificationChannel(new NotificationChannel(
        CHANNEL_ID, "Sunnypilot camera capture", NotificationManager.IMPORTANCE_LOW));

    Intent openApp = new Intent(this, MainActivity.class);
    PendingIntent pending = PendingIntent.getActivity(this, 0, openApp,
        PendingIntent.FLAG_IMMUTABLE | PendingIntent.FLAG_UPDATE_CURRENT);
    Notification notification = new Notification.Builder(this, CHANNEL_ID)
        .setContentTitle("Sunnypilot camera active")
        .setContentText("Waiting for the Linux VisionIPC bridge")
        .setSmallIcon(android.R.drawable.ic_menu_camera)
        .setContentIntent(pending)
        .setOngoing(true)
        .build();
    startForeground(NOTIFICATION_ID, notification, ServiceInfo.FOREGROUND_SERVICE_TYPE_CAMERA);

    PowerManager powerManager = getSystemService(PowerManager.class);
    wakeLock = powerManager.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK,
        "SunnypilotCamera::Capture");
    wakeLock.setReferenceCounted(false);
    wakeLock.acquire();
  }

  @Override
  public int onStartCommand(Intent intent, int flags, int startId) {
    if (nativeHandle == 0) {
      File ipcDirectory = new File(getFilesDir(), "ipc");
      if (!ipcDirectory.exists() && !ipcDirectory.mkdirs()) {
        Log.e(TAG, "Unable to create " + ipcDirectory);
        stopSelf();
        return START_NOT_STICKY;
      }

      String cameraId = intent != null ? intent.getStringExtra("camera_id") : null;
      int width = intent != null ? intent.getIntExtra("width", 1928) : 1928;
      int height = intent != null ? intent.getIntExtra("height", 1208) : 1208;
      int bufferCount = intent != null ? intent.getIntExtra("buffer_count", 18) : 18;
      nativeHandle = nativeStart(new File(ipcDirectory, "camerad.sock").getAbsolutePath(),
          cameraId == null ? "" : cameraId, width, height, bufferCount);
      if (nativeHandle == 0) {
        Log.e(TAG, "Camera startup failed");
        stopSelf();
        return START_NOT_STICKY;
      }
    }
    return START_STICKY;
  }

  @Override
  public void onDestroy() {
    if (nativeHandle != 0) {
      nativeStop(nativeHandle);
      nativeHandle = 0;
    }
    if (wakeLock != null && wakeLock.isHeld()) {
      wakeLock.release();
    }
    super.onDestroy();
  }

  @Override
  public IBinder onBind(Intent intent) {
    return null;
  }
}
