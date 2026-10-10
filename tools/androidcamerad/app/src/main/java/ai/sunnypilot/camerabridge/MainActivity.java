package ai.sunnypilot.camerabridge;

import android.Manifest;
import android.app.Activity;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.os.Bundle;
import android.view.Gravity;
import android.view.WindowManager;
import android.widget.LinearLayout;
import android.widget.TextView;

public final class MainActivity extends Activity {
  private static final int CAMERA_PERMISSION_REQUEST = 1;

  @Override
  protected void onCreate(Bundle state) {
    super.onCreate(state);
    getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

    LinearLayout layout = new LinearLayout(this);
    layout.setOrientation(LinearLayout.VERTICAL);
    layout.setGravity(Gravity.CENTER);
    layout.setPadding(48, 48, 48, 48);
    layout.setBackgroundColor(Color.rgb(238, 244, 241));

    TextView title = new TextView(this);
    title.setText("Sunnypilot Camera Bridge");
    title.setTextSize(26);
    title.setTextColor(Color.rgb(16, 35, 31));
    title.setGravity(Gravity.CENTER);
    layout.addView(title);

    TextView detail = new TextView(this);
    detail.setText("Camera2 frames are written directly into Linux VisionIPC buffers.\n\nThis window may be closed after capture starts.");
    detail.setTextSize(16);
    detail.setTextColor(Color.rgb(55, 78, 72));
    detail.setGravity(Gravity.CENTER);
    detail.setPadding(0, 32, 0, 0);
    layout.addView(detail);
    setContentView(layout);

    if (checkSelfPermission(Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED) {
      startCameraService();
    } else {
      requestPermissions(new String[]{Manifest.permission.CAMERA}, CAMERA_PERMISSION_REQUEST);
    }
  }

  @Override
  public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] results) {
    super.onRequestPermissionsResult(requestCode, permissions, results);
    if (requestCode == CAMERA_PERMISSION_REQUEST && results.length > 0 && results[0] == PackageManager.PERMISSION_GRANTED) {
      startCameraService();
    }
  }

  private void startCameraService() {
    Intent service = new Intent(this, CameraService.class);
    Intent launch = getIntent();
    service.putExtra("camera_id", launch.getStringExtra("camera_id"));
    service.putExtra("width", launch.getIntExtra("width", 1928));
    service.putExtra("height", launch.getIntExtra("height", 1208));
    service.putExtra("buffer_count", launch.getIntExtra("buffer_count", 18));
    startForegroundService(service);
  }
}
