/*
 * Copyright 2019 Google LLC. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.google.cardboard;

import android.Manifest;
import android.annotation.SuppressLint;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.content.res.AssetManager;
import android.hardware.usb.UsbDevice;
import android.net.Uri;
import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.provider.Settings;
import android.text.TextUtils;
import android.util.Log;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowManager;
import android.widget.PopupMenu;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.core.app.ActivityCompat;

import com.serenegiant.common.BaseActivity;
import com.serenegiant.usb.USBMonitor;

import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

/**
 * A Google Cardboard VR NDK sample application.
 *
 * <p>This is the main Activity for the sample application. It initializes a GLSurfaceView to allow
 * rendering.
 */
public class VrActivity extends BaseActivity implements PopupMenu.OnMenuItemClickListener {
  static {
    System.loadLibrary("cardboard_jni");
  }

  private static final String TAG = VrActivity.class.getSimpleName();
  private static final boolean DEBUG = true;

  // Permission request codes
  private static final int PERMISSIONS_REQUEST_CODE = 2;

  // Opaque native pointer to the native CardboardApp instance.
  // This object is owned by the VrActivity instance and passed to the native methods.
  private long nativeApp;

  private GLSurfaceView glView;

  private USBMonitor mUSBMonitor;
  private USBMonitor.UsbControlBlock mCtrlBlock;
  private final Object mSync = new Object();

  private static int mConnected = 0x00;

  private static final int OV_CONNECTED = 0x01;
  private static final int ST_CONNECTED = 0x10;

  private USBMonitor.OnDeviceConnectListener mOnDeviceConnectListener = new USBMonitor.OnDeviceConnectListener() {
    @Override
    public void onAttach(UsbDevice device) {
      int vid = device.getVendorId();
      int pid = device.getProductId();
            if (DEBUG) Log.v(TAG, String.format("=== onAttach: VID=0x%04x, PID=0x%04x, mConnected=0x%04x\n",vid, pid, mConnected));
            if (((vid == 0x05A9) && (pid == 0x0F87) && ((mConnected & OV_CONNECTED)!=OV_CONNECTED))
                || ((vid == 0x0483) && (pid == 0x7705) && ((mConnected&ST_CONNECTED)!= ST_CONNECTED))) {
                mUSBMonitor.requestPermission(device);
            }
    }

    @Override
    public void onDettach(UsbDevice device) {
      mConnected = 0x00;
      //Toast.makeText(MainActivity.this, "USB_DEVICE_DETACHED", Toast.LENGTH_SHORT).show();
      if (DEBUG) {
        Log.v(TAG,
                String.format("=== onDettach: VID=0x%04x, PID=0x%04x\n", device.getVendorId(),
                        device.getProductId()));
      }
    }

    String getUSBFSName(final USBMonitor.UsbControlBlock ctrlBlock) {
      String DEFAULT_USBFS = "/dev/bus/usb";
      String result = null;
      final String name = ctrlBlock.getDeviceName();
      final String[] v = !TextUtils.isEmpty(name) ? name.split("/") : null;
      if ((v != null) && (v.length > 2)) {
        final StringBuilder sb = new StringBuilder(v[0]);
        for (int i = 1; i < v.length - 2; i++) {
          sb.append("/").append(v[i]);
        }
        result = sb.toString();
      }
      if (TextUtils.isEmpty(result)) {
        Log.w(TAG, "failed to get USBFS path, try to use default path:" + name);
        result = DEFAULT_USBFS;
      }
      return result;
    }

    @Override
    public void onConnect(UsbDevice device, USBMonitor.UsbControlBlock ctrlBlock,
            boolean createNew) {
      queueEvent(new Runnable() {
        @Override
        public void run() {
          synchronized (mSync) {
            try {
              USBMonitor.UsbControlBlock mCtrlBlock = ctrlBlock.clone();
              int devId = mCtrlBlock.getDeviceId();
              int vid = mCtrlBlock.getVenderId();
              int pid = mCtrlBlock.getProductId();
              if (DEBUG) {
                Log.d(TAG,
                        String.format("onConnect: Device ID = %d\nVID=0x%04x\nPID=0x%04x\n", devId,
                                vid, pid));
              }
              if ((vid == 0x05A9) && (pid == 0x0F87)) {
                if (DEBUG) Log.d(TAG, "IMU MATCH FOUND!");
                String usbfs_path = mCtrlBlock.getDeviceName();
                if (DEBUG) Log.d(TAG, "imu_usbfs_path = " + usbfs_path);
                int file_descriptor = mCtrlBlock.getFileDescriptor();
                if (DEBUG) Log.d(TAG, "imu_fd = " + file_descriptor);
                String usb_fs = getUSBFSName(mCtrlBlock);
                if (DEBUG) Log.d(TAG, "imu_usb_fs = " + usb_fs);
                nativeSetUsbFileDescriptor(nativeApp, mCtrlBlock.getVenderId(), mCtrlBlock.getProductId(),
                        mCtrlBlock.getFileDescriptor(),
                        mCtrlBlock.getBusNum(),
                        mCtrlBlock.getDevNum(),
                        getUSBFSName(mCtrlBlock));
                showToast(R.string.ov_connected);
                mConnected = (mConnected | OV_CONNECTED);
              }
              if ((vid == 0x0483) && (pid == 0x7705)) {
                if (DEBUG) Log.d(TAG, "ST MATCH FOUND!");
                String usbfs_path = mCtrlBlock.getDeviceName();
                if (DEBUG) Log.d(TAG, "st_usbfs_path = " + usbfs_path);
                int file_descriptor = mCtrlBlock.getFileDescriptor();
                if (DEBUG) Log.d(TAG, "st_fd = " + file_descriptor);
                String usb_fs = getUSBFSName(mCtrlBlock);
                if (DEBUG) Log.d(TAG, "st_usb_fs = " + usb_fs);
                nativeSetStUfd(nativeApp, mCtrlBlock.getVenderId(), ctrlBlock.getProductId(),
                        mCtrlBlock.getFileDescriptor(),
                        mCtrlBlock.getBusNum(),
                        mCtrlBlock.getDevNum(),
                        getUSBFSName(mCtrlBlock));
                showToast(R.string.st_connected);
                mConnected = (mConnected | ST_CONNECTED);
              }
            } catch (IllegalStateException ex) {
              if (DEBUG) Log.d(TAG, "ex:", ex);
            } catch (CloneNotSupportedException e) {
              if (DEBUG) Log.d(TAG, "ex:", e);
            }
          }
        }
      }, 0);
    }


    @Override
    public void onDisconnect(UsbDevice device, USBMonitor.UsbControlBlock ctrlBlock) {
      try {
        USBMonitor.UsbControlBlock mCtrlBlock = ctrlBlock.clone();
        int devId = mCtrlBlock.getDeviceId();
        int vid = mCtrlBlock.getVenderId();
        int pid = mCtrlBlock.getProductId();
        if (DEBUG) {
          Log.d(TAG, String.format(
                  "onDisconnect: Device ID = %d\\nVID=0x%04x\\nPID=0x%04x\\n\", "
                          + "devId, vid, pid"));
        }
      } catch (IllegalStateException ex) {
        if (DEBUG) Log.d(TAG, "ex:", ex);
      } catch (CloneNotSupportedException e) {
        if (DEBUG) Log.d(TAG, "ex:", e);
      }
      queueEvent(new Runnable() {
        @Override
        public void run() {
          synchronized (mSync) {// TODO:
          }
        }
      }, 0);
    }

    @Override
    public void onCancel(UsbDevice device) {
      if (DEBUG) Log.d(TAG, "onCancel");
    }
  };

  @SuppressLint("ClickableViewAccessibility")
  @Override
  public void onCreate(Bundle savedInstance) {
    super.onCreate(savedInstance);

    nativeApp = nativeOnCreate(getAssets());

    mUSBMonitor = new USBMonitor(this, mOnDeviceConnectListener);
    setContentView(R.layout.activity_vr);
    glView = findViewById(R.id.surface_view);
    glView.setEGLContextClientVersion(2);
    Renderer renderer = new Renderer();
    glView.setRenderer(renderer);
    glView.setRenderMode(GLSurfaceView.RENDERMODE_CONTINUOUSLY);
    glView.setOnTouchListener(
        (v, event) -> {
          if (event.getAction() == MotionEvent.ACTION_DOWN) {
            // Signal a trigger event.
            glView.queueEvent(
                () -> {
                  nativeOnTriggerEvent(nativeApp);
                });
            return true;
          }
          return false;
        });

    // TODO(b/139010241): Avoid that action and status bar are displayed when pressing settings
    // button.
    setImmersiveSticky();
    View decorView = getWindow().getDecorView();
    decorView.setOnSystemUiVisibilityChangeListener(
        (visibility) -> {
          if ((visibility & View.SYSTEM_UI_FLAG_FULLSCREEN) == 0) {
            setImmersiveSticky();
          }
        });

    // Forces screen to max brightness.
    WindowManager.LayoutParams layout = getWindow().getAttributes();
    layout.screenBrightness = 1.f;
    getWindow().setAttributes(layout);

    // Prevents screen from dimming/locking.
    getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
  }

  @Override
  protected void onPause() {
    super.onPause();
    synchronized (mSync) {
      if (mUSBMonitor != null) {
        mUSBMonitor.unregister();
      }
    }
    if (DEBUG) {
      Log.d(TAG, "onPause");
    }
    if (mConnected != 0x00) {
      nativeOnPause(nativeApp);
      glView.onPause();
    }
  }

  @Override
  protected void onResume() {
    super.onResume();

    synchronized (mSync) {
      if (mUSBMonitor != null) {
        mUSBMonitor.register();
      }
    }
    // Checks for activity permissions, if not granted, requests them.
    if (!arePermissionsEnabled()) {
      requestPermissions();
      return;
    }
    if (DEBUG) {
      Log.d(TAG, "onResume");
    }
    if (mConnected == 0x11) {
      glView.onResume();
      nativeOnResume(nativeApp);
    }
  }

  @Override
  protected void onDestroy() {
    super.onDestroy();
    nativeOnDestroy(nativeApp);
    nativeApp = 0;
  }

  @Override
  public void onWindowFocusChanged(boolean hasFocus) {
    super.onWindowFocusChanged(hasFocus);
    if (hasFocus) {
      setImmersiveSticky();
    }
  }

  private class Renderer implements GLSurfaceView.Renderer {
    @Override
    public void onSurfaceCreated(GL10 gl10, EGLConfig eglConfig) {
      nativeOnSurfaceCreated(nativeApp);
    }

    @Override
    public void onSurfaceChanged(GL10 gl10, int width, int height) {
      nativeSetScreenParams(nativeApp, width, height);
    }

    @Override
    public void onDrawFrame(GL10 gl10) {
      nativeOnDrawFrame(nativeApp);
    }
  }

  /** Callback for when close button is pressed. */
  public void closeSample(View view) {
    Log.d(TAG, "Leaving VR sample");
    finish();
  }

  /** Callback for when settings_menu button is pressed. */
  public void showSettings(View view) {
    PopupMenu popup = new PopupMenu(this, view);
    MenuInflater inflater = popup.getMenuInflater();
    inflater.inflate(R.menu.settings_menu, popup.getMenu());
    popup.setOnMenuItemClickListener(this);
    popup.show();
  }

  @Override
  public boolean onMenuItemClick(MenuItem item) {
    if (item.getItemId() == R.id.switch_viewer) {
      nativeSwitchViewer(nativeApp);
      return true;
    }
    return false;
  }

  /**
   * Checks for activity permissions.
   *
   * @return whether the permissions are already granted.
   */
  private boolean arePermissionsEnabled() {
    return ActivityCompat.checkSelfPermission(this, Manifest.permission.READ_EXTERNAL_STORAGE)
        == PackageManager.PERMISSION_GRANTED;
  }

  /** Handles the requests for activity permissions. */
  private void requestPermissions() {
    final String[] permissions = new String[] {Manifest.permission.READ_EXTERNAL_STORAGE};
    ActivityCompat.requestPermissions(this, permissions, PERMISSIONS_REQUEST_CODE);
  }

  /** Callback for the result from requesting permissions. */
  @Override
  public void onRequestPermissionsResult(
          int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
    super.onRequestPermissionsResult(requestCode, permissions, grantResults);
    if (!arePermissionsEnabled()) {
      Toast.makeText(this, R.string.no_permissions, Toast.LENGTH_LONG).show();
      if (!ActivityCompat.shouldShowRequestPermissionRationale(
          this, Manifest.permission.READ_EXTERNAL_STORAGE)) {
        // Permission denied with checking "Do not ask again".
        launchPermissionsSettings();
      }
      finish();
    }
  }

  private void launchPermissionsSettings() {
    Intent intent = new Intent();
    intent.setAction(Settings.ACTION_APPLICATION_DETAILS_SETTINGS);
    intent.setData(Uri.fromParts("package", getPackageName(), null));
    startActivity(intent);
  }

  private void setImmersiveSticky() {
    getWindow()
        .getDecorView()
        .setSystemUiVisibility(
            View.SYSTEM_UI_FLAG_LAYOUT_STABLE
                | View.SYSTEM_UI_FLAG_LAYOUT_HIDE_NAVIGATION
                | View.SYSTEM_UI_FLAG_LAYOUT_FULLSCREEN
                | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION
                | View.SYSTEM_UI_FLAG_FULLSCREEN
                | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);
  }

  private native long nativeOnCreate(AssetManager assetManager);

  private native void nativeOnDestroy(long nativeApp);

  private native void nativeOnSurfaceCreated(long nativeApp);

  private native void nativeOnDrawFrame(long nativeApp);

  private native void nativeOnTriggerEvent(long nativeApp);

  private native void nativeOnPause(long nativeApp);

  private native void nativeOnResume(long nativeApp);

  private native void nativeSetScreenParams(long nativeApp, int width, int height);

  private native void nativeSwitchViewer(long nativeApp);

  private native void nativeSetUsbFileDescriptor(long nativeApp, int vid, int pid, int fd, int busnum, int devaddr, String usbfs_str);

  private native void nativeCloseUsb(long nativeApp);

  private native void nativeExitUsb(long nativeApp);

  private  native void nativeSetStUfd(long nativeApp, int vid, int pid, int fd, int busnum, int devaddr, String usbfs_str);
}
