// Copyright 2016 Google Inc.

package com.google.blocks.ftcrobotcontroller.runtime;

import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.webkit.JavascriptInterface;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

/**
 * A class that provides JavaScript access to the Android Gyroscope.
 *
 * @author lizlooney@google.com (Liz Looney)
 */
class AndroidGyroscopeAccess extends Access implements SensorEventListener {
  private final Activity activity;
  private volatile boolean listening;
  private volatile long timestamp;
  private volatile float x; // angular speed around the x-axis, in radians/second
  private volatile float y; // angular speed around the y-axis, in radians/second
  private volatile float z; // angular speed around the z-axis, in radians/second
  private volatile AngleUnit angleUnit = AngleUnit.RADIANS;

  AndroidGyroscopeAccess(String identifier, Activity activity) {
    super(identifier);
    this.activity = activity;
  }

  // Access methods

  @Override
  void close() {
    stopListening();
  }

  // SensorEventListener methods

  @Override
  public void onAccuracyChanged(Sensor sensor, int accuracy) {
  }

  @Override
  public void onSensorChanged(SensorEvent sensorEvent) {
    timestamp = sensorEvent.timestamp;
    x = sensorEvent.values[0];
    y = sensorEvent.values[1];
    z = sensorEvent.values[2];
  }

  // Javascript methods

  @SuppressWarnings("unused")
  @JavascriptInterface
  public void setAngleUnit(String angleUnitString) {
    try {
      angleUnit = AngleUnit.valueOf(angleUnitString.toUpperCase(Locale.ENGLISH));
    } catch (Exception e) {
      RobotLog.e("AndroidGyroscope.setAngelUnit - caught " + e);
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public float getX() {
    if (timestamp != 0) {
      return angleUnit.fromRadians(x);
    }
    return 0;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public float getY() {
    if (timestamp != 0) {
      return angleUnit.fromRadians(y);
    }
    return 0;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public float getZ() {
    if (timestamp != 0) {
      return angleUnit.fromRadians(z);
    }
    return 0;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public AngularVelocity getAngularVelocity() {
    if (timestamp != 0) {
      return new AngularVelocity(AngleUnit.RADIANS, x, y, z, timestamp).toAngleUnit(angleUnit);
    }
    return null;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public String getAngleUnit() {
    return angleUnit.toString();
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public boolean isAvailable() {
    SensorManager sensorManager = (SensorManager) activity.getSystemService(Context.SENSOR_SERVICE);
    return !sensorManager.getSensorList(Sensor.TYPE_GYROSCOPE).isEmpty();
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public void startListening() {
    if (!listening) {
      SensorManager sensorManager = (SensorManager) activity.getSystemService(Context.SENSOR_SERVICE);
      Sensor gyroscope = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
      sensorManager.registerListener(this, gyroscope, SensorManager.SENSOR_DELAY_FASTEST);
      listening = true;
    }
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public void stopListening() {
    if (listening) {
      SensorManager sensorManager = (SensorManager) activity.getSystemService(Context.SENSOR_SERVICE);
      sensorManager.unregisterListener(this);
      listening = false;
      timestamp = 0;
    }
  }
}
