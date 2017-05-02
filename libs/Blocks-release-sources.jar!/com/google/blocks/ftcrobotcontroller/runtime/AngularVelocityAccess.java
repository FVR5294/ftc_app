// Copyright 2016 Google Inc.

package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.Locale;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

/**
 * A class that provides JavaScript access to {@link AngularVelocity}.
 *
 * @author lizlooney@google.com (Liz Looney)
 */
class AngularVelocityAccess extends Access {

  AngularVelocityAccess(String identifier) {
    super(identifier);
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public String getAngleUnit(Object angularVelocity) {
    try {
      if (angularVelocity instanceof AngularVelocity) {
        AngleUnit angleUnit = ((AngularVelocity) angularVelocity).unit;
        if (angleUnit != null) {
          return angleUnit.toString();
        }
      } else {
        RobotLog.e("AngularVelocity.getAngleUnit - angularVelocity is not an AngularVelocity");
      }
    } catch (Exception e) {
      RobotLog.e("AngularVelocity.getAngleUnit - caught " + e);
    }
    return "";
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public float getXRotationRate(Object angularVelocity) {
    try {
      if (angularVelocity instanceof AngularVelocity) {
        return ((AngularVelocity) angularVelocity).xRotationRate;
      } else {
        RobotLog.e("AngularVelocity.getXRotationRate - angularVelocity is not an AngularVelocity");
      }
    } catch (Exception e) {
      RobotLog.e("AngularVelocity.getXRotationRate - caught " + e);
    }
    return 0;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public float getYRotationRate(Object angularVelocity) {
    try {
      if (angularVelocity instanceof AngularVelocity) {
        return ((AngularVelocity) angularVelocity).yRotationRate;
      } else {
        RobotLog.e("AngularVelocity.getYRotationRate - angularVelocity is not an AngularVelocity");
      }
    } catch (Exception e) {
      RobotLog.e("AngularVelocity.getYRotationRate - caught " + e);
    }
    return 0;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public float getZRotationRate(Object angularVelocity) {
    try {
      if (angularVelocity instanceof AngularVelocity) {
        return ((AngularVelocity) angularVelocity).zRotationRate;
      } else {
        RobotLog.e("AngularVelocity.getZRotationRate - angularVelocity is not an AngularVelocity");
      }
    } catch (Exception e) {
      RobotLog.e("AngularVelocity.getZRotationRate - caught " + e);
    }
    return 0;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public long getAcquisitionTime(Object angularVelocity) {
    try {
      if (angularVelocity instanceof AngularVelocity) {
        return ((AngularVelocity) angularVelocity).acquisitionTime;
      } else {
        RobotLog.e("AngularVelocity.getAcquisitionTime - angularVelocity is not an AngularVelocity");
      }
    } catch (Exception e) {
      RobotLog.e("AngularVelocity.getAcquisitionTime - caught " + e);
    }
    return 0;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public AngularVelocity create() {
    try {
      return new AngularVelocity();
    } catch (Exception e) {
      RobotLog.e("AngularVelocity.create - caught " + e);
    }
    return null;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public AngularVelocity create_withArgs(
      String angleUnitString, float xRotationRate, float yRotationRate,
      float zRotationRate, long acquisitionTime) {
    try {
      AngleUnit angleUnit =
          AngleUnit.valueOf(angleUnitString.toUpperCase(Locale.ENGLISH));
      return new AngularVelocity(
          angleUnit, xRotationRate, yRotationRate, zRotationRate, acquisitionTime);
    } catch (Exception e) {
      RobotLog.e("AngularVelocity.create - caught " + e);
    }
    return null;
  }

  @SuppressWarnings("unused")
  @JavascriptInterface
  public AngularVelocity toAngleUnit(Object angularVelocity, String angleUnitString) {
    try {
      if (angularVelocity instanceof AngularVelocity) {
        AngleUnit angleUnit =
            AngleUnit.valueOf(angleUnitString.toUpperCase(Locale.ENGLISH));
        return ((AngularVelocity) angularVelocity).toAngleUnit(angleUnit);
      } else {
        RobotLog.e("AngularVelocity.toAngleUnit - angularVelocity is not an AngularVelocity");
      }
    } catch (Exception e) {
      RobotLog.e("AngularVelocity.toAngleUnit - caught " + e);
    }
    return null;
  }
}
