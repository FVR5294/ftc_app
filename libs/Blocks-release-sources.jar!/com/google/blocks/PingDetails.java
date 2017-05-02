// Copyright 2016 Google Inc.

package com.google.blocks;

import com.google.gson.Gson;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

/**
 * Class representing the details of a ping request.
 *
 * @author lizlooney@google.com (Liz Looney)
 */
public final class PingDetails {
  private static final Map<String, String> remoteIpToMachineName = new HashMap<String, String>();

  public final String name;
  public final String machineName;

  public PingDetails(String name, String remoteIp, String userAgent) {
    this.name = name;

    String machineType = getMachineType(userAgent);
    String machineName = remoteIpToMachineName.get(remoteIp);
    if (machineName == null || !machineName.startsWith(machineType)) {
      for (int i = 1; i < Integer.MAX_VALUE; i++) {
        machineName = machineType + " #" + i;
        if (!remoteIpToMachineName.containsValue(machineName)) {
          break;
        }
      }
      remoteIpToMachineName.put(remoteIp, machineName);
    }
    this.machineName = machineName;
  }

  private static String getMachineType(String userAgent) {
    if (userAgent.contains("Windows Phone")) {
      return "WindowsPhone";
    } else if (userAgent.contains("Windows")) {
      return "Windows";
    } else if (userAgent.contains("Macintosh")) {
      return "Mac";
    } else if (userAgent.contains("CrOS")) {
      return "ChromeBook";
    } else if (userAgent.contains("android")) {
      return "Android";
    } else if (userAgent.contains("iPhone")) {
      return "iPhone";
    } else if (userAgent.contains("iPad")) {
      return "iPad";
    } else if (userAgent.contains("X11")) {
      return "Unix";
    }
    return "";
  }

  public String toJson() {
    return new Gson().toJson(this);
  }

  public static PingDetails fromJson(String json) {
    return new Gson().fromJson(json, PingDetails.class);
  }

  // java.lang.Object methods

  @Override
  public boolean equals(Object o) {
    if (o instanceof PingDetails) {
      PingDetails that = (PingDetails) o;
      return Objects.equals(this.name, that.name)
          && Objects.equals(this.machineName, that.machineName);
    }
    return false;
  }

  @Override
  public int hashCode() {
    return Objects.hash(name, machineName);
  }
}
