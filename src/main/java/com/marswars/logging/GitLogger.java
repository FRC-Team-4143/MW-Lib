package com.marswars.logging;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

public abstract class GitLogger {

  private static StringPublisher project_name_pub_ =
      NetworkTableInstance.getDefault().getStringTopic("/Metadata/PROJECT_NAME").publish();
  private static StringPublisher git_sha_pub_ =
      NetworkTableInstance.getDefault().getStringTopic("/Metadata/GIT_SHA").publish();
  private static StringPublisher git_date_pub_ =
      NetworkTableInstance.getDefault().getStringTopic("/Metadata/GIT_DATE").publish();
  private static StringPublisher git_branch_pub_ =
      NetworkTableInstance.getDefault().getStringTopic("/Metadata/GIT_BRANCH").publish();
  private static StringPublisher build_date_pub_ =
      NetworkTableInstance.getDefault().getStringTopic("/Metadata/BUILD_DATE").publish();
  private static StringPublisher dirty_pub_ =
      NetworkTableInstance.getDefault().getStringTopic("/Metadata/DIRTY").publish();

  private static Alert dirty_alert_ = new Alert("Deployed code has uncommitted changes - This can make it hard to reproduce results", AlertType.kWarning);

  /**
   * Logs Git and build metadata to NetworkTables using reflection.
   * Extracts and publishes project name, Git SHA, date, branch, build date, and dirty status.
   * 
   * @param build_constants a Class object containing build constants with Git metadata fields
   */
  public static void logGitData(Object build_constants) {
    try {
      // If buildConstants is a Class object, use it directly; otherwise get its class
      Class<?> clazz = (build_constants instanceof Class<?>) 
          ? (Class<?>) build_constants 
          : build_constants.getClass();
      
      project_name_pub_.set((String) clazz.getField("MAVEN_NAME").get(null));
      String fullSha = (String) clazz.getField("GIT_SHA").get(null);
      git_sha_pub_.set(fullSha.length() > 7 ? fullSha.substring(0, 7) : fullSha);
      git_date_pub_.set((String) clazz.getField("GIT_DATE").get(null));
      git_branch_pub_.set((String) clazz.getField("GIT_BRANCH").get(null));
      build_date_pub_.set((String) clazz.getField("BUILD_DATE").get(null));
      int dirtyFlag = (int) clazz.getField("DIRTY").get(null);
      switch (dirtyFlag) {
        case 0:
          dirty_pub_.set("All changes committed");
          dirty_alert_.set(false);
          break;
        case 1:
          dirty_pub_.set("Uncommitted changes");
          dirty_alert_.set(true);
          break;
        default:
          dirty_pub_.set("Unknown");
          break;
      }
    } catch (Exception e) {
      System.err.println("Failed to log git data: " + e.getMessage());
      e.printStackTrace();
    }
  }
}
