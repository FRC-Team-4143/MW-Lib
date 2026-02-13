package com.marswars.logging;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

public abstract class GitLogger {

  private static StringPublisher project_name =
      NetworkTableInstance.getDefault().getStringTopic("/Metadata/PROJECT_NAME").publish();
  private static StringPublisher git_sha =
      NetworkTableInstance.getDefault().getStringTopic("/Metadata/GIT_SHA").publish();
  private static StringPublisher git_date =
      NetworkTableInstance.getDefault().getStringTopic("/Metadata/GIT_DATE").publish();
  private static StringPublisher git_branch =
      NetworkTableInstance.getDefault().getStringTopic("/Metadata/GIT_BRANCH").publish();
  private static StringPublisher build_date =
      NetworkTableInstance.getDefault().getStringTopic("/Metadata/BUILD_DATE").publish();
  private static StringPublisher dirty =
      NetworkTableInstance.getDefault().getStringTopic("/Metadata/DIRTY").publish();

  /**
   * Logs Git and build metadata to NetworkTables using reflection.
   * Extracts and publishes project name, Git SHA, date, branch, build date, and dirty status.
   * 
   * @param buildConstants a Class object containing build constants with Git metadata fields
   */
  public static void logGitData(Object buildConstants) {
    try {
      // If buildConstants is a Class object, use it directly; otherwise get its class
      Class<?> clazz = (buildConstants instanceof Class<?>) 
          ? (Class<?>) buildConstants 
          : buildConstants.getClass();
      
      project_name.set((String) clazz.getField("MAVEN_NAME").get(null));
      git_sha.set((String) clazz.getField("GIT_SHA").get(null));
      git_date.set((String) clazz.getField("GIT_DATE").get(null));
      git_branch.set((String) clazz.getField("GIT_BRANCH").get(null));
      build_date.set((String) clazz.getField("BUILD_DATE").get(null));
      int dirtyFlag = (int) clazz.getField("DIRTY").get(null);
      switch (dirtyFlag) {
        case 0:
          dirty.set("All changes committed");
          break;
        case 1:
          dirty.set("Uncommitted changes");
          break;
        default:
          dirty.set("Unknown");
          break;
      }
    } catch (Exception e) {
      System.err.println("Failed to log git data: " + e.getMessage());
      e.printStackTrace();
    }
  }
}
