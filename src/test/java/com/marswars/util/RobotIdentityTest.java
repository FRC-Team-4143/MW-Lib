package com.marswars.util;

import static org.junit.jupiter.api.Assertions.*;

import org.junit.jupiter.api.Test;

class RobotIdentityTest {
  private enum Robot {
    ALPHA_BOT,
    BETA_BOT,
    SIM_BOT
  }

  @Test
  void matchesIgnoringCaseAndUnderscores() {
    assertEquals(Robot.BETA_BOT, RobotIdentity.match("BetaBot", Robot.class));
    assertEquals(Robot.SIM_BOT, RobotIdentity.match("SimBot", Robot.class));
    assertEquals(Robot.ALPHA_BOT, RobotIdentity.match("alpha_bot", Robot.class));
  }

  @Test
  void unknownNameDoesNotMatch() {
    assertNull(RobotIdentity.match("GammaBot", Robot.class));
  }
}
