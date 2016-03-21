This module is based on the idea about offloading GPS parsing and calculations from a low powered flight controller processor (AtMega328)
  * and make all data available via the I2C bus.
  * Beside of GPS data the module also implement waypoints and pos hold functions.
  * There are 15 Waypoints (#0 is the RTH position), these waypoints can be set via I2C and module gives back the distance and direction towards the active
  * waypoint.
  * Pos hold can overwrite the active waypoint temporary and then can continue toward the last active waypoint.
  * waypoints also can be set to the current gps position,

**Disclaimer : Since I don't see a penny from third party manufacturers i2cgps board versions, i cannot provide support for such boards. The primary supported platform for I2C-GPS-NAV is a pure vanilia arduino mini or nano with direcly attached GPS and sonar modules.**