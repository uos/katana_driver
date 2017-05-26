^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package katana
^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.1 (2017-05-26)
------------------
* Compile katana package with c++11
  This fixes the following warning introduced in adf463b6:
  katana_driver/katana/include/katana/Katana300.h:64:47: warning: non-static data member initializers only available with -std=c++11 or -std=gnu++11 [enabled by default]
  const double JOINTS_STOPPED_POS_TOLERANCE = 0.01;
* Contributors: Martin Günther

1.1.0 (2017-05-26)
------------------
* Fix build with gcc6 and C++11
* Contributors: Jochen Sprickerhof

1.0.8 (2017-05-26)
------------------

1.0.7 (2017-02-11)
------------------
* KNIKinematics: Remove GetKinematicSolverInfo service
  The service definition was removed from moveit_msgs on Kinetic in commit 2f3f97b9.
* Contributors: Martin Guenther

1.0.6 (2017-01-27)
------------------
* Initial release to Kinetic
* Contributors: Martin Günther

1.0.5 (2016-04-12)
------------------

1.0.4 (2016-04-11)
------------------

1.0.3 (2015-06-29)
------------------

1.0.2 (2015-05-06)
------------------

1.0.1 (2015-03-17)
------------------

1.0.0 (2015-03-16)
------------------
* Initial release to Debian packages
* Contributors: Martin Günther, Henning Deeken, Jochen Sprickerhof, Benjamin Reiner, Michael Görner, André Potenza, Frederik Hegger
