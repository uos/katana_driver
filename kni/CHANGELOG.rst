^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kni
^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.5 (2016-04-12)
------------------

1.0.4 (2016-04-11)
------------------

1.0.3 (2015-06-29)
------------------
* Removed external project handling scheme from CMakeLists.txt.
  The kni package's catkin definitions now do not rely on the original
  project's Makefiles anymore. It now directly handles the build process.
  This has the advantage that resulting targets are no "meta" targets
  anymore. This caused client applications to link against the absolute
  library path from build time, not using the library name + path pattern.
* Contributors: Leon Ziegler

1.0.2 (2015-05-06)
------------------
* Even more KNI dependencies
* Contributors: Jochen Sprickerhof

1.0.1 (2015-03-17)
------------------
* kni: add missing boost dependency in package.xml
* Contributors: Martin Günther

1.0.0 (2015-03-16)
------------------
* Initial release to Debian packages
* Contributors: Martin Günther, Henning Deeken, Jochen Sprickerhof, Michael Görner
