^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package katana_arm_gazebo
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.1 (2017-05-26)
------------------

1.1.0 (2017-05-26)
------------------

1.0.8 (2017-05-26)
------------------

1.0.7 (2017-02-11)
------------------

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
* remove dependencies to non-existing cmake targets
* find_package gazebo instead of using PkgConfig
* Contributors: Michael Görner

1.0.2 (2015-05-06)
------------------
* gazebo urdfs: remove self_collide tags
  Due to fixed joint reduction, there is only one link in the resulting
  SDF anyway, so there is nothing to collide with. Apart from that, this
  never made any sense for static models anyway.
* self_collide false for tall table
  The links of the table are statically connected.
  There's no need for collision checking here.
  gazebo5 raises a warning because the self_collide statements
  were inconsistent (not all specified) before.
* Contributors: Martin Günther, Michael Görner

1.0.1 (2015-03-17)
------------------

1.0.0 (2015-03-16)
------------------
* Initial release to Debian packages
* Contributors: Martin Günther, Henning Deeken, Jochen Sprickerhof, Michael Görner, André Potenza, Karl Glatz
