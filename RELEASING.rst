Steps for Releasing
-------------------

.. code-block:: bash
  catkin_generate_changelog

* Update changelog to point to GitHub release log (e.g.
  https://github.com/cartographer-project/cartographer/compare/0.1.0...0.2.0)

.. code-block:: bash
  git commit -am"Update changelog for release"
  catkin_prepare_release --bump minor --no-push

* Create PR
* Add release via GitHub web UI
