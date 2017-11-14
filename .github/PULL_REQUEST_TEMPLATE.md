Want to contribute? Great! Make sure you've read, understood and considered all
the points below before creating your PR:

- Keep your PR under 200 lines of code and address a single concern.
- Add unit test(s) and documentation (these do not count toward your 200 lines).
- Adhere to the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).
- Run `ninja test` or `catkin_make_isolated --install --use-ninja --pkg cartographer --make-args test` as appropriate.
- Keep rebasing (or merging) of master branch to a minimum. It triggers Travis
  runs for every update which blocks merging of other changes. Our merge bot
  will rebase your branch, reformat your source code and merge as the last step
  in the review process.
- Please replace this template text with the commit message you want for your
  PR. You and/or the reviewer should keep it updated during the course of the
  review using the GitHub edit feature.
