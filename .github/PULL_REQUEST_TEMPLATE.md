Want to contribute? Great! Make sure you've checked all these boxes before creating your PR:

- [ ] Keep your PR rebased to master.
- [ ] Keep your PR under 200 lines of code and address a single concern.
- [ ] Add unit test(s) and documentation (these do not count toward your 200 lines).
- [ ] Adhere to the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).
- [ ] Run `clang-format -style=Google` on your code (this alone is not enough to ensure you've followed the style guide).
- [ ] Run `ninja test` or `catkin_make_isolated --install --use-ninja --pkg cartographer --make-args test` as appropriate.
