Want to contribute? Great! First, read this page.

### Before you contribute

Any contribution that you make to this repository will
be under the Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0):

```
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
```

### Developer Certificate of Origin

Contributors must sign-off each commit by adding a `Signed-off-by: ...`
line to commit messages to certify that they have the right to submit
the code they are contributing to the project according to the
[Developer Certificate of Origin (DCO)](https://developercertificate.org/).
You can sign-off a commit via `git commit -s`.

### Code reviews

All submissions, including submissions by project members, require review.
We use GitHub pull requests for this purpose. Make sure you've read,
understood and considered all the points below before creating your PR.

#### Style guide

C++ code should adhere to the
[Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html).
You can handle the formatting part of the style guide via `git clang-format`.

#### Best practices

When preparing your PR and also during the code review make sure to follow
[best practices](https://google.github.io/eng-practices/review/developer/).
Most importantly, keep your PR under 200 lines of code and address a single
concern.

#### Testing

- Add unit tests and documentation (these do not count toward your 200 lines).
- Run `ninja test` or `catkin_make_isolated --install --use-ninja --pkg cartographer --make-args test` as appropriate.
- Keep rebasing (or merging) of master branch to a minimum. It triggers Travis
  runs for every update which blocks merging of other changes.
