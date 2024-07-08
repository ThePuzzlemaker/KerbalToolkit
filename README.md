# KerbalToolkit

KerbalToolkit is a work-in-progress mission planning and trajectory
design program for Kerbal Space Program.

KerbalToolkit requires [kRPC](https://github.com/krpc/krpc) and the
addon `KRPC.KerbTk` (available
[here](https://git.sr.ht/~thepuzzlemaker/krpc-kerbtk) or
[here](https://github.com/ThePuzzlemaker/krpc-kerbtk)). Additionally,
the patch in
[`0001-Editor-API-for-Part-Introspection.patch`](0001-Editor-API-for-Part-Introspection.patch)
must be applied to kRPC.

Please submit [issues][iss] and [patches][patch] through
[SourceHut](https://git.sr.ht/~thepuzzlemaker/KerbalToolkit).

[iss]: https://todo.sr.ht/~thepuzzlemaker/KerbalToolkit
[patch]: https://lists.sr.ht/~thepuzzlemaker/kerbaltoolkit-devel

## Building

- Ensure that [Rust](https://rust-lang.org) and Cargo are installed. An easy way is using [rustup](https://rustup.rs).
- If on Windows and not using WSL, make sure [Visual Studio](https://visualstudio.microsoft.com/) (with the MSVC C/C++ compiler) is installed.
- Ensure that [CMake](https://cmake.org) and [protoc / protobuf-compiler](https://protobuf.dev) are installed.
- Run `cargo build --release` (for debug builds, you can omit `--release`, however many things are often too slow to reasonably use for testing)
- The resulting binary will be in `target/release/kerbtk-bin` (perhaps with `.exe` on Windows).

## License

This project is licensed under either of

 * Apache License, Version 2.0 ([LICENSES](LICENSES) or
   http://www.apache.org/licenses/LICENSE-2.0)
 * MIT license ([LICENSES](LICENSES) or
   http://opensource.org/licenses/MIT)

at your option.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally
submitted for inclusion in the work by you, as defined in the
Apache-2.0 license, shall be dual licensed as above, without any
additional terms or conditions.
