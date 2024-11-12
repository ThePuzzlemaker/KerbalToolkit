{
  description = "Build a cargo workspace";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";

    crane = {
      url = "github:ipetkov/crane";
      inputs.nixpkgs.follows = "nixpkgs";
    };

    fenix = {
      url = "github:nix-community/fenix";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.rust-analyzer-src.follows = "";
    };

    flake-utils.url = "github:numtide/flake-utils";

    advisory-db = {
      url = "github:rustsec/advisory-db";
      flake = false;
    };
  };

  outputs = {
    nixpkgs,
    crane,
    fenix,
    flake-utils,
    ...
  }:
    flake-utils.lib.eachDefaultSystem (system: let
      pkgs = nixpkgs.legacyPackages.${system};

      inherit (pkgs) lib;

      craneLib = (crane.mkLib pkgs).overrideToolchain (fenix.packages.${system}.stable.toolchain);
      src = lib.fileset.toSource {
        root = ./.;
        fileset = lib.fileset.unions [
          ./Cargo.toml
          ./Cargo.lock
          ./src
          ./lib
          ./egui-grid
          ./rust-nlopt
        ];
      };

      commonArgs = {
        inherit src;
        strictDeps = true;

        nativeBuildInputs = [
          pkgs.cmake
          pkgs.gnumake
          pkgs.pkg-config
          pkgs.glib.out
          pkgs.gtk3.out
          pkgs.wrapGAppsHook3
          pkgs.protobuf
          pkgs.makeWrapper
          pkgs.julia
        ];

        buildInputs = [
          pkgs.glib.out
          pkgs.gtk3.out
        ];
      };

      dynamicRuntimeDeps = [
        pkgs.libxkbcommon.out
        pkgs.gtk3.out
        pkgs.glib.out
        pkgs.atk.out
        pkgs.libglvnd.out
        pkgs.libGL.out
        pkgs.xorg.libX11.out
        pkgs.xorg.libXrandr.out
        pkgs.xorg.libXcursor.out
        pkgs.xorg.libXi.out
        pkgs.fontconfig.out
        pkgs.wayland.out
        pkgs.gdk-pixbuf.out
        pkgs.cairo.out
        pkgs.pango.out
        pkgs.gsettings-desktop-schemas.out
        pkgs.alacritty.out
	      pkgs.zlib.out
    pkgs.blas
    pkgs.gcc
    pkgs.gfortran
    pkgs.gfortran.cc.lib
    pkgs.gnum4
    pkgs.lapack
    pkgs.libgccjit
    pkgs.openblas
    pkgs.perl
      ];

      kerbtk-bin = craneLib.buildPackage (commonArgs
        // {
          cargoArtifacts = craneLib.buildDepsOnly commonArgs;
          doCheck = false;
          cargoExtraArgs = "-p kerbtk-bin";
          preBuild = ''
            export JULIA_DIR=${pkgs.julia}
          '';
          postInstall = ''
            wrapProgram $out/bin/kerbtk-bin \
              --suffix LD_LIBRARY_PATH : ${lib.makeLibraryPath dynamicRuntimeDeps}
          '';
        });
    in {
      formatter = pkgs.alejandra;

      packages = {
        inherit kerbtk-bin;
        default = kerbtk-bin;
      };

      apps = {
        kerbtk-bin = flake-utils.lib.mkApp {
          drv = kerbtk-bin;
        };
      };

      devShells.default = craneLib.devShell rec {
        nativeBuildInputs = kerbtk-bin.nativeBuildInputs;

        buildInputs = kerbtk-bin.buildInputs;

        packages =
          nativeBuildInputs
          ++ buildInputs
          ++ dynamicRuntimeDeps
          ++ [
            pkgs.clippy
            pkgs.rustfmt
            pkgs.rust-analyzer
          ];

        LD_LIBRARY_PATH = nixpkgs.lib.makeLibraryPath packages;
        shellHook = ''
          export XDG_DATA_DIRS=${pkgs.gsettings-desktop-schemas}/share/gsettings-schemas/${pkgs.gsettings-desktop-schemas.name}:${pkgs.gtk3}/share/gsettings-schemas/${pkgs.gtk3.name}:$XDG_DATA_DIRS
          export JULIA_DIR=${pkgs.julia}
          export NIX_LD_LIBRARY_PATH=${nixpkgs.lib.makeLibraryPath packages}
        '';
      };
    });
}
