{
  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    utils.url = "github:numtide/flake-utils";
  };
  outputs = {
    self,
    nixpkgs,
    nixpkgs-unstable,
    utils,
  }:
    utils.lib.eachDefaultSystem (
      system: let
        pkgs = nixpkgs.legacyPackages.${system};
        runtimeLibs = with pkgs; [
          # lwjgl
          glfw
          libpulseaudio
          libGL
          openal
          stdenv.cc.cc.lib

          vulkan-loader # VulkanMod's lwjgl

          udev # oshi

          xorg.libX11
          xorg.libXext
          xorg.libXcursor
          xorg.libXrandr
          xorg.libXxf86vm
          xorg.libXrender
          xorg.libXtst
          xorg.libXi
          flite
          libusb1
          wayland
          freetype
          fontconfig
        ];
      in {
        formatter = pkgs.alejandra;
        devShell = pkgs.mkShell {
          packages = with pkgs; [
            glxinfo
            pciutils # need lspci
            xorg.xrandr # needed for LWJGL [2.9.2, 3) https://github.com/LWJGL/lwjgl/issues/128
            jdk21
            (gradle.override {
              javaToolchains = [jdk21];
            })
          ];

          LD_LIBRARY_PATH = "${pkgs.addDriverRunpath.driverLink}/lib:${pkgs.lib.makeLibraryPath runtimeLibs}";
        };
      }
    );
}
